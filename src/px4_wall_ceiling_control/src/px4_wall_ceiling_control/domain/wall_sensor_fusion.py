"""Side-wall EKF parameter transactions; no ROS imports or control-loop work.

Acknowledged parameters are deliberately not called "fused".  This policy only
reports permission for consumers to start their own fresh-data confirmation.
All service calls are represented by one outstanding Operation.  A timeout does
not release that slot: a late disable must complete before a restore is sent.
"""
import json
import math
import os
import tempfile
import uuid
from dataclasses import dataclass

FLOW = "EKF2_OF_CTRL"
RANGE = "EKF2_RNG_CTRL"
PARAMETERS = (FLOW, RANGE)
RECOVERY_STATES = ("DETACH_ROTATE", "RECOVER", "ABORT")


def total_tilt_deg(quaternion):
    """Angle of body +Z from world +Z, quaternion in x,y,z,w order."""
    values = tuple(float(v) for v in quaternion)
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError("attitude quaternion is not finite")
    norm2 = sum(v * v for v in values)
    # MAVROS orientation is a unit quaternion.  Do not normalize corrupt data
    # into an apparently usable orientation; tolerate ordinary floating error.
    if not 0.98 ** 2 <= norm2 <= 1.02 ** 2:
        raise ValueError("attitude quaternion is not normalized")
    x, y, _, _ = (v / math.sqrt(norm2) for v in values)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x*x + y*y)))))


class FreshSample:
    """Check both source and receipt clocks without rebasing on replay."""
    def __init__(self, timeout, future_tolerance=0.1):
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self.source = self.receipt = self.monotonic = None
        self.valid = False

    def observe(self, source, receipt, monotonic):
        if (not all(math.isfinite(v) for v in (source, receipt, monotonic))
                or source <= 0 or (self.source is not None and source <= self.source)
                or (self.receipt is not None and receipt < self.receipt)
                or receipt - source > self.timeout
                or source - receipt > self.future_tolerance + 1e-9):
            self.valid = False
            return False
        self.source, self.receipt, self.monotonic = source, receipt, monotonic
        self.valid = True
        return True

    def fresh(self, now, monotonic):
        return bool(self.valid and self.source is not None
                    and -self.future_tolerance - 1e-9 <= now-self.source <= self.timeout
                    and 0 <= now-self.receipt <= self.timeout
                    and 0 <= monotonic-self.monotonic <= self.timeout)


class RestoreJournal:
    """A durable original-value record, written before any parameter mutation."""
    def __init__(self, path, vehicle_id):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.vehicle_id = vehicle_id

    def load(self):
        try:
            with open(self.path, "r") as stream:
                data = json.load(stream)
        except FileNotFoundError:
            return None
        if (not isinstance(data, dict) or data.get("version") != 1 or data.get("vehicle_id") != self.vehicle_id
                or not isinstance(data.get("episode_id"), str)
                or not data["episode_id"] or data.get("desired") not in ("suspend", "restore")
                or not isinstance(data.get("originals"), dict)
                or set(data["originals"]) != set(PARAMETERS)
                or any(type(data["originals"][key]) is not int
                       or not 0 <= data["originals"][key] <= 2147483647 for key in PARAMETERS)):
            raise ValueError("invalid restore journal or vehicle identity mismatch")
        pending = data.get("pending")
        if pending is not None and (not isinstance(pending, dict)
                or type(pending.get("token")) is not int or pending["token"] <= 0
                or pending.get("parameter") not in PARAMETERS
                or type(pending.get("value")) is not int or not 0 <= pending["value"] <= 2147483647
                or type(pending.get("request_stamp")) not in (float, int)
                or not math.isfinite(pending["request_stamp"]) or pending["request_stamp"] < 0):
            raise ValueError("invalid pending parameter request in restore journal")
        return data

    def save(self, data):
        directory = os.path.dirname(self.path)
        os.makedirs(directory, mode=0o700, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=".fusion-", dir=directory)
        try:
            with os.fdopen(fd, "w") as stream:
                json.dump(data, stream, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            self._sync_directory(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def clear(self):
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            return
        self._sync_directory(os.path.dirname(self.path))

    @staticmethod
    def _sync_directory(directory):
        fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


@dataclass(frozen=True)
class Operation:
    token: int
    kind: str
    revision: int
    issued_at: float
    issued_mono: float
    parameter: str = ""
    value: int = -1


class FusionPolicy:
    """Serialized, retryable transactions with a durable restore obligation.

    Call observe_* / tick / complete under one node lock.  Execute the returned
    Operation in a different thread, then pass its result to complete.  Snapshot
    and verify operations require named, fresh FCU readback of both parameters.
    """
    def __init__(self, journal, vehicle_id, enabled=False, restore_tilt_deg=80.,
                 application_settle_s=1.2, odom_timeout_s=.3,
                 status_timeout_s=.6, state_timeout_s=1.5,
                 future_tolerance_s=.1, request_timeout_s=5.,
                 retry_initial_s=1., retry_max_s=8., restart_quarantine_s=10.):
        limits = (restore_tilt_deg, application_settle_s, odom_timeout_s,
                  status_timeout_s, state_timeout_s, request_timeout_s,
                  retry_initial_s, retry_max_s, future_tolerance_s, restart_quarantine_s)
        if (not all(math.isfinite(v) for v in limits)
                or not 0 < restore_tilt_deg <= 90
                or application_settle_s < 0
                or min(odom_timeout_s, status_timeout_s, state_timeout_s,
                       request_timeout_s, retry_initial_s) <= 0
                or retry_max_s < retry_initial_s or restart_quarantine_s < 10.
                or not 0 <= future_tolerance_s <= .1):
            raise ValueError("invalid fusion parameter management limits")
        if not vehicle_id:
            raise ValueError("a non-empty vehicle identity is required")
        self.journal, self.vehicle_id, self.enabled = journal, vehicle_id, enabled
        self.restore_tilt_deg, self.settle_s = restore_tilt_deg, application_settle_s
        self.request_timeout_s = request_timeout_s
        self.retry_initial_s, self.retry_max_s = retry_initial_s, retry_max_s
        self.restart_quarantine_s = restart_quarantine_s
        self.restart_quarantine_until = None
        self.restart_pending = False
        self.recovered_pending = None
        self.restarted_obligation = False
        self.odom = FreshSample(odom_timeout_s, future_tolerance_s)
        self.wall = FreshSample(status_timeout_s, future_tolerance_s)
        self.fcu = FreshSample(state_timeout_s, future_tolerance_s)
        self.wall_state = None
        self.connected = False
        self.armed = True
        self.kill_active = False
        self.emergency_restore = False
        self.tilt = math.nan
        self.previous_tilt = math.nan
        self.last_orientation_receipt = self.last_orientation_mono = None
        self.recovery_entry_pending = False
        self.recovery_seen_odom = None
        self.recovery_threshold_event = None
        self.idle_pending = False
        self.originals, self.confirmed = {}, {}
        self.episode_id = ""
        self.restore_required = False
        self.restore_intent = False
        self.restored = False
        self.cycle_consumed = False
        self.disable_done, self.restore_done = set(), set()
        self.settle_until = None
        self.ready_after = 0.
        self.request_stamp = self.confirmed_stamp = 0.
        self.in_flight = None
        self.operation_stage = 'executing'
        self.execution_started_mono = None
        self.refresh_on_arm = False
        self.revision = self.sequence = 0
        self.retry_count = self.failures = 0
        self.next_attempt = 0.
        self.error = ""
        self.fatal = False
        self.startup_reconcile = False
        try:
            saved = journal.load()
            if saved:
                self.originals = dict(saved["originals"])
                self.episode_id = saved["episode_id"]
                self.restore_required = self.cycle_consumed = True
                self.restore_intent = saved["desired"] == "restore"
                # Never infer an ACK from a saved target or from a closed value.
                self.startup_reconcile = True
                self.restarted_obligation = True
                self.recovered_pending = saved.get("pending")
                self.restart_pending = self.recovered_pending is not None
                self.error = "pending restore journal loaded; waiting for fresh vehicle state"
        except (OSError, ValueError, TypeError) as exc:
            self.fatal = True
            self.error = "restore journal unreadable: {}".format(exc)

    def observe_odom(self, quaternion, source, now, monotonic):
        previous_fresh = self.odom.fresh(now, monotonic)
        reacquiring = (self.last_orientation_receipt is None
                       or now-self.last_orientation_receipt > self.odom.timeout
                       or monotonic-self.last_orientation_mono > self.odom.timeout)
        if not self.odom.observe(source, now, monotonic):
            return False
        try:
            tilt = total_tilt_deg(quaternion)
        except (TypeError, ValueError, OverflowError):
            self.odom.valid = False
            return False
        self.previous_tilt, self.tilt = self.tilt if previous_fresh else math.nan, tilt
        self.last_orientation_receipt, self.last_orientation_mono = now, monotonic
        if (self.restore_required and not self.restore_intent
                and self.wall_state in RECOVERY_STATES
                and self.wall.fresh(now, monotonic)):
            if reacquiring:
                # A real telemetry gap can conceal the whole return to level.
                # Treat its first fresh measured attitude as recovery re-entry;
                # stale frames themselves never authorize a request.
                self.recovery_entry_pending = True
            self._observe_recovery_threshold(now, monotonic)
            # Capture qualified evidence at the sample rate. A delayed timer
            # must not lose a crossing while subsequent fresh poses stay level.
            # This changes only intent; the worker still serializes all writes.
            self._update_restore_intent(now, monotonic)
        return True

    def observe_wall(self, state, source, now, monotonic):
        if not self.wall.observe(source, now, monotonic):
            return False
        if state != self.wall_state:
            if state == "IDLE":
                self.idle_pending = True
            if state in RECOVERY_STATES:
                self.recovery_entry_pending = True
                self.recovery_seen_odom = None
                self.recovery_threshold_event = None
            self.wall_state = state
            if state in RECOVERY_STATES and self.odom.fresh(now, monotonic):
                self._observe_recovery_threshold(now, monotonic)
                self._update_restore_intent(now, monotonic)
        return True

    def observe_fcu(self, connected, armed, source, now, monotonic):
        rearmed = not self.armed and bool(armed)
        if not self.fcu.observe(source, now, monotonic):
            return False
        self.connected, self.armed = bool(connected), bool(armed)
        if rearmed and not self.restore_required and self.originals:
            self.refresh_on_arm = True
        return True

    def observe_kill(self, active, monotonic):
        newly_active = active and not self.kill_active
        self.kill_active = active
        if not newly_active or self.emergency_restore:
            return
        self.emergency_restore = True
        self.originals = {FLOW: 1, RANGE: 1}
        self.episode_id = self.episode_id or uuid.uuid4().hex
        self.cycle_consumed = self.restore_required = True
        self.restored = False
        self.startup_reconcile = True
        self.restore_intent = False
        # Supersede queued writes while keeping any dispatched write in its
        # serial slot. Reconcile only after that write has returned.
        self._request_restore(monotonic)

    def _vehicle_fresh(self, now, monotonic):
        return self.connected and self.fcu.fresh(now, monotonic)

    def _suspension_allowed(self, now, monotonic):
        return bool(not self.kill_active and self._vehicle_fresh(now, monotonic) and self.armed
                    and self.wall.fresh(now, monotonic)
                    and self.wall_state == "WALL_HOLD")

    def _update_restore_intent(self, now, monotonic):
        if self.restore_required and not self.restore_intent:
            if ((self._vehicle_fresh(now, monotonic) and not self.armed)
                    or self._recovery_allows_restore(now, monotonic)):
                self._request_restore(monotonic)

    def authorize_operation(self, operation, now, monotonic):
        """Recheck live intent immediately before work and after durable I/O.

        The worker calls this under the node lock. A recovery sample can arrive
        between policy ticks or while fsync is pending; it must invalidate a
        queued disable before that disable reaches the FCU.
        """
        self._update_restore_intent(now, monotonic)
        if self.in_flight != operation or operation.revision != self.revision:
            return False
        if not self._vehicle_fresh(now, monotonic):
            raise RuntimeError('fresh FCU state unavailable before parameter dispatch')
        if operation.kind == 'set' and not self.restore_intent:
            return self._suspension_allowed(now, monotonic)
        return True

    def _record(self, pending=None):
        return {"version": 1, "vehicle_id": self.vehicle_id,
                "episode_id": self.episode_id, "originals": self.originals,
                "desired": "restore" if self.restore_intent else "suspend",
                "pending": pending if pending is not None else self.recovered_pending}

    def _save(self, monotonic, pending=None):
        try:
            self.journal.save(self._record(pending))
            return True
        except (OSError, ValueError) as exc:
            self._failed("cannot persist restore journal: {}".format(exc), monotonic)
            return False

    def _failed(self, reason, monotonic):
        self.error = reason
        self.retry_count += 1
        self.failures += 1
        self.next_attempt = monotonic + min(self.retry_max_s,
                                           self.retry_initial_s * 2 ** min(self.failures-1, 8))

    def _operation_failed(self, operation, reason, monotonic):
        self._failed(reason, monotonic)
        if (operation.kind == 'set' and self.restore_intent
                and operation.revision != self.revision):
            # An old disable may genuinely fail after recovery was requested.
            # Keep its diagnostic and unknown parameter evidence, but do not
            # count its retry delay against the new restoration transaction.
            self.failures = 0
            self.next_attempt = monotonic

    def _operation(self, kind, now, monotonic, parameter="", value=-1):
        self.sequence += 1
        operation = Operation(self.sequence, kind, self.revision, now, monotonic,
                              parameter, value)
        if kind == "set" and not self._save(monotonic, {
                "token": operation.token, "parameter": parameter, "value": value,
                "request_stamp": now}):
            return None
        self.in_flight = operation
        self.operation_stage = 'executing'
        self.execution_started_mono = monotonic
        self.request_stamp = now
        return operation

    def mark_operation_stage(self, operation, stage, now, monotonic):
        if self.in_flight != operation:
            return False
        self.operation_stage = stage
        if stage == 'executing':
            self.execution_started_mono = monotonic
            self.request_stamp = now
        else:
            self.execution_started_mono = None
        return True

    def _request_restore(self, monotonic):
        if self.restore_intent:
            return
        self.restore_intent = True
        self.revision += 1
        self.restore_done.clear()
        self.settle_until = None
        # Disable failures belong to the old intent. Recovery must not inherit
        # their exponential retry delay; an actual in-flight write still owns
        # the serial slot until its worker returns.
        self.error = ""
        self.failures = 0
        self.next_attempt = monotonic
        # Keep any outstanding set in the journal until its actual return.
        op = self.in_flight
        pending = ({"token": op.token, "parameter": op.parameter,
                    "value": op.value, "request_stamp": op.issued_at}
                   if op and op.kind == "set" else None)
        self._save(monotonic, pending)

    def _observe_recovery_threshold(self, now, monotonic):
        # This runs at the odometry rate, so a crossing followed by equal-angle
        # samples cannot be lost between two 10 Hz policy timer callbacks.
        if self.recovery_seen_odom == self.odom.source:
            return
        self.recovery_seen_odom = self.odom.source
        at_entry = self.recovery_entry_pending
        self.recovery_entry_pending = False
        toward_level = math.isfinite(self.previous_tilt) and self.tilt < self.previous_tilt
        if self.tilt <= self.restore_tilt_deg + 1e-9 and (at_entry or toward_level):
            self.recovery_threshold_event = (self.odom.source, now, monotonic)

    def _recovery_allows_restore(self, now, monotonic):
        if not self.wall.fresh(now, monotonic) or not self.odom.fresh(now, monotonic):
            return False
        # When the whole decision stack restarted, its initial IDLE/EXIT no
        # longer carries the previous DETACH state.  The journal proves that
        # this is recovery of an old suppression, never a new maneuver.
        if self.restarted_obligation and self.wall_state in ("IDLE", "EXIT"):
            return self.tilt <= self.restore_tilt_deg + 1e-9
        if self.wall_state not in RECOVERY_STATES:
            return False
        self._observe_recovery_threshold(now, monotonic)
        event = self.recovery_threshold_event
        return bool(event and self.tilt <= self.restore_tilt_deg + 1e-9
                    and -self.odom.future_tolerance <= now-event[0] <= self.odom.timeout
                    and 0 <= now-event[1] <= self.odom.timeout
                    and 0 <= monotonic-event[2] <= self.odom.timeout)

    def tick(self, now, monotonic):
        """Return at most one operation, never abandoning an unresolved call."""
        if self.fatal:
            return None
        vehicle_fresh = self._vehicle_fresh(now, monotonic)
        # Latch qualifying attitude even while disconnected; execution still
        # requires a fresh connected FCU, and an unresolved write stays serial.
        self._update_restore_intent(now, monotonic)
        if self.restart_pending:
            # Deployed MAVROS 1.20.1 waits at most 1 s * (3 + 2) for a
            # PARAM_SET.  Ten monotonic seconds drain a previous process's
            # request before a new forced pull / restore; do not lower this
            # for that implementation.  The old pending record survives any
            # restart during this quarantine, even if restore intent changes.
            if self.restart_quarantine_until is None:
                self.restart_quarantine_until = monotonic + self.restart_quarantine_s
            if monotonic < self.restart_quarantine_until:
                self.error = "restart quarantine: draining prior MAVROS 1.20.1 parameter request"
                return None
            self.restart_pending = False
            self.error = ""
        if self.in_flight is not None:
            if (self.execution_started_mono is not None
                    and monotonic-self.execution_started_mono > self.request_timeout_s):
                self.error = "{} timed out; waiting for unresolved service call".format(self.in_flight.kind)
            return None
        if monotonic < self.next_attempt or not vehicle_fresh:
            return None
        if self.restore_required:
            if self.startup_reconcile:
                # Read-only reconciliation before touching a recovered journal.
                return self._operation("reconcile", now, monotonic)
            if self.restore_intent:
                for key in (RANGE, FLOW):
                    if key not in self.restore_done:
                        return self._operation("set", now, monotonic, key, self.originals[key])
                if self.settle_until is None:
                    self.settle_until = monotonic + self.settle_s
                if monotonic >= self.settle_until:
                    return self._operation("verify", now, monotonic)
            else:
                # The same live HOLD guard applies to scheduling and dispatch,
                # including partial suspension in a normal, unrestarted cycle.
                # Outside HOLD keep the restore obligation, but never enqueue
                # another disable that dispatch would immediately cancel.
                if not self._suspension_allowed(now, monotonic):
                    return None
                for key in (FLOW, RANGE):
                    if key not in self.disable_done:
                        return self._operation("set", now, monotonic, key, 0)
            return None
        if not self.enabled or self.kill_active:
            return None
        if not self.wall.fresh(now, monotonic):
            return None
        if self.idle_pending and self.wall_state == "IDLE":
            self.idle_pending = False
            self.cycle_consumed = False
            self.disable_done.clear()
            self.restore_done.clear()
            self.restore_intent = False
            self.error = ""
        if self.refresh_on_arm and self.wall_state == "IDLE":
            self.refresh_on_arm = False
            # Keep the verified snapshot visible until this fresh read completes.
            return self._operation('snapshot', now, monotonic)
        if not self.originals:
            if self.wall_state == "IDLE":
                return self._operation("snapshot", now, monotonic)
            self.error = "no safe IDLE snapshot; parameter writes inhibited"
            return None
        if self._suspension_allowed(now, monotonic) and not self.cycle_consumed:
            self.episode_id = uuid.uuid4().hex
            self.cycle_consumed = True
            self.restore_required = True
            self.restored = False
            self.restore_intent = False
            self.revision += 1
            self.ready_after = 0.
            self.disable_done.clear()
            self.restore_done.clear()
            # _operation persists originals before the very first PARAM_SET.
            return self._operation("set", now, monotonic, FLOW, 0)
        return None

    def complete(self, operation, success, values, now, monotonic, reason=""):
        """Apply a worker return.  Old ACKs update evidence, never current intent."""
        if self.in_flight != operation:
            return False
        self.in_flight = None
        self.execution_started_mono = None
        self.operation_stage = ''
        if self.emergency_restore and operation.revision != self.revision:
            # Even an acknowledged old snapshot must not replace the emergency
            # target. The fresh reconcile read will account for old writes.
            return True
        if not success:
            if values.get('cancelled') is True:
                # No FCU write occurred. Do not erase confirmed evidence or
                # classify a same-revision state change as a transport failure.
                self.error = ''
                self.next_attempt = monotonic
                return True
            if operation.kind == "set":
                self.confirmed.pop(operation.parameter, None)
            self._operation_failed(operation, reason or "{} failed".format(operation.kind), monotonic)
            return True
        if operation.kind == "set":
            value = values.get(operation.parameter)
            if type(value) is not int or value != operation.value:
                self.confirmed.pop(operation.parameter, None)
                self._operation_failed(operation, "PARAM_SET acknowledgement value mismatch", monotonic)
                return True
            self.confirmed[operation.parameter] = value
            self.confirmed_stamp = now
            if operation.revision == self.revision:
                if self.restore_intent:
                    self.restore_done.add(operation.parameter)
                    if self.restore_done == set(PARAMETERS):
                        self.settle_until = monotonic + self.settle_s
                else:
                    self.disable_done.add(operation.parameter)
            # A late disable ACK cannot satisfy the restoration barrier.
            if not self._save(monotonic):
                return True
        else:
            if (set(values) != set(PARAMETERS)
                    or any(type(values[k]) is not int or not 0 <= values[k] <= 2147483647
                           for k in PARAMETERS)):
                self._failed("missing or invalid integer parameter readback", monotonic)
                return True
            self.confirmed = dict(values)
            self.confirmed_stamp = now
            if operation.kind == "snapshot":
                self.originals = dict(values)
                self.restored = True
                self.ready_after = now
            elif operation.kind == "reconcile":
                self.startup_reconcile = False
                self.recovered_pending = None
                if self.emergency_restore:
                    # Preserve values already at the requested setting.
                    self.restore_done = {key for key in PARAMETERS if values[key] == self.originals[key]}
                # Normal restart recovery still explicitly acknowledges both writes.
            elif operation.kind == "verify":
                if values != self.originals:
                    self.restore_done.clear()
                    self.settle_until = None
                    self._failed("post-settle parameter readback differs from originals", monotonic)
                    return True
                try:
                    self.journal.clear()
                except OSError as exc:
                    self._failed("cannot clear restore journal: {}".format(exc), monotonic)
                    return True
                self.restore_required = False
                self.restarted_obligation = False
                self.emergency_restore = False
                self.restored = True
                self.ready_after = now
        self.error = ""
        self.failures = 0
        self.next_attempt = monotonic
        return True

    def status(self, now, monotonic):
        active = self.enabled or self.restore_required or self.fatal
        connected = self._vehicle_fresh(now, monotonic)
        if not active:
            phase = "DISABLED"
        elif self.fatal or self.error or not connected:
            phase = "ERROR" if self.fatal or self.error or self.restore_required else "INITIALIZING"
        elif not self.originals:
            phase = "INITIALIZING"
        elif not self.restore_required:
            phase = "READY"
        elif self.restore_intent:
            phase = "SETTLING" if self.restore_done == set(PARAMETERS) else "RESTORING"
        else:
            phase = "SUSPENDED" if self.disable_done == set(PARAMETERS) else "DISABLING"
        target = (self.originals if not self.restore_required or self.restore_intent
                  else {FLOW: 0, RANGE: 0})
        ready = (not active or (phase == "READY" and self.in_flight is None
                              and connected and self.wall.fresh(now, monotonic)))
        reason = self.error
        if not reason and self.in_flight is not None and self.operation_stage != 'executing':
            reason = 'parameter transaction: '+self.operation_stage
        if (not reason and self.restore_required and not self.restore_intent
                and self.in_flight is None and not self._suspension_allowed(now, monotonic)):
            reason = 'suspension paused: waiting for fresh HOLD or qualified recovery evidence'
        if active and not connected:
            reason = reason or "waiting for fresh connected FCU state"
        if phase == "READY" and not self.wall.fresh(now, monotonic):
            reason = "waiting for fresh wall state"
        return dict(enabled=active, phase=phase, episode_id=self.episode_id,
                    snapshot_valid=bool(self.originals), restore_required=self.restore_required,
                    parameters_restored=self.restored and not self.restore_required,
                    control_ready=ready, original_flow=self.originals.get(FLOW, -1),
                    original_range=self.originals.get(RANGE, -1),
                    target_flow=target.get(FLOW, -1), target_range=target.get(RANGE, -1),
                    confirmed_flow=self.confirmed.get(FLOW, -1),
                    confirmed_range=self.confirmed.get(RANGE, -1),
                    request_stamp=self.request_stamp, confirmed_stamp=self.confirmed_stamp,
                    ready_after=self.ready_after, measured_tilt_deg=self.tilt,
                    retry_count=self.retry_count, reason=reason)
