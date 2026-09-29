"""ROS-independent owner and handoff interlock state machine."""

from dataclasses import dataclass


OWNER_NONE = 0
OWNER_CEILING = 1
OWNER_WALL = 2


@dataclass(frozen=True)
class CandidateIntent:
    enabled: bool = False
    fresh: bool = False
    acquire_request: bool = False
    keep_ownership: bool = False
    candidate_valid: bool = False
    required_operator_mode: int = -1


@dataclass(frozen=True)
class ArbitrationEvent:
    owner: int
    interlock_active: bool
    interlock_started: bool = False
    interlock_cleared: bool = False
    conflict: bool = False
    reason: str = ""


class OwnerArbiter:
    """Own exactly one flight-output producer with neutral handoff dwell."""

    def __init__(self, handoff_dwell_s, owner=OWNER_NONE):
        self.handoff_dwell_s = max(0.0, float(handoff_dwell_s))
        self.owner = owner
        self.interlock_active = False
        self.interlock_clear_since_s = None

    def restore(self, owner, interlock_active, interlock_clear_since_s):
        """Restore state owned by an older ROS wrapper during migration/tests."""

        self.owner = owner
        self.interlock_active = bool(interlock_active)
        self.interlock_clear_since_s = interlock_clear_since_s

    def _event(self, **changes):
        values = {
            "owner": self.owner,
            "interlock_active": self.interlock_active,
        }
        values.update(changes)
        return ArbitrationEvent(**values)

    def _start_interlock(self, reason, conflict=False):
        already_active = self.interlock_active
        self.owner = OWNER_NONE
        self.interlock_active = True
        self.interlock_clear_since_s = None
        return self._event(
            interlock_started=not already_active,
            conflict=conflict,
            reason=reason,
        )

    def update(self, now_s, operator_mode, operator_neutral, ceiling, wall):
        if self.interlock_active:
            self.owner = OWNER_NONE
            if not operator_neutral:
                self.interlock_clear_since_s = None
                return self._event()
            if self.interlock_clear_since_s is None:
                self.interlock_clear_since_s = now_s
                return self._event()
            if now_s - self.interlock_clear_since_s < self.handoff_dwell_s:
                return self._event()
            self.interlock_active = False
            self.interlock_clear_since_s = None
            return self._event(interlock_cleared=True)

        if self.owner == OWNER_CEILING:
            if ceiling.enabled and ceiling.fresh and ceiling.keep_ownership:
                return self._event()
            return self._start_interlock(
                "ceiling ownership ended or candidate timed out"
            )

        if self.owner == OWNER_WALL:
            if wall.enabled and wall.fresh and wall.keep_ownership:
                return self._event()
            return self._start_interlock("wall ownership ended or candidate timed out")

        ceiling_request = bool(
            ceiling.enabled
            and ceiling.fresh
            and ceiling.acquire_request
            and ceiling.candidate_valid
            and operator_mode == ceiling.required_operator_mode
        )
        wall_request = bool(
            wall.enabled
            and wall.fresh
            and wall.acquire_request
            and wall.candidate_valid
            and operator_mode == wall.required_operator_mode
        )
        if ceiling_request and wall_request:
            return self._start_interlock(
                "conflicting acquisition requests", conflict=True
            )
        if ceiling_request:
            self.owner = OWNER_CEILING
        elif wall_request:
            self.owner = OWNER_WALL
        return self._event()
