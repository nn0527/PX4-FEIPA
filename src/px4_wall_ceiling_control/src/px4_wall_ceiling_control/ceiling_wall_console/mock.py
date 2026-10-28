"""Explicit offline UI scenarios. No ROS, SSH, Docker or flight transports."""
import copy
import secrets
import threading
import time

from px4_wall_ceiling_control.shared_bridge import SharedBridge
from px4_wall_ceiling_control.wall_console import ConsoleState, validate_config


SCENARIOS = {
    'grounded': '落地待命 / 启动配置',
    'ceiling_ready': '顶吸：POSCTL 入口',
    'ceiling_approach': '顶吸：向上接近',
    'ceiling_hold': '顶吸：保持',
    'ceiling_detach': '顶吸：融合高度下降脱离',
    'ceiling_recovery_locked': '顶吸：恢复锁定 / 等待 CH12',
    'ceiling_returned': '顶吸：POSCTL 已确认',
    'wall_ready': '侧吸：POSCTL 入口',
    'wall_wait': '侧吸：手动接近等待',
    'wall_hold': '侧吸：贴墙保持 / 融合暂停',
    'range_rotating': '离墙：阶段 1 / 等待首帧',
    'range_control': '离墙：阶段 2 / 测距闭环',
    'range_handover': '离墙：阶段 3 / 等待 POSCTL',
    'range_returned': '离墙：POSCTL 已确认',
    'range_fault': '离墙：控制超时',
    'sensor_loss': '离墙：测距失效 / 速度无效',
    'fusion_restore': '离墙：等待光流参数恢复',
    'kill': '急停已触发',
    'stale': '反馈过期',
    'disconnected': '连接中断',
}


class MockRuntime:
    def __init__(self):
        self.state = ConsoleState()
        self.scenario = 'grounded'
        self.running = False
        self.episode_started = time.time()-4.
        self._refresh()

    def select(self, scenario):
        if scenario not in SCENARIOS:
            raise ValueError('未知离线场景')
        self.scenario = scenario
        self.running = scenario != 'grounded'
        self.episode_started = time.time()-4.
        self._refresh()

    def _refresh(self):
        name, now = self.scenario, time.time()
        ceiling = name.startswith('ceiling_')
        range_task = name.startswith('range_') or name in ('sensor_loss', 'fusion_restore')
        returned = name in ('ceiling_returned', 'range_returned')
        active = name not in ('grounded', 'ceiling_ready', 'wall_ready', 'wall_wait') and not returned
        armed = name != 'grounded'
        mode = 'OFFBOARD' if active else 'POSCTL'
        phase = 'WAITING_WALL' if name == 'wall_wait' else 'ACTIVE' if active else 'IDLE'
        owner = (1 if ceiling else 2) if active else 0
        wall_state = {'wall_hold': 'WALL_HOLD', 'range_rotating': 'DETACH_ROTATE',
                      'range_handover': 'EXIT', 'range_fault': 'ABORT'}.get(name, 'RECOVER' if range_task and not returned else 'IDLE')
        task = ('COMPLETED' if returned else 'ROTATING_TO_RANGE_ENTRY' if name == 'range_rotating'
                else 'HANDOVER_READY' if name == 'range_handover' else 'FAULT' if name == 'range_fault'
                else 'RANGE_HEIGHT_CONTROL') if range_task else 'NONE'
        valid = range_task and name not in ('sensor_loss', 'range_rotating')
        target = 1.24
        height = target if name in ('range_handover', 'range_returned') else 1.10
        reason = SCENARIOS[name]+'（离线模拟）'
        wall = dict(state_name=wall_state, reason=reason, fault_detected=name == 'range_fault',
                    fault_recovery_completed=False,
                    hold_achieved=range_task or name == 'wall_hold', reached_state_name=wall_state,
                    detach_completed=name == 'range_returned', recovery_completed=name == 'range_returned',
                    altitude_reference_valid=True, height_above_takeoff_m=2.4, min_entry_height_m=.3, entry_height_ready=True,
                    horizontal_velocity_enabled=True, horizontal_velocity_ready=True, horizontal_velocity_stable=True,
                    horizontal_velocity_active=active, horizontal_velocity_reason='离线模拟水平速度有效',
                    horizontal_velocity_x_enu_mps=.02, horizontal_velocity_y_enu_mps=.01, optical_flow_quality=180,
                    detach_ready=name == 'wall_hold', detach_pending=False, detach_block_reason='',
                    detach_range_height_enabled=True, detach_range_target_locked=range_task and name != 'range_rotating',
                    detach_range_episode=1, detach_range_maneuver_id='offline-demo',
                    detach_task_phase_name=task, detach_range_target_m=target,
                    detach_range_height_m=height if valid else None, detach_range_raw_m=height if valid else None,
                    detach_range_projected_m=height if valid else None,
                    detach_range_velocity_valid=valid, detach_range_velocity_mps=.02 if valid else None,
                    detach_range_velocity_age_s=.05 if valid else None,
                    detach_range_velocity_reason='慢速回归速度有效' if valid else '速度等待重建',
                    detach_range_fast_velocity_valid=valid,
                    detach_range_fast_velocity_mps=.03 if valid else None,
                    detach_range_fast_velocity_age_s=.05 if valid else None,
                    detach_range_fast_velocity_reason='快速相邻高度速度有效' if valid else '快速速度尚未就绪',
                    detach_range_valid=valid, detach_range_age_s=.05 if valid else 1.2,
                    detach_range_source_time=now-.05, detach_range_received_time=now-.03,
                    detach_range_lock_raw_m=1.24, detach_range_lock_height_m=1.24, detach_range_lock_tilt_deg=2.66,
                    detach_range_lock_source_time=self.episode_started, detach_range_lock_received_time=self.episode_started+.02,
                    detach_range_entry_tilt_deg=10., detach_range_sample_tilt_deg=2.66, detach_range_current_tilt_deg=2.66,
                    detach_range_pose_span_s=.08, detach_range_pose_wait_s=.02,
                    detach_range_stable_elapsed_s=1.2 if task == 'HANDOVER_READY' else 0.,
                    detach_range_stable_samples=5 if task == 'HANDOVER_READY' else 0,
                    detach_range_handover_ready=task == 'HANDOVER_READY', detach_range_block_reason=reason,
                    detach_range_target_revision=0,
                    detach_range_target_source='range_first_qualified_message')
        cstate = {'ceiling_approach': 'APPROACH', 'ceiling_hold': 'SURFACE_HOLD', 'ceiling_detach': 'DETACH', 'ceiling_recovery_locked':'RECOVERY_HOVER'}.get(name, 'NORMAL_FLIGHT')
        fusion_phase = 'SUSPENDED' if name == 'wall_hold' else 'RESTORING' if name in ('range_rotating', 'fusion_restore') else 'READY'
        values = dict(runtime=dict(enabled=self.running, ceiling_enabled=True, wall_pilot_mode='POSCTL',
                                   ceiling_motion_enabled=True, ceiling_distance_gate_enabled=True, wall_range_faults_diagnostic_only=True, logger=self.running),
                      state=dict(connected=True, armed=armed, mode=mode),
                      operator=dict(mode=2 if ceiling and active else 3 if owner == 2 or name == 'wall_wait' else 1,
                                    enabled=active or name == 'wall_wait'),
                      phase=phase, owner=owner, wall=wall, reason=reason,
                      odom=dict(finite=True, vertical_speed=-.08, angles=[0., 2.66, 0.]),
                      estimator=dict(attitude=True, vertical_velocity=True, horizontal_velocity=True),
                      health=dict(ceiling_valid=True, front_valid=True), ceiling=dict(range_m=.4), front=dict(range_m=1.0),
                      kill=dict(valid=True, active=name == 'kill', channel=7), setpoint=dict(thrust=.49),
                      battery=dict(voltage=15.8), rc=dict(channels=[1500]*8),
                      ceiling_status=dict(rc_manual_coordination_enabled=True, rc_manual_feedback_ready=True, rc_manual_active=False, rc_position_weight=1., hold_elapsed_s=1.5, hold_timeout_s=0., baseline_ready=True,detach_target_drop_m=.40,detach_max_drop_m=1.50,vertical_integral_thrust=.02,vertical_support_thrust=.49,vertical_proportional_thrust=.01, recovery_ready=name == 'ceiling_recovery_locked', recovery_attitude_locked=name == 'ceiling_recovery_locked', recovery_lock_time=now-.5 if name == 'ceiling_recovery_locked' else 0., motion_control_enabled=True, motion_ready=True, motion_paused=False, horizontal_drift_m=.04, horizontal_speed_mps=.02, actual_tilt_deg=2., actual_body_rate_deg_s=3., motion_reason="离线模拟：水平稳定", state_name=cstate, reason=reason, fault_detected=False, odom_fresh=True,
                                          height_source='px4_fused_hold_relative', height_feedback_valid=True, height_reference_ready=True, fused_height_m=1.7 if cstate == 'DETACH' else 1.6 if cstate == 'RECOVERY_HOVER' else 2., hold_fused_height_m=2., fused_vertical_speed_mps=-.08, baro_descended_height_m=.30, baro_fresh=True, baro_reference_ready=True,
                                          range_fresh=True, ceiling_distance_filtered_m=.056 if cstate == 'SURFACE_HOLD' else .4,
                                          descended_height_m=.3 if cstate == 'DETACH' else .4 if name in ('ceiling_returned','ceiling_recovery_locked') else 0., detach_height_ready=name == 'ceiling_recovery_locked'),
                      wall_sensor_fusion=dict(enabled=True, phase=fusion_phase, measured_tilt_deg=2.66,
                                              control_ready=fusion_phase == 'READY', parameters_restored=fusion_phase == 'READY',
                                              original_flow=1, original_range=1, target_flow=0 if fusion_phase == 'SUSPENDED' else 1,
                                              target_range=0 if fusion_phase == 'SUSPENDED' else 1, confirmed_flow=1, confirmed_range=1,
                                              reason='离线模拟：80°请求恢复，等待读回与应用缓冲' if fusion_phase == 'RESTORING' else reason))
        values['rc']['channels'][7] = 999 if ceiling else 1999
        values['rc_selected_mode'] = 'ceiling' if ceiling else 'wall'
        values['ceiling_failsafe'] = dict(verified=True,connected=True,parameters={
            'COM_OBL_RC_ACT':dict(actual=4,expected=4), 'COM_OF_LOSS_T':dict(actual=2.,expected=2.)},
            reason='离线模拟值，未读取或写入任何真实飞控参数')
        for key, value in values.items():
            self.state.update(key, value)

    def start(self, config):
        if self.state.snapshot()['status']['state']['value']['armed']:
            raise ValueError('离线场景仍为已解锁，不能重新配置启动')
        self.state.config = validate_config(config)
        self.running = True
        self._refresh()
        return {'message': '离线模拟：基础程序已启动，仍为未解锁状态'}

    def _action(self, permission, scenario, message):
        self._refresh()
        if not self.state.snapshot()[permission]:
            raise ValueError('当前离线场景不满足动作条件')
        self.select(scenario)
        return {'message': '离线模拟：'+message}

    def ceiling(self):
        return self._action('can_ceiling', 'ceiling_approach', '顶吸请求已提交，等待场景反馈')

    def ceiling_detach(self):
        return self._action('can_ceiling_detach', 'ceiling_detach', '进入融合高度下降脱离')

    def wall(self):
        return self._action('can_wall', 'wall_wait', '进入手动接近等待')

    def detach(self):
        return self._action('can_detach', 'range_rotating', '开始离墙翻转')

    def cancel_wait(self):
        return self._action('can_cancel_wait', 'wall_ready', '已取消贴墙等待')


    def stop(self, config=None):
        if self.state.snapshot()['status']['state']['value']['armed']:
            raise ValueError('离线场景仍为已解锁，请选择落地待命')
        self.select('grounded')
        return {'message': '离线模拟：基础链路已停止'}

    def repair_mavros(self, config):
        if not config.get('grounded_confirmed') or self.state.snapshot()['status']['state']['value']['armed']:
            raise ValueError('连接修复需要落地未解锁确认')
        return {'message': '离线模拟：连接修复完成，没有启动外部进程'}


class MockHub:
    def __init__(self):
        self.lock = threading.RLock()
        self.runtime = MockRuntime()
        self.broker = SharedBridge(self.runtime)

    def connect(self, label):
        return MockTransport(self, label)


class MockTransport:
    def __init__(self, hub, label):
        self.hub, self.identifier = hub, secrets.token_hex(16)
        self.last_snapshot = None
        self.closed = False
        hub.broker.connect(self.identifier, label)

    def status(self, config):
        with self.hub.lock:
            runtime = self.hub.runtime
            meta = self.hub.broker.metadata(self.identifier)
            runtime._refresh()
            snapshot = runtime.state.snapshot()
            live = runtime.scenario not in ('disconnected', 'stale') and meta['push']
            if not meta['push'] and self.last_snapshot is not None:
                snapshot = copy.deepcopy(self.last_snapshot)
            if not live:
                for item in snapshot['status'].values():
                    item['fresh'] = False
                for key in list(snapshot):
                    if key.startswith('can_'):
                        snapshot[key] = False
            if not meta['control']:
                for key in list(snapshot):
                    if key.startswith('can_'):
                        snapshot[key] = False
            snapshot.update(session=dict(meta, feedback_live=live),
                            ssh=dict(connected=runtime.scenario != 'disconnected', host='offline · 无设备连接', error=''),
                            offline=True, mock_scenarios=SCENARIOS, mock_scenario=runtime.scenario)
            self.last_snapshot = copy.deepcopy(snapshot)
            return snapshot

    def request(self, action, config=None):
        with self.hub.lock:
            if action == 'mock_scenario':
                meta = self.hub.broker.metadata(self.identifier)
                if not meta['control'] or not meta['push']:
                    raise ValueError('选择离线场景需要控制权和开启反馈')
                self.hub.runtime.select((config or {}).get('scenario'))
                return {'message': '已切换离线模拟场景；没有发送飞行指令'}
            if action not in ('acquire', 'release', 'push', 'stop') and self.hub.runtime.scenario in ('stale', 'disconnected'):
                raise ValueError('离线场景反馈中断，动作禁用')
            return self.hub.broker.request(self.identifier, action, config)

    def close(self):
        with self.hub.lock:
            self.closed = True
            self.hub.broker.disconnect(self.identifier)
