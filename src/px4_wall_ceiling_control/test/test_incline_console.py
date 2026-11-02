import copy
import unittest
from unittest.mock import Mock
from test_wall_console import server

class InclineConsoleTests(unittest.TestCase):
    def setUp(self):
        values = dict(runtime=dict(enabled=True), state=dict(connected=True, armed=True, mode='POSCTL'), wall=dict(incline_ready=True), operator=dict(mode=1), phase='IDLE', owner=0, kill=dict(valid=True, active=False), odom={}, estimator={})
        self.data = {'status': {k: dict(fresh=True, value=v) for k,v in values.items()}}
        self.runtime = server.RemoteRuntime.__new__(server.RemoteRuntime)
        self.runtime.state = Mock(snapshot=lambda: self.data)

    def test_start_and_every_stale_source(self):
        self.runtime._incline_gate()
        for key in self.data['status']:
            self.data['status'][key]['fresh'] = False
            with self.assertRaises(ValueError): self.runtime._incline_gate()
            self.data['status'][key]['fresh'] = True

    def test_uncalibrated_busy_and_kill_rejected(self):
        for key,field,value in [('wall','incline_ready',False),('runtime','enabled',False),('kill','active',True),('state','armed',False),('operator','mode',2)]:
            old = copy.deepcopy(self.data)
            self.data['status'][key]['value'][field] = value
            with self.assertRaises(ValueError): self.runtime._incline_gate()
            self.data = old

    def test_detach_requires_actual_incline_hold(self):
        self.data['status']['state']['value']['mode'] = 'OFFBOARD'
        self.data['status']['wall']['value'].update(incline_active=True, incline_phase='HOLD',detach_ready=True)
        self.runtime._incline_gate(detach=True)
        for field,value in [('incline_active',False),('incline_phase','RETURN'),('detach_ready',False)]:
            old = copy.deepcopy(self.data)
            self.data['status']['wall']['value'][field] = value
            with self.assertRaises(ValueError): self.runtime._incline_gate(detach=True)
            self.data = old
