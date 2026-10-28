"""Targeted PX4 failsafe reads/writes; fresh wire replies, never cached values."""
import math
import struct
import time
from px4_wall_ceiling_control.adapters.fusion_mavlink import MavrosParameterTransport

PARAMETERS = {'COM_OBL_RC_ACT': (6, 4), 'COM_OF_LOSS_T': (9, 2.0), 'COM_OBL_ACT': (6, 0)}


def decode_failsafe_parameter(message, system, component):
    if (message.msgid != 22 or message.sysid != system or message.compid != component
            or message.framing_status != 1):
        return None
    raw = struct.pack('<%dQ' % len(message.payload64), *message.payload64)[:message.len]
    if not 8 <= len(raw) <= 25:
        return None
    raw = raw.ljust(25, b'\0')
    name = raw[8:24].split(b'\0', 1)[0].decode('ascii', errors='replace')
    if name not in PARAMETERS or raw[24] != PARAMETERS[name][0]:
        return None
    value = struct.unpack('<i' if raw[24] == 6 else '<f', raw[:4])[0]
    return (name, value) if math.isfinite(value) else None


class CeilingFailsafeTransport(MavrosParameterTransport):
    def _on_message(self, message):
        received = time.monotonic()
        value = decode_failsafe_parameter(message, self.target_system, self.target_component)
        if value is None or message.header.stamp.to_sec() <= 0:
            return
        with self.condition:
            self.responses.append((received, message.header.stamp.to_sec(), *value))
            self.responses = self.responses[-32:]
            self.condition.notify_all()

    def _publish(self, parameter, expected):
        if parameter not in PARAMETERS:
            raise ValueError('unexpected failsafe parameter')
        kind, required = PARAMETERS[parameter]
        if expected is None or kind == 6:
            return super()._publish(parameter, expected)
        if abs(expected - required) > 1e-6:
            raise ValueError('unexpected failsafe timeout')
        packet = self.encoder.param_set_encode(self.target_system, self.target_component,
            parameter.encode('ascii'), expected, kind)
        packet.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) & 255
        raw = packet.get_payload()
        header = packet.get_header()
        message = self.message_type()
        message.header.stamp = self.ros.Time.now()
        message.framing_status, message.magic = 1, 253
        message.len, message.seq = header.mlen, header.seq
        message.sysid, message.compid, message.msgid = header.srcSystem, header.srcComponent, header.msgId
        message.checksum = packet.get_crc()
        raw += b'\0' * ((-len(raw)) % 8)
        message.payload64 = list(struct.unpack('<%dQ' % (len(raw)//8), raw))
        self.publisher.publish(message)

    def _exchange(self, parameter, expected=None):
        # Compare float32 readback exactly in its wire representation.
        if expected is not None and PARAMETERS[parameter][0] == 9:
            expected = struct.unpack('<f', struct.pack('<f', expected))[0]
        return super()._exchange(parameter, expected)
