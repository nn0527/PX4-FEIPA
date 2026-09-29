#!/usr/bin/env python3
"""Serial adapter for one L1 range sensor or the legacy dual-range prototype."""

import math
import re

import rospy
from sensor_msgs.msg import Range


class LidarDistanceNode:
    def __init__(self):
        self.serial_port = str(rospy.get_param("~serial_port", ""))
        self.baudrate = int(rospy.get_param("~baudrate", 38400))
        self.serial_timeout_s = float(rospy.get_param("~serial_timeout_s", 0.2))
        self.input_format = str(rospy.get_param("~input_format", "l1_ascii"))
        self.startup_command = str(rospy.get_param("~startup_command", "iACM"))
        self.halt_on_shutdown = bool(rospy.get_param("~halt_on_shutdown", False))
        self.input_scale_to_m = float(rospy.get_param("~input_scale_to_m", 1.0))
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.03))
        self.max_range_m = float(rospy.get_param("~max_range_m", 40.0))
        self.field_of_view_rad = float(
            rospy.get_param("~field_of_view_rad", 0.035)
        )
        self.ceiling_frame_id = rospy.get_param(
            "~ceiling_frame_id", "ceiling_lidar_link"
        )
        self.wall_frame_id = rospy.get_param("~wall_frame_id", "wall_lidar_link")

        # range_topic/frame_id are the preferred interface for one physical L1.
        # The old ceiling/wall names remain available for legacy_pair recordings.
        self.range_topic = rospy.get_param(
            "~range_topic",
            rospy.get_param("~ceiling_range_topic", "/sensor/ceiling/raw"),
        )
        self.frame_id = rospy.get_param(
            "~frame_id", rospy.get_param("~ceiling_frame_id", "ceiling_lidar_link")
        )
        self.range_publisher = rospy.Publisher(
            self.range_topic, Range, queue_size=10
        )
        if self.input_format == "legacy_pair":
            ceiling_topic = rospy.get_param(
                "~ceiling_range_topic", "/sensor/ceiling/raw"
            )
            wall_topic = rospy.get_param("~wall_range_topic", "/sensor/front/raw")
            self.ceiling_publisher = rospy.Publisher(
                ceiling_topic, Range, queue_size=10
            )
            self.wall_publisher = rospy.Publisher(wall_topic, Range, queue_size=10)

    @staticmethod
    def decode_legacy_pair(raw_line):
        """Decode the temporary `ceiling=1.2,wall=0.8` or `1.2,0.8` format."""
        text = raw_line.decode("ascii", errors="strict").strip()
        if not text:
            raise ValueError("empty frame")

        parts = [part.strip() for part in text.split(",")]
        if len(parts) != 2:
            raise ValueError("expected two comma-separated distances")

        if "=" in parts[0] or "=" in parts[1]:
            values = {}
            for part in parts:
                key, value = part.split("=", 1)
                values[key.strip().lower()] = float(value.strip())
            if "ceiling" not in values or "wall" not in values:
                raise ValueError("named frame requires ceiling and wall fields")
            return values["ceiling"], values["wall"]

        return float(parts[0]), float(parts[1])

    @staticmethod
    def decode_l1_ascii(raw_line):
        """Return distance in metres from `D=1.314m,520#`; raise on L1 errors."""
        text = raw_line.decode("ascii", errors="strict").strip()
        error_match = re.fullmatch(r"E=(\d+)", text)
        if error_match:
            error_code = int(error_match.group(1))
            descriptions = {
                252: "temperature too high",
                253: "temperature too low",
                255: "weak reflection or calculation failure",
                256: "strong reflection",
                258: "outside configured range",
                285: "photosensor fault",
                286: "laser fault",
                290: "hardware fault",
            }
            description = descriptions.get(error_code, "unknown sensor error")
            raise ValueError("L1 error {} ({})".format(error_code, description))

        measurement = re.fullmatch(
            r"D=([+-]?(?:\d+(?:\.\d*)?|\.\d+))m(?:,\d+)?#?", text
        )
        if not measurement:
            raise ValueError("unrecognized L1 response: {!r}".format(text))
        distance_m = float(measurement.group(1))
        if not math.isfinite(distance_m):
            raise ValueError("L1 distance is not finite")
        return distance_m

    def _make_range(self, distance_m, frame_id, stamp):
        message = Range()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.radiation_type = Range.INFRARED
        message.field_of_view = self.field_of_view_rad
        message.min_range = self.min_range_m
        message.max_range = self.max_range_m
        message.range = distance_m
        return message

    def _publish_pair(self, ceiling_raw, wall_raw):
        ceiling_m = ceiling_raw * self.input_scale_to_m
        wall_m = wall_raw * self.input_scale_to_m
        if not math.isfinite(ceiling_m) or not math.isfinite(wall_m):
            raise ValueError("distance is not finite")
        stamp = rospy.Time.now()
        self.ceiling_publisher.publish(
            self._make_range(ceiling_m, self.ceiling_frame_id, stamp)
        )
        self.wall_publisher.publish(
            self._make_range(wall_m, self.wall_frame_id, stamp)
        )

    def _publish_single(self, distance_m):
        distance_m *= self.input_scale_to_m
        if not math.isfinite(distance_m):
            raise ValueError("distance is not finite")
        self.range_publisher.publish(
            self._make_range(distance_m, self.frame_id, rospy.Time.now())
        )

    def _handle_line(self, raw_line):
        if self.input_format == "l1_ascii":
            self._publish_single(self.decode_l1_ascii(raw_line))
        elif self.input_format == "legacy_pair":
            ceiling, wall = self.decode_legacy_pair(raw_line)
            self._publish_pair(ceiling, wall)
        else:
            raise ValueError("unsupported input_format: {}".format(self.input_format))

    def run(self):
        if not self.serial_port:
            rospy.logwarn(
                "serial_port is empty; lidar UART adapter is in placeholder mode"
            )
            rospy.spin()
            return

        try:
            import serial  # Imported lazily so the safe placeholder can still launch.
        except ImportError:
            rospy.logerr("pyserial is missing; install the python3-serial package")
            rospy.spin()
            return

        while not rospy.is_shutdown():
            try:
                rospy.loginfo(
                    "opening lidar UART %s at %d", self.serial_port, self.baudrate
                )
                with serial.Serial(
                    self.serial_port,
                    self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.serial_timeout_s,
                ) as device:
                    device.reset_input_buffer()
                    if self.input_format == "l1_ascii" and self.startup_command:
                        device.write(self.startup_command.encode("ascii"))
                        device.flush()
                        rospy.loginfo("sent L1 startup command %s", self.startup_command)
                    try:
                        while not rospy.is_shutdown():
                            raw_line = device.readline()
                            if not raw_line:
                                continue
                            try:
                                self._handle_line(raw_line)
                            except (UnicodeError, ValueError) as exc:
                                rospy.logwarn_throttle(2.0, "invalid lidar frame: %s", exc)
                    finally:
                        if self.input_format == "l1_ascii" and self.halt_on_shutdown:
                            device.write(b"iHALT")
                            device.flush()
            except (OSError, serial.SerialException) as exc:
                rospy.logerr_throttle(5.0, "lidar UART error: %s", exc)
                rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("lidar_distance_node")
    LidarDistanceNode().run()
