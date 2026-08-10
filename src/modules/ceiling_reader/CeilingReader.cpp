/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CeilingReader.cpp
 *
 * Subscribes to the ESP32 UART frame topic and extracts a ceiling distance in
 * millimetres. This example does not publish control setpoints or command the
 * vehicle.
 */

#include "CeilingReader.hpp"

#include <px4_platform_common/log.h>

#include <inttypes.h>

CeilingReader::CeilingReader() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool CeilingReader::init()
{
	if (!_esp32_frame_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

bool CeilingReader::validate_frame(const esp32_uart_frame_s &frame) const
{
	return frame.frame[0] == FRAME_HEADER_0
	       && frame.frame[1] == FRAME_HEADER_1
	       && (frame.frame[4] == SENSOR_ID_Z || frame.frame[4] == SENSOR_ID_FRONT)
	       && frame.frame[5] == FRAME_TAIL_0
	       && frame.frame[6] == FRAME_TAIL_1;
}

uint16_t CeilingReader::parse_distance_mm(const esp32_uart_frame_s &frame) const
{
	return (static_cast<uint16_t>(frame.frame[2]) << 8) | frame.frame[3];
}

void CeilingReader::Run()
{
	if (should_exit()) {
		_esp32_frame_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	esp32_uart_frame_s frame{};

	while (_esp32_frame_sub.update(&frame)) {
		if (!validate_frame(frame)) {
			_invalid_frames.fetch_add(1);
			continue;
		}

		const uint16_t distance_mm = parse_distance_mm(frame);
		const hrt_abstime now = hrt_absolute_time();
		_received_frames.fetch_add(1);

		// frame[4] was reserved in the original protocol. It is now the sensor ID:
		// 0 = suction/contact sensor mounted along body -Z, 1 = body +X/front sensor.
		// ID 0 remains backward compatible with the original single-sensor sender.
		const uint8_t sensor_id = frame.frame[4];
		distance_sensor_s distance_msg{};
		distance_msg.timestamp        = now;
		distance_msg.current_distance = distance_mm * 0.001f; // 转换为米
		distance_msg.min_distance     = 0.005f; // 5mm
		distance_msg.max_distance     = 50.0f;  // 50m
		distance_msg.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		distance_msg.signal_quality   = 100;

		if (sensor_id == SENSOR_ID_FRONT) {
			distance_msg.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
			_front_distance_sensor_pub.publish(distance_msg);
			_last_front_distance_mm.store(distance_mm);
			_last_front_update_timestamp.store(now);
			_front_frames.fetch_add(1);

		} else {
			// PX4 uses the FRD body frame. The suction face points upward while the
			// vehicle is level, therefore its direction is body -Z (UPWARD_FACING).
			distance_msg.orientation = distance_sensor_s::ROTATION_UPWARD_FACING;
			_z_distance_sensor_pub.publish(distance_msg);
			_last_z_distance_mm.store(distance_mm);
			_last_z_update_timestamp.store(now);
			_z_frames.fetch_add(1);
		}

		if (now - _last_log_timestamp >= LOG_INTERVAL_US) {
			PX4_INFO("sensor=%s distance=%u mm", sensor_id == SENSOR_ID_FRONT ? "front" : "z",
				 static_cast<unsigned>(distance_mm));
			_last_log_timestamp = now;
		}
	}
}

int CeilingReader::task_spawn(int argc, char *argv[])
{
	CeilingReader *instance = new CeilingReader();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CeilingReader::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CeilingReader::print_status()
{
	PX4_INFO("running");
	PX4_INFO("received frames: %" PRIu64, _received_frames.load());
	PX4_INFO("invalid frames: %" PRIu64, _invalid_frames.load());
	PX4_INFO("z/front frames: %" PRIu64 "/%" PRIu64, _z_frames.load(), _front_frames.load());

	const hrt_abstime last_z_update = _last_z_update_timestamp.load();
	const hrt_abstime last_front_update = _last_front_update_timestamp.load();

	if (last_z_update > 0) {
		PX4_INFO("z distance: %" PRIu32 " mm (%" PRIu64 " ms ago)", _last_z_distance_mm.load(),
			 (hrt_absolute_time() - last_z_update) / 1000);

	} else {
		PX4_INFO("z distance: none");
	}

	if (last_front_update > 0) {
		PX4_INFO("front distance: %" PRIu32 " mm (%" PRIu64 " ms ago)", _last_front_distance_mm.load(),
			 (hrt_absolute_time() - last_front_update) / 1000);

	} else {
		PX4_INFO("front distance: none");
	}

	return PX4_OK;
}

int CeilingReader::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		"Subscribes to `esp32_uart_frame`, validates `AA 55 distance_high "
		"distance_low sensor_id 0D 0A`, and publishes standard multi-instance "
		"`distance_sensor` data. sensor_id 0 is the body -Z suction/contact "
		"sensor and sensor_id 1 is the body +X/front sensor.");

	PRINT_MODULE_USAGE_NAME("CeilingReader", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the ceiling distance publisher");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

extern "C" __EXPORT int CeilingReader_main(int argc, char *argv[])
{
	return CeilingReader::main(argc, argv);
}
