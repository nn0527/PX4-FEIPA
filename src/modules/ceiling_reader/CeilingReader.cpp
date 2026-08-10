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
 * Subscribes to ESP32 UART frames and publishes independent upward and
 * forward-facing distance sensor instances.
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

uint8_t CeilingReader::crc8_atm(const uint8_t *data, size_t length)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < length; ++i) {
		crc ^= data[i];

		for (uint8_t bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
			      : static_cast<uint8_t>(crc << 1U);
		}
	}

	return crc;
}

bool CeilingReader::validate_frame(const esp32_uart_frame_s &frame) const
{
	return frame.frame[0] == FRAME_HEADER_0
	       && frame.frame[1] == FRAME_HEADER_1
	       && frame.frame[7] == FRAME_TAIL_0
	       && frame.frame[8] == FRAME_TAIL_1
	       && crc8_atm(frame.frame, CRC_DATA_LENGTH) == frame.frame[CRC_INDEX];
}

bool CeilingReader::publish_distance(uint16_t distance_mm, uint8_t orientation, hrt_abstime timestamp,
				     uORB::PublicationMulti<distance_sensor_s> &publication)
{
	distance_sensor_s distance_msg{};
	distance_msg.timestamp = timestamp;
	distance_msg.current_distance = distance_mm * 0.001f;
	distance_msg.min_distance = 0.001f;
	distance_msg.max_distance = 65.534f;
	distance_msg.signal_quality = 100;
	distance_msg.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	distance_msg.orientation = orientation;
	return publication.publish(distance_msg);
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

		const uint16_t up_distance_mm = (static_cast<uint16_t>(frame.frame[2]) << 8) | frame.frame[3];
		const uint16_t front_distance_mm = (static_cast<uint16_t>(frame.frame[4]) << 8) | frame.frame[5];
		const hrt_abstime sample_timestamp = frame.timestamp != 0 ? frame.timestamp : hrt_absolute_time();
		_received_frames.fetch_add(1);

		if (up_distance_mm != 0 && up_distance_mm != 0xFFFFU
		    && publish_distance(up_distance_mm, distance_sensor_s::ROTATION_UPWARD_FACING,
					sample_timestamp, _up_distance_sensor_pub)) {
			_last_up_distance_mm.store(up_distance_mm);
			_last_up_update_timestamp.store(sample_timestamp);
			_up_publications.fetch_add(1);
		}

		if (front_distance_mm != 0 && front_distance_mm != 0xFFFFU
		    && publish_distance(front_distance_mm, distance_sensor_s::ROTATION_FORWARD_FACING,
					sample_timestamp, _front_distance_sensor_pub)) {
			_last_front_distance_mm.store(front_distance_mm);
			_last_front_update_timestamp.store(sample_timestamp);
			_front_publications.fetch_add(1);
		}

		const hrt_abstime now = hrt_absolute_time();

		if (now - _last_log_timestamp >= LOG_INTERVAL_US) {
			PX4_INFO("ranges: UP=%u mm, FRONT=%u mm", static_cast<unsigned>(up_distance_mm),
				 static_cast<unsigned>(front_distance_mm));
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
	PX4_INFO("publications: UP=%" PRIu64 ", FRONT=%" PRIu64,
		 _up_publications.load(), _front_publications.load());

	const hrt_abstime last_up_update = _last_up_update_timestamp.load();
	const hrt_abstime last_front_update = _last_front_update_timestamp.load();

	if (last_up_update > 0) {
		PX4_INFO("last UP: %" PRIu32 " mm, %" PRIu64 " ms ago", _last_up_distance_mm.load(),
			 (hrt_absolute_time() - last_up_update) / 1000);

	} else {
		PX4_INFO("last UP: none");
	}

	if (last_front_update > 0) {
		PX4_INFO("last FRONT: %" PRIu32 " mm, %" PRIu64 " ms ago", _last_front_distance_mm.load(),
			 (hrt_absolute_time() - last_front_update) / 1000);

	} else {
		PX4_INFO("last FRONT: none");
	}

	return PX4_OK;
}

int CeilingReader::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		"Subscribes to `esp32_uart_frame`, validates the 9-byte dual-range frame and "
		"publishes UP (orientation 24) and FRONT (orientation 0) as independent "
		"`distance_sensor` instances. Zero and 0xffff invalidate only that direction.");

	PRINT_MODULE_USAGE_NAME("CeilingReader", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the ceiling distance publisher");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

extern "C" __EXPORT int CeilingReader_main(int argc, char *argv[])
{
	return CeilingReader::main(argc, argv);
}
