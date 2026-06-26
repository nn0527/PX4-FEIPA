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
 * @file CeilingReader.hpp
 *
 * Example module that consumes ESP32 UART distance frames from uORB.
 */

#pragma once

#include <drivers/drv_hrt.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/esp32_uart_frame.h>
#include <uORB/topics/distance_sensor.h>

#include <cstdint>

class CeilingReader : public ModuleBase<CeilingReader>, public px4::ScheduledWorkItem
{
public:
	CeilingReader();
	~CeilingReader() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	static constexpr uint8_t FRAME_HEADER_0 = 0xAA;
	static constexpr uint8_t FRAME_HEADER_1 = 0x55;
	static constexpr uint8_t FRAME_TAIL_0 = 0x0D;
	static constexpr uint8_t FRAME_TAIL_1 = 0x0A;
	static constexpr hrt_abstime LOG_INTERVAL_US = 500000; // 2 Hz dmesg output limit

	void Run() override;
	bool validate_frame(const esp32_uart_frame_s &frame) const;
	uint16_t parse_distance_mm(const esp32_uart_frame_s &frame) const;

	uORB::SubscriptionCallbackWorkItem _esp32_frame_sub{this, ORB_ID(esp32_uart_frame)};
	uORB::PublicationMulti<distance_sensor_s> _distance_sensor_pub{ORB_ID(distance_sensor)};

	px4::atomic<uint64_t> _received_frames{0};
	px4::atomic<uint64_t> _invalid_frames{0};
	px4::atomic<uint32_t> _last_distance_mm{0};
	px4::atomic<hrt_abstime> _last_update_timestamp{0};
	hrt_abstime _last_log_timestamp{0};
};
