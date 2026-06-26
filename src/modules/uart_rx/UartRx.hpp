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
 * @file UartRx.hpp
 *
 * ESP32 fixed-frame UART receiver for TELEM2 on PX4 FMUv6X boards.
 */

#pragma once

#include <drivers/drv_hrt.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/esp32_uart_frame.h>

#include <termios.h>

#include <cstddef>
#include <cstdint>

class UartRx : public ModuleBase<UartRx>
{
public:
	UartRx(const char *device_path, unsigned baudrate);
	~UartRx() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UartRx *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	void run() override;

	/** @see ModuleBase */
	int print_status() override;

private:
	static constexpr size_t DEVICE_PATH_LENGTH = 32;
	static constexpr size_t FRAME_LENGTH = 7;
	static constexpr uint8_t FRAME_HEADER_0 = 0xAA;
	static constexpr uint8_t FRAME_HEADER_1 = 0x55;
	static constexpr uint8_t FRAME_TAIL_0 = 0x0D;
	static constexpr uint8_t FRAME_TAIL_1 = 0x0A;

	static bool baud_to_speed(unsigned baudrate, speed_t &speed);

	bool configure_uart();
	void close_uart();
	void process_byte(uint8_t byte);
	void publish_frame();

	int _fd{-1};
	struct termios _original_uart_config {};
	bool _uart_configured{false};
	char _device_path[DEVICE_PATH_LENGTH] {};
	unsigned _baudrate{0};

	uint8_t _frame[FRAME_LENGTH] {};
	uint8_t _frame_index{0};

	px4::atomic<uint64_t> _rx_bytes{0};
	px4::atomic<uint64_t> _valid_frames{0};
	px4::atomic<uint64_t> _invalid_frames{0};
	px4::atomic<uint32_t> _read_errors{0};
	px4::atomic<hrt_abstime> _last_valid_frame_timestamp{0};

	uORB::Publication<esp32_uart_frame_s> _frame_pub{ORB_ID(esp32_uart_frame)};
};
