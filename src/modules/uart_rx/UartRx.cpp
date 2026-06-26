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
 * @file UartRx.cpp
 *
 * The UART setup and non-blocking read flow follow PX4's Lightware serial
 * driver. The baud-rate mapping follows PX4's serial_test command.
 *
 * Source references (PX4-Autopilot v1.16.0, commit 6ea3539):
 * - src/drivers/distance_sensor/lightware_laser_serial/lightware_laser_serial.cpp
 * - src/systemcmds/serial_test/serial_test.c
 */

#include "UartRx.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <cerrno>
#include <cstring>
#include <inttypes.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace
{

constexpr char DEFAULT_DEVICE[] = "/dev/ttyS4";
constexpr unsigned DEFAULT_BAUDRATE = 115200;
constexpr size_t RX_BUFFER_LENGTH = 64;
constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2200);

} // namespace

UartRx::UartRx(const char *device_path, unsigned baudrate) :
	_baudrate(baudrate)
{
	strncpy(_device_path, device_path, sizeof(_device_path) - 1);
	_device_path[sizeof(_device_path) - 1] = '\0';
}

UartRx::~UartRx()
{
	close_uart();
}

bool UartRx::baud_to_speed(unsigned baudrate, speed_t &speed)
{
	switch (baudrate) {
	case 9600:
		speed = B9600;
		return true;

	case 19200:
		speed = B19200;
		return true;

	case 38400:
		speed = B38400;
		return true;

	case 57600:
		speed = B57600;
		return true;

	case 115200:
		speed = B115200;
		return true;

	case 230400:
		speed = B230400;
		return true;

	case 460800:
		speed = B460800;
		return true;

	case 921600:
		speed = B921600;
		return true;

	default:
		return false;
	}
}

bool UartRx::configure_uart()
{
	speed_t speed;

	if (!baud_to_speed(_baudrate, speed)) {
		PX4_ERR("unsupported baud rate: %u", _baudrate);
		return false;
	}

	if (tcgetattr(_fd, &_original_uart_config) < 0) {
		PX4_ERR("tcgetattr failed (%i)", errno);
		return false;
	}

	struct termios uart_config = _original_uart_config;

	// Raw 8N1, no hardware or software flow control.
	uart_config.c_iflag = 0;
	uart_config.c_oflag = 0;
	uart_config.c_lflag = 0;
	uart_config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
	uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

#ifdef CRTSCTS
	uart_config.c_cflag &= ~CRTSCTS;
#endif

	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baud rate %u (%i)", _baudrate, errno);
		return false;
	}

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed (%i)", errno);
		return false;
	}

	_uart_configured = true;
	return true;
}

void UartRx::close_uart()
{
	if (_fd < 0) {
		return;
	}

	if (_uart_configured && tcsetattr(_fd, TCSANOW, &_original_uart_config) < 0) {
		PX4_WARN("failed to restore UART settings (%i)", errno);
	}

	::close(_fd);
	_fd = -1;
	_uart_configured = false;
}

void UartRx::publish_frame()
{
	esp32_uart_frame_s message{};
	message.timestamp = hrt_absolute_time();
	memcpy(message.frame, _frame, sizeof(message.frame));
	_frame_pub.publish(message);

	const uint16_t distance_mm = (static_cast<uint16_t>(message.frame[2]) << 8) | message.frame[3];
	PX4_INFO("distance: %u mm", static_cast<unsigned>(distance_mm));

	_valid_frames.fetch_add(1);
	_last_valid_frame_timestamp.store(message.timestamp);
}

void UartRx::process_byte(uint8_t byte)
{
	if (_frame_index == 0) {
		if (byte == FRAME_HEADER_0) {
			_frame[0] = byte;
			_frame_index = 1;
		}

		return;
	}

	if (_frame_index == 1) {
		if (byte == FRAME_HEADER_1) {
			_frame[1] = byte;
			_frame_index = 2;

		} else {
			// The second header byte is invalid. A repeated 0xAA is retained as
			// the potential start of the next frame to resynchronize immediately.
			_invalid_frames.fetch_add(1);
			_frame[0] = byte;
			_frame_index = (byte == FRAME_HEADER_0) ? 1 : 0;
		}

		return;
	}

	_frame[_frame_index++] = byte;

	if (_frame_index != FRAME_LENGTH) {
		return;
	}

	if (_frame[FRAME_LENGTH - 2] == FRAME_TAIL_0 && _frame[FRAME_LENGTH - 1] == FRAME_TAIL_1) {
		publish_frame();

	} else {
		_invalid_frames.fetch_add(1);
	}

	if (byte == FRAME_HEADER_0) {
		_frame[0] = byte;
		_frame_index = 1;

	} else {
		_frame_index = 0;
	}
}

void UartRx::run()
{
	_fd = ::open(_device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed (%i)", _device_path, errno);
		return;
	}

	if (!configure_uart()) {
		close_uart();
		return;
	}

	PX4_INFO("reading ESP32 frames from %s at %u baud (8N1)", _device_path, _baudrate);

	while (!should_exit()) {
		px4_pollfd_struct_t fds[1] {};
		fds[0].fd = _fd;
		fds[0].events = POLLIN;

		const int poll_ret = px4_poll(fds, 1, 250);

		if (poll_ret < 0) {
			if (errno != EINTR) {
				_read_errors.fetch_add(1);
				PX4_ERR("poll failed (%i)", errno);
			}

			continue;
		}

		if (poll_ret == 0) {
			continue;
		}

		if (fds[0].revents & POLLIN) {
			uint8_t buffer[RX_BUFFER_LENGTH];
			const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

			if (bytes_read > 0) {
				_rx_bytes.fetch_add(bytes_read);

				for (ssize_t i = 0; i < bytes_read; i++) {
					process_byte(buffer[i]);
				}

			} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
				_read_errors.fetch_add(1);
				PX4_ERR("read failed (%i)", errno);
			}
		}

		if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
			_read_errors.fetch_add(1);
			PX4_ERR("UART poll error: 0x%lx", (unsigned long)fds[0].revents);
		}
	}

	close_uart();
	PX4_INFO("stopped");
}

int UartRx::task_spawn(int argc, char *argv[])
{
	const int task_id = px4_task_spawn_cmd("uart_rx", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
						  TASK_STACK_SIZE, run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		return -errno;
	}

	_task_id = task_id;
	return PX4_OK;
}

UartRx *UartRx::instantiate(int argc, char *argv[])
{
	char device_path[DEVICE_PATH_LENGTH] {};
	strncpy(device_path, DEFAULT_DEVICE, sizeof(device_path) - 1);
	int baudrate = DEFAULT_BAUDRATE;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0 || baudrate <= 0) {
				PX4_ERR("invalid baud rate");
				return nullptr;
			}

			break;

		case 'd':
			if (strlen(myoptarg) >= sizeof(device_path)) {
				PX4_ERR("serial device path is too long");
				return nullptr;
			}

			strncpy(device_path, myoptarg, sizeof(device_path) - 1);
			device_path[sizeof(device_path) - 1] = '\0';
			break;

		default:
			return nullptr;
		}
	}

	speed_t speed;

	if (!baud_to_speed(static_cast<unsigned>(baudrate), speed)) {
		PX4_ERR("unsupported baud rate: %d", baudrate);
		return nullptr;
	}

	return new UartRx(device_path, static_cast<unsigned>(baudrate));
}

int UartRx::print_status()
{
	PX4_INFO("running: %s at %u baud (8N1)", _device_path, _baudrate);
	PX4_INFO("received: %" PRIu64 " bytes, read errors: %" PRIu32,
		 _rx_bytes.load(), _read_errors.load());
	PX4_INFO("frames: %" PRIu64 " valid, %" PRIu64 " invalid",
		 _valid_frames.load(), _invalid_frames.load());

	const hrt_abstime last_frame = _last_valid_frame_timestamp.load();

	if (last_frame > 0) {
		PX4_INFO("last valid frame: %" PRIu64 " ms ago", (hrt_absolute_time() - last_frame) / 1000);

	} else {
		PX4_INFO("last valid frame: none");
	}

	return PX4_OK;
}

int UartRx::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		"Receives fixed 7-byte ESP32 UART distance frames and publishes valid frames on the "
		"`esp32_uart_frame` uORB topic. The default is TELEM2 (`/dev/ttyS4`) "
		"at 115200 baud on PX4 FMUv6X boards.\n"
		"\n"
		"Expected frame: `AA 55 distance_high distance_low reserved 0D 0A`. "
		"Distance is printed in millimetres for each valid frame.\n"
		"The module only reads from the UART and requires exclusive ownership of the port.");

	PRINT_MODULE_USAGE_NAME("uart_rx", "example");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the ESP32 UART receiver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', DEFAULT_DEVICE, "<file:dev>", "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', DEFAULT_BAUDRATE, 9600, 921600, "Baud rate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

extern "C" __EXPORT int uart_rx_main(int argc, char *argv[])
{
	return UartRx::main(argc, argv);
}
