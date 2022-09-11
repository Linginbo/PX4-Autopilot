/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include "RCInput.hpp"

#include <uORB/topics/vehicle_command_ack.h>

#include <termios.h>

using namespace time_literals;

RcDsm::RcDsm(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	// initialize raw_rc values and count
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_raw_rc_values[i] = UINT16_MAX;
	}

	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

RcDsm::~RcDsm()
{
#if defined(SPEKTRUM_POWER_PASSIVE)
	// Disable power controls for Spektrum receiver
	SPEKTRUM_POWER_PASSIVE();
#endif
	dsm_deinit();

	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int RcDsm::init()
{
	// dsm_init sets some file static variables and returns a file descriptor
	// it also powers on the radio if needed
	_rcs_fd = dsm_init(_device);

	if (_rcs_fd < 0) {
		return -errno;
	}

	if (board_rc_swap_rxtx(_device)) {
#if defined(TIOCSSWAP)
		ioctl(_rcs_fd, TIOCSSWAP, SER_SWAP_ENABLED);
#endif // TIOCSSWAP
	}

	rc_io_invert(false);

	return 0;
}

int
RcDsm::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;
#if defined(RC_SERIAL_PORT)
	device_name = RC_SERIAL_PORT;
#endif // RC_SERIAL_PORT

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		RcDsm *instance = new RcDsm(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(_current_update_interval);

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

void
RcDsm::fill_rc_in(uint16_t raw_rc_count_local,
		  uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		  hrt_abstime now, bool frame_drop, bool failsafe,
		  unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

void RcDsm::rc_io_invert(bool invert)
{
	// First check if the board provides a board-specific inversion method (e.g. via GPIO),
	// and if not use an IOCTL
	if (!board_rc_invert_input(_device, invert)) {
#if defined(TIOCSINVERT)

		if (invert) {
			ioctl(_rcs_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);

		} else {
			ioctl(_rcs_fd, TIOCSINVERT, 0);
		}

#endif // TIOCSINVERT
	}
}

void RcDsm::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {

		perf_begin(_cycle_perf);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			}
		}

		const hrt_abstime cycle_timestamp = hrt_absolute_time();


		/* vehicle command */
		vehicle_command_s vcmd;

		if (_vehicle_cmd_sub.update(&vcmd)) {
			// Check for a pairing command
			if (vcmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) {

				uint8_t cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#if defined(SPEKTRUM_POWER)

				if (!_rc_scan_locked && !_armed) {
					if ((int)vcmd.param1 == 0) {
						// DSM binding command
						int dsm_bind_mode = (int)vcmd.param2;

						int dsm_bind_pulses = 0;

						if (dsm_bind_mode == 0) {
							dsm_bind_pulses = DSM2_BIND_PULSES;

						} else if (dsm_bind_mode == 1) {
							dsm_bind_pulses = DSMX_BIND_PULSES;

						} else {
							dsm_bind_pulses = DSMX8_BIND_PULSES;
						}

						bind_spektrum(dsm_bind_pulses);

						cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					}

				} else {
					cmd_ret = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

#endif // SPEKTRUM_POWER

				// publish acknowledgement
				vehicle_command_ack_s command_ack{};
				command_ack.command = vcmd.command;
				command_ack.result = cmd_ret;
				command_ack.target_system = vcmd.source_system;
				command_ack.target_component = vcmd.source_component;
				command_ack.timestamp = hrt_absolute_time();
				uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
				vehicle_command_ack_pub.publish(command_ack);
			}
		}

		bool rc_updated = false;

		// This block scans for a supported serial RC input and locks onto the first one found
		// Scan for 500 msec, then switch protocol
		constexpr hrt_abstime rc_scan_max = 500_ms;

		unsigned frame_drops = 0;

		// TODO: needs work (poll _rcs_fd)
		// int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		// then update priority to SCHED_PRIORITY_FAST_DRIVER
		// read all available data from the serial RC input UART

		// read all available data from the serial RC input UART
		int newBytes = ::read(_rcs_fd, &_rcs_buf[0], RC_MAX_BUFFER_SIZE);

		if (newBytes > 0) {
			_bytes_rx += newBytes;
		}

		const bool rc_scan_locked = _rc_scan_locked;


		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;
			// Configure serial port for DSM
			dsm_config(_rcs_fd);

			// flush serial buffer and any existing buffered data
			tcflush(_rcs_fd, TCIOFLUSH);
			memset(_rcs_buf, 0, sizeof(_rcs_buf));

		} else if (_rc_scan_locked
			   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			if (newBytes > 0) {
				int8_t dsm_rssi = 0;
				bool dsm_11_bit = false;

				// parse new data
				rc_updated = dsm_parse(cycle_timestamp, &_rcs_buf[0], newBytes, &_raw_rc_values[0], &_raw_rc_count,
						       &dsm_11_bit, &frame_drops, &dsm_rssi, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new DSM frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;
					fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
						   false, false, frame_drops, dsm_rssi);
					_rc_scan_locked = true;
				}
			}

		}

		perf_end(_cycle_perf);

		if (rc_updated) {
			perf_count(_publish_interval_perf);

			_to_input_rc.publish(_rc_in);

		} else if (!rc_updated && !_armed && (hrt_elapsed_time(&_rc_in.timestamp_last_signal) > 1_s)) {
			_rc_scan_locked = false;
		}

		if (!rc_scan_locked && _rc_scan_locked) {
			PX4_INFO("RC input locked");
		}

	}
}

#if defined(SPEKTRUM_POWER)
bool RcDsm::bind_spektrum(int arg) const
{
	int ret = PX4_ERROR;

	/* specify 11ms DSMX. RX will automatically fall back to 22ms or DSM2 if necessary */

	/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
	PX4_INFO("DSM_BIND_START: DSM%s RX", (arg == 0) ? "2" : ((arg == 1) ? "-X" : "-X8"));

	if (arg == DSM2_BIND_PULSES ||
	    arg == DSMX_BIND_PULSES ||
	    arg == DSMX8_BIND_PULSES) {

		dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);

		dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
		usleep(500000);

		dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
		usleep(72000);

		irqstate_t flags = px4_enter_critical_section();
		dsm_bind(DSM_CMD_BIND_SEND_PULSES, arg);
		px4_leave_critical_section(flags);

		usleep(50000);

		dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

		ret = OK;

	} else {
		PX4_ERR("DSM bind failed");
		ret = -EINVAL;
	}

	return (ret == PX4_OK);
}
#endif /* SPEKTRUM_POWER */

int RcDsm::custom_command(int argc, char *argv[])
{
#if defined(SPEKTRUM_POWER)
	const char *verb = argv[0];

	if (!strcmp(verb, "bind")) {
		uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
		vehicle_command_s vcmd{};
		vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
		vcmd.timestamp = hrt_absolute_time();
		vehicle_command_pub.publish(vcmd);
		return 0;
	}

#endif /* SPEKTRUM_POWER */

	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RcDsm::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RcDsm::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	PX4_INFO("RC state: %s", _rc_scan_locked ? "found" : "searching for signal");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	if (hrt_elapsed_time(&_rc_in.timestamp) < 1_s) {
		print_message(ORB_ID(input_rc), _rc_in);
	}

	return 0;
}

int RcDsm::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:
- DSM

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_dsm", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

#if defined(SPEKTRUM_POWER)
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "Send a DSM bind command (module must be running)");
#endif /* SPEKTRUM_POWER */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rc_dsm_main(int argc, char *argv[])
{
	return RcDsm::main(argc, argv);
}
