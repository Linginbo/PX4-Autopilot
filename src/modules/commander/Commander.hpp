/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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

#pragma once

/*   Helper classes  */
#include "Arming/ArmStateMachine/ArmStateMachine.hpp"
#include "failure_detector/FailureDetector.hpp"
#include "failsafe/failsafe.h"
#include "Safety.hpp"
#include "worker_thread.hpp"
#include "HealthAndArmingChecks/HealthAndArmingChecks.hpp"
#include "HomePosition.hpp"
#include "UserModeIntention.hpp"

#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/action_request.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/iridiumsbd_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vtol_vehicle_status.h>

using math::constrain;
using systemlib::Hysteresis;

using namespace time_literals;

// TODO: generate
static constexpr bool operator ==(const actuator_armed_s &a, const actuator_armed_s &b)
{
	return (a.armed == b.armed &&
		a.prearmed == b.prearmed &&
		a.ready_to_arm == b.ready_to_arm &&
		a.lockdown == b.lockdown &&
		a.manual_lockdown == b.manual_lockdown &&
		a.force_failsafe == b.force_failsafe &&
		a.in_esc_calibration_mode == b.in_esc_calibration_mode);
}

class Commander : public ModuleBase<Commander>, public ModuleParams
{
public:
	Commander();
	~Commander();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Commander *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void enable_hil();

private:

	enum class ActuatorFailureActions {
		DISABLED    = 0,
		AUTO_LOITER = 1,
		AUTO_LAND   = 2,
		AUTO_RTL    = 3,
		TERMINATE   = 4
	};

	enum class PrearmedMode {
		DISABLED      = 0,
		SAFETY_BUTTON = 1,
		ALWAYS        = 2
	};

	enum class RcOverrideBits : int32_t {
		AUTO_MODE_BIT     = (1 << 0),
		OFFBOARD_MODE_BIT = (1 << 1)
	};

	enum VEHICLE_MODE_FLAG {
		VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1,   /* 0b00000001 Reserved for future use. | */
		VEHICLE_MODE_FLAG_TEST_ENABLED         = 2,   /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
		VEHICLE_MODE_FLAG_AUTO_ENABLED         = 4,   /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
		VEHICLE_MODE_FLAG_GUIDED_ENABLED       = 8,   /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
		VEHICLE_MODE_FLAG_STABILIZE_ENABLED    = 16,  /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
		VEHICLE_MODE_FLAG_HIL_ENABLED          = 32,  /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
		VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,  /* 0b01000000 remote control input is enabled. | */
		VEHICLE_MODE_FLAG_SAFETY_ARMED         = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
		VEHICLE_MODE_FLAG_ENUM_END             = 129  /*  | */
	};

	void answerCommand(const vehicle_command_s &cmd, const uint8_t result);

	transition_result_t arm(const arm_disarm_reason_t calling_reason, const bool run_preflight_checks = true);

	transition_result_t disarm(const arm_disarm_reason_t calling_reason, const bool forced = false);

	void battery_status_check();

	void checkAndInformReadyForTakeoff();

	void checkForMissionUpdate();

	void checkGeofenceStatus();

	void checkWindSpeedThresholds();

	void checkWorkerThread();

	void controlStatusLeds(bool changed, const uint8_t battery_warning);

	/**
	 * Checks the status of all available data links and handles switching between different system telemetry states.
	 */
	void dataLinkCheck();

	void executeActionRequest(const action_request_s &action_request);

	void handleAutoDisarm();

	/**
	 * @brief Handle incoming vehicle command relavant to Commander
	 *
	 * It ignores irrelevant vehicle commands defined inside the switch case statement
	 * in the function.
	 *
	 * @param cmd           Vehicle command to handle
	 */
	bool handleCommand(const vehicle_command_s &cmd);

	unsigned handleCommandActuatorTest(const vehicle_command_s &cmd);

	bool handleModeIntentionAndFailsafe();

	void handlePowerButtonState();

	void landDetectorUpdate();

	void manualControlCheck();

	void offboardControlCheck();

	void printRejectMode(uint8_t nav_state);

	void safetyButtonUpdate();

	bool getPrearmState() const;

	void sendParachuteCommand();

	bool shutdownIfAllowed();

	void systemPowerUpdate();

	void updateControlMode();

	void updateParameters();

	void updateTunes();

	void vtolStatusUpdate();


	/* Decouple update interval and hysteresis counters, all depends on intervals */
	static constexpr uint64_t COMMANDER_MONITORING_INTERVAL{10_ms};
	static constexpr uint64_t INAIR_RESTART_HOLDOFF_INTERVAL{500_ms};

	orb_advert_t   _mavlink_log_pub{nullptr};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _preflight_check_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": preflight check")};

	ArmStateMachine		_arm_state_machine{};
	Failsafe		_failsafe_instance{this};
	FailsafeBase		&_failsafe{_failsafe_instance};
	FailureDetector		_failure_detector{this};
	HealthAndArmingChecks	_health_and_arming_checks{this, _vehicle_status};
	HomePosition            _home_position{_vehicle_status_flags};
	Safety			_safety{};
	UserModeIntention	_user_mode_intention{this, _vehicle_status, _health_and_arming_checks};
	WorkerThread 		_worker_thread{};

	const failsafe_flags_s &_failsafe_flags{_health_and_arming_checks.failsafeFlags()};

	actuator_armed_s        _actuator_armed{};
	commander_state_s       _commander_state{};
	geofence_result_s       _geofence_result{};
	vehicle_control_mode_s  _vehicle_control_mode{};
	vehicle_land_detected_s _vehicle_land_detected{};
	vehicle_status_s        _vehicle_status{};
	vehicle_status_flags_s  _vehicle_status_flags{};
	vtol_vehicle_status_s   _vtol_vehicle_status{};

	Hysteresis _auto_disarm_landed{false};
	Hysteresis _auto_disarm_killed{false};
	Hysteresis _offboard_available{false};

	hrt_abstime _battery_failsafe_timestamp{0};
	hrt_abstime _boot_timestamp{0};

	hrt_abstime _datalink_last_heartbeat_avoidance_system{0};
	hrt_abstime _datalink_last_heartbeat_gcs{0};
	hrt_abstime _datalink_last_heartbeat_onboard_controller{0};
	hrt_abstime _datalink_last_heartbeat_open_drone_id_system{0};
	hrt_abstime _datalink_last_heartbeat_parachute_system{0};

	hrt_abstime _high_latency_datalink_heartbeat{0};
	hrt_abstime _high_latency_datalink_lost{0};

	hrt_abstime _last_disarmed_timestamp{0};
	hrt_abstime _last_health_and_arming_check{0};
	hrt_abstime _last_print_mode_reject_time{0};    ///< To remember when last notification was sent
	hrt_abstime _last_termination_message_sent{0};
	hrt_abstime _last_valid_manual_control_setpoint{0};
	hrt_abstime _last_wind_warning{0};

	hrt_abstime _led_overload_toggle{0};
	hrt_abstime _led_armed_state_toggle{0};

	hrt_abstime _overload_start{0};                 ///< time when CPU overload started

	uint8_t _battery_warning{battery_status_s::BATTERY_WARNING_NONE};

	bool _avoidance_system_lost{false};
	bool _circuit_breaker_flight_termination_disabled{false};

	bool _failsafe_old{false};   
	bool _failsafe_user_override_request{false}; ///< override request due to stick movements
	bool _flight_termination_triggered{false};

	bool _imbalanced_propeller_check_triggered{false};
	bool _is_throttle_above_center{false};
	bool _is_throttle_low{false};

	bool _last_overload{false};
	bool _lockdown_triggered{false};
	bool _mode_switch_mapped{false};

	bool _open_drone_id_system_lost{true};
	bool _onboard_controller_lost{false};
	bool _parachute_system_lost{true};

	bool _rtl_time_actions_done{false};
	bool _status_changed{true};

	// Arming flags
	bool _arm_tune_played{false};
	bool _have_taken_off_since_arming{false};
	bool _was_armed{false};

	// Geofence flags
	bool _geofence_loiter_on{false};
	bool _geofence_rtl_on{false};
	bool _geofence_land_on{false};
	bool _primary_geofence_warning_action_on{false};
	bool _primary_geofence_breached_prev{false};
	bool _secondary_geofence_warning_action_on{false};
	bool _secondary_geofence_breached_prev{false};

	// Subscriptions
	uORB::Subscription _action_request_sub{ORB_ID(action_request)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription _geofence_result_sub{ORB_ID(geofence_result)};
	uORB::Subscription _iridiumsbd_status_sub{ORB_ID(iridiumsbd_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _system_power_sub{ORB_ID(system_power)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionMultiArray<telemetry_status_s> _telemetry_status_subs{ORB_ID::telemetry_status};

#if defined(BOARD_HAS_POWER_CONTROL)
	uORB::Subscription _power_button_state_sub {ORB_ID(power_button_state)};
#endif // BOARD_HAS_POWER_CONTROL

	uORB::SubscriptionData<mission_result_s>        _mission_result_sub{ORB_ID(mission_result)};
	uORB::SubscriptionData<offboard_control_mode_s> _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};

	// Publications
	uORB::Publication<actuator_armed_s>          _actuator_armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<actuator_test_s>           _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<commander_state_s>         _commander_state_pub{ORB_ID(commander_state)};
	uORB::Publication<failure_detector_status_s> _failure_detector_status_pub{ORB_ID(failure_detector_status)};
	uORB::Publication<vehicle_command_ack_s>     _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_control_mode_s>    _vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_status_s>          _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_status_flags_s>    _vehicle_status_flags_pub{ORB_ID(vehicle_status_flags)};

	// optional parameters
	param_t _param_mav_comp_id{PARAM_INVALID};
	param_t _param_mav_sys_id{PARAM_INVALID};
	param_t _param_mav_type{PARAM_INVALID};
	param_t _param_rc_map_fltmode{PARAM_INVALID};

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::COM_DISARM_LAND>)  _param_com_disarm_land,
		(ParamFloat<px4::params::COM_DISARM_PRFLT>) _param_com_disarm_preflight,
		(ParamInt<px4::params::COM_DL_LOSS_T>)      _param_com_dl_loss_t,
		(ParamInt<px4::params::COM_HLDL_LOSS_T>)    _param_com_hldl_loss_t,
		(ParamInt<px4::params::COM_HLDL_REG_T>)     _param_com_hldl_reg_t,
		(ParamBool<px4::params::COM_HOME_EN>)       _param_com_home_en,
		(ParamBool<px4::params::COM_HOME_IN_AIR>)   _param_com_home_in_air,
		(ParamInt<px4::params::COM_FLT_PROFILE>)    _param_com_flt_profile,
		(ParamBool<px4::params::COM_FORCE_SAFETY>)  _param_com_force_safety,
		(ParamFloat<px4::params::COM_KILL_DISARM>)  _param_com_kill_disarm,
		(ParamBool<px4::params::COM_MOT_TEST_EN>)   _param_com_mot_test_en,
		(ParamBool<px4::params::COM_OBS_AVOID>)     _param_com_obs_avoid,
		(ParamFloat<px4::params::COM_OBC_LOSS_T>)   _param_com_obc_loss_t,
		(ParamInt<px4::params::COM_PREARM_MODE>)    _param_com_prearm_mode,
		(ParamInt<px4::params::COM_RC_OVERRIDE>)    _param_com_rc_override,
		(ParamInt<px4::params::COM_FLIGHT_UUID>)    _param_flight_uuid,
		(ParamInt<px4::params::COM_TAKEOFF_ACT>)    _param_takeoff_finished_action,
		(ParamFloat<px4::params::COM_CPU_MAX>)      _param_com_cpu_max
	)
};
