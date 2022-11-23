/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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
 * @file tecs.cpp
 *
 * @author Paul Riseborough
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/tecs_status.h>
#include <uORB/uORB.h>
#include <motion_planning/VelocitySmoothing.hpp>
#include <motion_planning/ManualVelocitySmoothingZ.hpp>

class TECSAirspeedFilter
{
public:
	/**
	 * @brief State of the equivalent airspeed filter.
	 *
	 */
	struct AirspeedFilterState {
		float speed;		///< speed of the air in EAS [m/s]
		float speed_rate;	///< rate of speed of the air [m/s²]
	};

	/**
	 * @brief Parameters of the airspeed filter.
	 *
	 */
	struct Param {
		float equivalent_airspeed_trim; 	///< the trim value of the equivalent airspeed om [m/s].
		float airspeed_measurement_std_dev; 	///< airspeed measurement standard deviation in [m/s].
		float airspeed_rate_measurement_std_dev;///< airspeed rate measurement standard deviation in [m/s²].
		float airspeed_rate_noise_std_dev; 	///< standard deviation on the airspeed rate deviation in the model in [m/s²].
	};

	/**
	 * @brief Input, which will be filtered.
	 *
	 */
	struct Input {
		float equivalent_airspeed; 	///< the measured equivalent airspeed in [m/s].
		float equivalent_airspeed_rate;	///< the measured rate of equivalent airspeed in [m/s²].
	};
public:
	TECSAirspeedFilter() = default;
	~TECSAirspeedFilter() = default;
	/**
	 * @brief Initialize filter
	 *
	 * @param[in] equivalent_airspeed is the equivalent airspeed in [m/s].
	 */
	void initialize(float equivalent_airspeed);

	/**
	 * @brief Update filter
	 *
	 * @param[in] dt is the timestep in [s].
	 * @param[in] input are the raw measured values.
	 * @param[in] param are the filter parameters.
	 * @param[in] airspeed_sensor_available boolean if the airspeed sensor is available.
	 */
	void update(float dt, const Input &input, const Param &param, const bool airspeed_sensor_available);

	/**
	 * @brief Get the filtered airspeed states.
	 *
	 * @return Current state of the airspeed filter.
	 */
	AirspeedFilterState getState() const;

private:
	// States
	AirspeedFilterState _airspeed_state{.speed = 0.0f, .speed_rate = 0.0f};	///< Complimentary filter state
};

class TECSReferenceModel
{
public:
	/**
	 * @brief Altitude reference state.
	 *
	 */
	struct AltitudeReferenceState {
		float alt; 	///< Reference altitude amsl in [m].
		float alt_rate;	///< Reference altitude rate in [m/s].
	};

	/**
	 * @brief Parameters for the reference model.
	 *
	 */
	struct Param {
		float target_climbrate;	///< The target climbrate in [m/s].
		float target_sinkrate;	///< The target sinkrate in [m/s].
		float jerk_max;		///< Magnitude of the maximum jerk allowed [m/s³].
		float vert_accel_limit;	///< Magnitude of the maximum vertical acceleration allowed [m/s²].
		float max_climb_rate;	///< Climb rate produced by max allowed throttle [m/s].
		float max_sink_rate;	///< Maximum safe sink rate [m/s].
	};

public:
	TECSReferenceModel() = default;
	~TECSReferenceModel() = default;

	/**
	 * @brief Initialize reference models.
	 *
	 * @param[in] state is the current altitude state of the vehicle.
	 */
	void initialize(const AltitudeReferenceState &state);

	/**
	 * @brief Update reference models.
	 *
	 * @param[in] dt is the update interval in [s].
	 * @param[in] setpoint are the desired setpoints.
	 * @param[in] altitude is the altitude amsl in [m].
	 * @param[in] param are the reference model parameters.
	 */
	void update(float dt, const AltitudeReferenceState &setpoint, float altitude, const Param &param);

	/**
	 * @brief Get the current altitude reference of altitude reference model.
	 *
	 * @return Altitude reference state.
	 */
	AltitudeReferenceState getAltitudeReference() const;

	/**
	 * @brief Get the altitude rate reference of the altitude rate reference model.
	 *
	 * @return Current altitude rate reference point.
	 */
	float getAltitudeRateReference() const;

private:
	// State
	VelocitySmoothing
	_alt_control_traj_generator;		///< Generates altitude rate and altitude setpoint trajectory when altitude is commanded.

	// Output
	float _alt_rate_ref; 			///< Altitude rate reference in [m/s].
};

class TECSControl
{
public:
	/**
	 * @brief The control parameters.
	 *
	 */
	struct Param {
		// Vehicle specific params
		float max_sink_rate;			///< Maximum safe sink rate [m/s].
		float max_climb_rate;			///< Climb rate produced by max allowed throttle [m/s].
		float vert_accel_limit;			///< Magnitude of the maximum vertical acceleration allowed [m/s²].
		float equivalent_airspeed_trim;		///< Equivalent cruise airspeed for airspeed less mode [m/s].
		float tas_min;				///< True airpeed demand lower limit [m/s].
		float pitch_max;			///< Maximum pitch angle allowed in [rad].
		float pitch_min;			///< Minimal pitch angle allowed in [rad].
		float throttle_trim;			///< Normalized throttle required to fly level at given eas.
		float throttle_max;			///< Normalized throttle upper limit.
		float throttle_min;			///< Normalized throttle lower limit.

		// Altitude control param
		float altitude_error_gain;		///< Altitude error inverse time constant [1/s].
		float altitude_setpoint_gain_ff;	///< Gain from altitude demand derivative to demanded climb rate.

		// Airspeed control param
		/// [0,1] percentage of true airspeed trim corresponding to expected (safe) true airspeed tracking errors
		float tas_error_percentage;
		float airspeed_error_gain;				///< Airspeed error inverse time constant [1/s].

		// Energy control param
		float ste_rate_time_const;				///< Filter time constant for specific total energy rate (damping path) [s].
		float seb_rate_ff;					///< Specific energy balance rate feedforward gain.

		// Pitch control param
		float pitch_speed_weight;				///< Speed control weighting used by pitch demand calculation.
		float integrator_gain_pitch;				///< Integrator gain used by the pitch demand calculation.
		float pitch_damping_gain;				///< Damping gain of the pitch demand calculation [s].

		// Throttle control param
		float integrator_gain_throttle;				///< Integrator gain used by the throttle demand calculation.
		float throttle_damping_gain;				///< Damping gain of the throttle demand calculation [s].
		float throttle_slewrate;				///< Throttle demand slew rate limit [1/s].

		float load_factor_correction;				///< Gain from normal load factor increase to total energy rate demand [m²/s³].
		float load_factor;					///< Additional normal load factor.
	};

	/**
	 * @brief The debug output
	 *
	 */
	struct DebugOutput {
		float altitude_rate_control;			///< Altitude rate setpoint from altitude control loop [m/s].
		float true_airspeed_derivative_control;		///< Airspeed rate setpoint from airspeed control loop [m/s²].
		float total_energy_rate_error;			///< Total energy rate error [m²/s³].
		float total_energy_rate_sp;			///< Total energy rate setpoint [m²/s³].
		float energy_balance_rate_error;		///< Energy balance rate error [m²/s³].
		float energy_balance_rate_sp;			///< Energy balance rate setpoint [m²/s³].
	};

	/**
	 * @brief Given setpoint to control.
	 *
	 */
	struct Setpoint {
		TECSReferenceModel::AltitudeReferenceState altitude_reference;	///< Altitude reference from reference model.
		float altitude_rate_setpoint;					///< Altitude rate setpoint.
		float tas_setpoint;						///< True airspeed setpoint.
	};

	/**
	 * @brief Givent current measurement from the UAS.
	 *
	 */
	struct Input {
		float altitude;		///< Current altitude of the UAS [m].
		float altitude_rate;	///< Current altitude rate of the UAS [m/s].
		float tas;		///< Current true airspeed of the UAS [m/s].
		float tas_rate;		///< Current true airspeed rate of the UAS [m/s²].
	};

	/**
	 * @brief Control flags.
	 *
	 */
	struct Flag {
		bool airspeed_enabled;			///< Flag if the airspeed sensor is enabled.
		bool climbout_mode_active;		///< Flag if climbout mode is activated.
		bool detect_underspeed_enabled;		///< Flag if underspeed detection is enabled.
	};
public:
	TECSControl() = default;
	~TECSControl() = default;
	/**
	 * @brief Initialization of the state.
	 *
	 */
	void initialize();
	/**
	 * @brief Update state and output.
	 *
	 * @param[in] dt is the update time intervall in [s].
	 * @param[in] setpoint is the current setpoint struct.
	 * @param[in] input is the current input measurements.
	 * @param[in] param is the current parameter set.
	 * @param[in] flag is the current activated flags.
	 */
	void update(float dt, const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag);
	/**
	 * @brief Reset the control loop integrals.
	 *
	 */
	void resetIntegrals();
	/**
	 * @brief Get the percent of the undersped.
	 *
	 * @return Percentage of detected undersped.
	 */
	float getPercentUndersped() const {return _percent_undersped;};
	/**
	 * @brief Get the throttle setpoint.
	 *
	 * @return throttle setpoint.
	 */
	float getThrottleSetpoint() const {return _throttle_setpoint;};
	/**
	 * @brief Get the pitch setpoint.
	 *
	 * @return THe commanded pitch angle in [rad].
	 */
	float getPitchSetpoint() const {return _pitch_setpoint;};
	/**
	 * @brief Get specific total energy rate.
	 *
	 * @return the total specific energy rate in [m²/s³].
	 */
	float getSteRate() const {return _ste_rate;};
	/**
	 * @brief Get the Debug Output
	 *
	 * @return the debug outpus struct.
	 */
	DebugOutput getDebugOutput() const {return _debug_output;};

private:
	/**
	 * @brief Specific total energy limit.
	 *
	 */
	struct STELimit {
		float STE_rate_max;	///< Maximum specific total energy rate limit [m²/s³].
		float STE_rate_min;	///< Minimal specific total energy rate limit [m²/s³].
	};

	/**
	 * @brief Calculated specific energy rates.
	 *
	 */
	struct SpecificEnergyRates {
		struct {
			float rate;		///< Specific kinetic energy rate in [m²/s³].
			float rate_setpoint;	///< Specific kinetic energy setpoint rate in [m²/s³].
			float rate_error;	///< Specific kinetic energy rate error in [m²/s³].
		} ske;				///< Specific kinetic energy.
		struct {
			float rate;		///< Specific potential energy rate in [m²/s³].
			float rate_setpoint;	///< Specific potential energy setpoint rate in [m²/s³].
			float rate_error;	///< Specific potential energy rate error in [m²/s³].
		} spe;				///< Specific potential energy rate.
	};

	/**
	 * @brief Controlled altitude and pitch setpoints.
	 *
	 */
	struct AltitudePitchControl {
		float altitude_rate_setpoint;	///< Controlled altitude rate setpoint [m/s].
		float tas_rate_setpoint;	///< Controlled true airspeed rate setpoint [m/s²].
	};

	/**
	 * @brief Weight factors for specific energy.
	 *
	 */
	struct SpecificEnergyWeighting {
		float spe_weighting;	///< Specific potential energy weight.
		float ske_weighting;	///< Specific kinetic energy weight.
	};

private:
	/**
	 * @brief Calculate specific total energy rate limits.
	 *
	 * @param[in] param are the control parametes.
	 * @return Specific total energy rate limits.
	 */
	STELimit _calculateTotalEnergyRateLimit(const Param &param) const;
	/**
	 * @brief Airspeed control loop.
	 *
	 * @param setpoint is the control setpoints.
	 * @param input	is the current input measurment of the UAS.
	 * @param param	is the control parameters.
	 * @param flag	is the control flags.
	 * @return controlled airspeed rate setpoint in [m/s²].
	 */
	float _airspeedControl(const Setpoint &setpoint, const Input &input, const Param &param, const Flag &flag) const;
	/**
	 * @brief Altitude control loop.
	 *
	 * @param setpoint is the control setpoints.
	 * @param input is the current input measurment of the UAS.
	 * @param param is the control parameters.
	 * @return controlled altitude rate setpoint in [m/s].
	 */
	float _altitudeControl(const Setpoint &setpoint, const Input &input, const Param &param) const;
	/**
	 * @brief Calculate specific energy rates.
	 *
	 * @param control_setpoint is the controlles altitude and airspeed rate setpoints.
	 * @param input is the current input measurment of the UAS.
	 * @return Specific energy rates.
	 */
	SpecificEnergyRates _calcSpecificEnergyRates(const AltitudePitchControl &control_setpoint, const Input &input) const;
	/**
	 * @brief Detect underspeed.
	 *
	 * @param input is the current input measurment of the UAS.
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 */
	void _detectUnderspeed(const Input &input, const Param &param, const Flag &flag);
	/**
	 * @brief Update specific energy balance weights.
	 *
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 * @return Weights used for the specific energy balance.
	 */
	SpecificEnergyWeighting _updateSpeedAltitudeWeights(const Param &param, const Flag &flag);
	/**
	 * @brief Update controlled pitch setpoint.
	 *
	 * @param dt is the update time intervall in [s].
	 * @param input is the current input measurment of the UAS.
	 * @param se is the calculated specific energy.
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 */
	void _updatePitchSetpoint(float dt, const Input &input, const SpecificEnergyRates &se, Param &param, const Flag &flag);
	/**
	 * @brief Update controlled throttle setpoint.
	 *
	 * @param dt is the update time intervall in [s].
	 * @param se is the calculated specific energy.
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 */
	void _updateThrottleSetpoint(float dt, const SpecificEnergyRates &se, const Param &param, const Flag &flag);

private:
	// State
	AlphaFilter<float> _ste_rate_error_filter;		///< Low pass filter for the specific total energy rate.
	float _pitch_integ_state{0.0f};				///< Pitch integrator state [rad].
	float _throttle_integ_state{0.0f};			///< Throttle integrator state.


	// Output
	DebugOutput _debug_output;
	float _pitch_setpoint{0.0f};				///< Controlled pitch setpoint [rad].
	float _throttle_setpoint{0.0f};				///< Controlled throttle setpoint.
	float _percent_undersped{0.0f};				///< A continuous representation of how "undersped" the TAS is [0,1]
	float _ste_rate{0.0f};					///< Specific total energy rate [m²/s³].
};

class TECS
{
public:
	enum ECL_TECS_MODE {
		ECL_TECS_MODE_NORMAL = 0,
		ECL_TECS_MODE_UNDERSPEED,
		ECL_TECS_MODE_BAD_DESCENT,
		ECL_TECS_MODE_CLIMBOUT
	};

	struct DebugOutput : TECSControl::DebugOutput {
		float true_airspeed_filtered;
		float true_airspeed_derivative;
		float altitude_sp;
		float altitude_rate;
		float altitude_rate_setpoint;
		enum ECL_TECS_MODE tecs_mode;
	};
public:
	TECS() = default;
	~TECS() = default;

	// no copy, assignment, move, move assignment
	TECS(const TECS &) = delete;
	TECS &operator=(const TECS &) = delete;
	TECS(TECS &&) = delete;
	TECS &operator=(TECS &&) = delete;

	DebugOutput getStatus() const {return _debug_status;};

	/**
	 * Get the current airspeed status
	 *
	 * @return true if airspeed is enabled for control
	 */
	bool airspeed_sensor_enabled() { return _airspeed_enabled; }

	/**
	 * Set the airspeed enable state
	 */
	void enable_airspeed(bool enabled) { _airspeed_enabled = enabled; }

	/**
	 * @brief Update the control loop calculations
	 *
	 */
	void update(float pitch, float altitude, float hgt_setpoint, float EAS_setpoint, float equivalent_airspeed,
		    float eas_to_tas, bool climb_out_setpoint, float pitch_min_climbout, float throttle_min, float throttle_setpoint_max,
		    float throttle_trim, float pitch_limit_min, float pitch_limit_max, float target_climbrate, float target_sinkrate,
		    const float speed_deriv_forward, float hgt_rate, float hgt_rate_sp = NAN);

	/**
	 * @brief Initialize the control loop
	 *
	 */
	void initialize(const float altitude, const float altitude_rate, const float equivalent_airspeed);

	void resetIntegrals()
	{
		_control.resetIntegrals();
	}

	void set_detect_underspeed_enabled(bool enabled) { _detect_underspeed_enabled = enabled; };

	// // setters for parameters
	void set_airspeed_measurement_std_dev(float std_dev) {_airspeed_param.airspeed_measurement_std_dev = std_dev;};
	void set_airspeed_rate_measurement_std_dev(float std_dev) {_airspeed_param.airspeed_rate_measurement_std_dev = std_dev;};
	void set_airspeed_filter_process_std_dev(float std_dev) {_airspeed_param.airspeed_rate_noise_std_dev = std_dev;};

	void set_integrator_gain_throttle(float gain) { _control_param.integrator_gain_throttle = gain;};
	void set_integrator_gain_pitch(float gain) { _control_param.integrator_gain_pitch = gain; };

	void set_max_sink_rate(float sink_rate) { _control_param.max_sink_rate = sink_rate; _reference_param.max_sink_rate = sink_rate; };
	void set_max_climb_rate(float climb_rate) { _control_param.max_climb_rate = climb_rate; _reference_param.max_climb_rate = climb_rate; };

	void set_altitude_rate_ff(float altitude_rate_ff) { _control_param.altitude_setpoint_gain_ff = altitude_rate_ff; };
	void set_altitude_error_time_constant(float time_const) { _control_param.altitude_error_gain = 1.0f / math::max(time_const, 0.1f);; };

	void set_equivalent_airspeed_max(float airspeed) { _equivalent_airspeed_max = airspeed; }
	void set_equivalent_airspeed_min(float airspeed) { _equivalent_airspeed_min = airspeed; }
	void set_equivalent_airspeed_trim(float airspeed) { _equivalent_airspeed_trim = airspeed; }

	void set_pitch_damping(float damping) { _control_param.pitch_damping_gain = damping; }
	void set_vertical_accel_limit(float limit) { _reference_param.vert_accel_limit = limit; _control_param.vert_accel_limit = limit; };

	void set_speed_weight(float weight) { _control_param.pitch_speed_weight = weight; };
	void set_airspeed_error_time_constant(float time_const) { _control_param.airspeed_error_gain = 1.0f / math::max(time_const, 0.1f); };

	void set_throttle_damp(float throttle_damp) { _control_param.throttle_damping_gain = throttle_damp; };
	void set_throttle_slewrate(float slewrate) { _control_param.throttle_slewrate = slewrate; };

	void set_roll_throttle_compensation(float compensation) { _control_param.load_factor_correction = compensation; };
	void set_load_factor(float load_factor) { _control_param.load_factor = load_factor; };

	void set_ste_rate_time_const(float time_const) { _control_param.ste_rate_time_const = time_const; };

	void set_seb_rate_ff_gain(float ff_gain) { _control_param.seb_rate_ff = ff_gain; };

	/**
	 * Handle the altitude reset
	 *
	 * If the estimation system resets the height in one discrete step this
	 * will gracefully even out the reset over time.
	 */
	void handle_alt_step(float altitude, float altitude_rate)
	{
		TECSReferenceModel::AltitudeReferenceState init_state{ .alt = altitude,
				.alt_rate = altitude_rate};

		// reset altitude reference model.
		_reference_model.initialize(init_state);
	}

	float get_pitch_setpoint() {return _control.getPitchSetpoint();}
	float get_throttle_setpoint() {return _control.getThrottleSetpoint();}

	// // TECS status
	uint64_t timestamp() { return _update_timestamp; }
	ECL_TECS_MODE tecs_mode() { return _tecs_mode; }

	static constexpr float DT_DEFAULT = 0.02f;

private:
	TECSControl 		_control;				///< Control submodule.
	TECSAirspeedFilter 	_airspeed_filter;			///< Airspeed filter submodule.
	TECSReferenceModel 	_reference_model;			///< Setpoint reference model submodule.

	enum ECL_TECS_MODE _tecs_mode {ECL_TECS_MODE_NORMAL};		///< Current activated mode.

	uint64_t _update_timestamp{0};					///< last timestamp of the update function call.

	float _equivalent_airspeed_min{3.0f};				///< equivalent airspeed demand lower limit (m/sec)
	float _equivalent_airspeed_max{30.0f};				///< equivalent airspeed demand upper limit (m/sec)
	float _equivalent_airspeed_trim{15.0f};				///< equivalent cruise airspeed for airspeed less mode (m/sec)

	// controller mode logic
	bool _uncommanded_descent_recovery{false};			///< true when a continuous descent caused by an unachievable airspeed demand has been detected
	bool _airspeed_enabled{false};					///< true when airspeed use has been enabled
	bool _detect_underspeed_enabled{false};				///< true when underspeed detection is enabled

	static constexpr float DT_MIN = 0.001f;				///< minimum allowed value of _dt (sec)
	static constexpr float DT_MAX = 1.0f;				///< max value of _dt allowed before a filter state reset is performed (sec)

	static constexpr float _jerk_max = 1000.0f;			///< Magnitude of the maximum jerk allowed [m/s³].
	static constexpr float _tas_error_percentage =
		0.15f;		///< [0,1] percentage of true airspeed trim corresponding to expected (safe) true airspeed tracking errors

	DebugOutput _debug_status{};

	// Params
	/// Airspeed filter parameters.
	TECSAirspeedFilter::Param _airspeed_param{
		.equivalent_airspeed_trim = 0.0f,
		.airspeed_measurement_std_dev = 0.2f,
		.airspeed_rate_measurement_std_dev = 0.05f,
		.airspeed_rate_noise_std_dev = 0.02
	};
	/// Reference model parameters.
	TECSReferenceModel::Param _reference_param{
		.target_climbrate = 2.0f,
		.target_sinkrate = 2.0f,
		.jerk_max = _jerk_max,
		.vert_accel_limit = 0.0f,
		.max_climb_rate = 2.0f,
		.max_sink_rate = 2.0f,
	};
	/// Control parameters.
	TECSControl::Param _control_param{
		.max_sink_rate = 2.0f,
		.max_climb_rate = 2.0f,
		.vert_accel_limit = 0.0f,
		.equivalent_airspeed_trim = 15.0f,
		.tas_min = 3.0f,
		.pitch_max = 5.0f,
		.pitch_min = -5.0f,
		.throttle_trim = 0.0f,
		.throttle_max = 1.0f,
		.throttle_min = 0.1f,
		.altitude_error_gain = 0.2f,
		.altitude_setpoint_gain_ff = 0.0f,
		.tas_error_percentage = _tas_error_percentage,
		.airspeed_error_gain = 0.1f,
		.ste_rate_time_const = 0.1f,
		.seb_rate_ff = 1.0f,
		.pitch_speed_weight = 1.0f,
		.integrator_gain_pitch = 0.0f,
		.pitch_damping_gain = 0.0f,
		.integrator_gain_throttle = 0.0f,
		.throttle_damping_gain = 0.0f,
		.throttle_slewrate = 0.0f,
		.load_factor_correction = 0.0f,
		.load_factor = 1.0f,
	};

	/**
	 * Update the desired airspeed
	 */
	float _update_speed_setpoint(const float tas_min, const float tas_max, const float tas_setpoint, const float tas);

	/**
	 * Detect an uncommanded descent
	 */
	void _detect_uncommanded_descent(float throttle_setpoint_max, float altitude, float altitude_setpoint, float tas,
					 float tas_setpoint);
};

