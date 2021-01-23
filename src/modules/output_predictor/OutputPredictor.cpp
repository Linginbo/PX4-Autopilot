
#include "OutputPredictor.hpp"


OutputPredictor::OutputPredictor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

bool OutputPredictor::init()
{
	if (!_vehicle_imu_sub.registerCallback()) {
		PX4_ERR("vehicle_imu callback registration failed!");
		return false;
	}

	ScheduleNow();

	return true;
}

void OutputPredictor::Run()
{
	perf_begin(_cycle_perf);

	if (_sensor_selection_sub.updated()) {
		sensor_selection_s sensor_selectio;

		if (_sensor_selection_sub.copy(&sensor_selectio)) {
			for (uint8_t i = 0; i < 4; i++) {
				uORB::SubscriptionData<vehicle_imu_s> vehicle_imu{ORB_ID(vehicle_imu), i};

				if (vehicle_imu.get().accel_device_id == sensor_selectio.accel_device_id) {
					_vehicle_imu_sub.ChangeInstance(i);
					break;
				}
			}
		}
	}

	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_states_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	bool states_updated = false;
	estimator_states_s states;

	if (_estimator_states_sub.update(&states)) {
		_state.quat_nominal = Quatf{states.states[0], states.states[1], states.states[2], states.states[3]};
		_state.vel = Vector3f{states.states[4], states.states[5], states.states[6]};
		_state.pos = Vector3f{states.states[7], states.states[8], states.states[9]};
		_delta_ang_bias = Vector3f{states.states[10], states.states[11], states.states[12]};
		_delta_vel_bias = Vector3f{states.states[13], states.states[14], states.states[15]};

		if (_time_last_estimator_states != 0) {
			const float dt = (states.timestamp_sample - _time_last_estimator_states) * 1e-6f;
			_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * dt;
		}

		_time_last_estimator_states = states.timestamp_sample;

		states_updated = true;
	}

	vehicle_imu_s imu;

	if (_vehicle_imu_sub.update(&imu)) {

		imuSample imu_sample {
			.time_us = imu.timestamp_sample,
			.delta_ang = Vector3f{imu.delta_angle},
			.delta_vel = Vector3f{imu.delta_velocity},
			.delta_ang_dt = imu.delta_angle_dt * 1.e-6f,
			.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f,
		};

		const float dt = math::constrain((imu_sample.time_us - _time_last_imu) * 1e-6f, 0.0001f, 0.02f);

		if (_time_last_imu > 0) {
			_dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;
		}

		_time_last_imu = imu_sample.time_us;

		_imu_buffer.push(imu_sample);

		// get the oldest data from the buffer
		_imu_sample_delayed = _imu_buffer.get_oldest();

		// the output observer always runs
		// Use full rate IMU data at the current time horizon
		calculateOutputStates(imu_sample);

		if (states_updated) {
			correctOutputStates(imu_sample);
		}
	}

	perf_end(_cycle_perf);
}

void OutputPredictor::calculateOutputStates(const imuSample &imu)
{
	// Use full rate IMU data at the current time horizon

	// correct delta angles for bias offsets
	const float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle(imu.delta_ang - _delta_ang_bias * dt_scale_correction + _delta_angle_corr);

	const Quatf dq(AxisAnglef{delta_angle});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.time_us = imu.time_us;
	_output_new.quat_nominal = _output_new.quat_nominal * dq;
	_output_new.quat_nominal.normalize(); // the quaternions must always be normalised after modification

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = _output_new.quat_nominal;

	// correct delta velocity for bias offsets
	const Vector3f delta_vel_body{imu.delta_vel - _delta_vel_bias * dt_scale_correction};

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{_R_to_earth_now * delta_vel_body};

	// correct for measured acceleration due to gravity
	delta_vel_earth(2) += CONSTANTS_ONE_G * imu.delta_vel_dt;

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.vel);

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (imu.delta_vel_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += imu.delta_vel_dt;


	// Publish
	output_predictor_s output_predictor{};
	// generate vehicle local position data
	output_predictor.timestamp_sample = imu.time_us;

	_output_new.quat_nominal.copyTo(output_predictor.q);

	// Position of body origin in local NED frame
	// rotate the position of the IMU relative to the boy origin into earth frame
	const Vector3f pos_offset_earth{_R_to_earth_now * _params.imu_pos_body};
	// subtract from the EKF position (which is at the IMU) to get position at the body origin
	Vector3f(_output_new.pos - pos_offset_earth).copyTo(output_predictor.position);

	// Velocity of body origin in local NED frame (m/s)
	// correct velocity for IMU offset

	// calculate the average angular rate across the last IMU update
	const Vector3f ang_rate{imu.delta_ang *(1.f / imu.delta_ang_dt)};

	// calculate the velocity of the IMU relative to the body origin
	const Vector3f vel_imu_rel_body{ang_rate % _params.imu_pos_body};

	// rotate the relative velocity into earth frame
	const Vector3f vel_imu_rel_body_ned{_R_to_earth_now * vel_imu_rel_body};
	Vector3f(_output_new.vel - vel_imu_rel_body_ned).copyTo(output_predictor.velocity);

	// vertical position time derivative (m/s)
	output_predictor.z_deriv = _output_vert_new.vert_vel - vel_imu_rel_body_ned(2);

	// Acceleration of body origin in local frame
	// calculate the earth frame velocity derivatives
	const Vector3f vel_deriv{delta_vel_earth *(1.f / imu.delta_vel_dt)};
	vel_deriv.copyTo(output_predictor.acceleration);

	output_predictor.delta_angle_error_norm = _delta_angle_error_norm;
	output_predictor.velocity_error_norm = _velocity_error_norm;
	output_predictor.position_error_norm = _position_error_norm;

	output_predictor.timestamp = hrt_absolute_time();
	_output_predictor_pub.publish(output_predictor);
}

void OutputPredictor::correctOutputStates(const imuSample &imu)
{
	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer

	_output_buffer.push(_output_new);
	_output_vert_buffer.push(_output_vert_new);

	// get the oldest INS state data from the ring buffer
	// this data will be at the EKF fusion time horizon
	// TODO: there is no guarantee that data is at delayed fusion horizon
	//       Shouldnt we use pop_first_older_than?
	const outputSample &output_delayed = _output_buffer.get_oldest();
	const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

	// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
	const Quatf q_error((_state.quat_nominal.inversed() * output_delayed.quat_nominal).normalized());

	// convert the quaternion delta to a delta angle
	const float scalar = (q_error(0) >= 0.f) ? -2.f : 2.f;

	const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

	// calculate a gain that provides tight tracking of the estimator attitude states and
	// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
	const float time_delay = fmaxf((imu.time_us - _imu_sample_delayed.time_us) * 1e-6f, _dt_imu_avg);
	const float att_gain = 0.5f * _dt_imu_avg / time_delay;

	// calculate a corrrection to the delta angle
	// that will cause the INS to track the EKF quaternions
	_delta_angle_corr = delta_ang_error * att_gain;
	_delta_angle_error_norm = delta_ang_error.norm();

	/*
	 * Loop through the output filter state history and apply the corrections to the velocity and position states.
	 * This method is too expensive to use for the attitude states due to the quaternion operations required
	 * but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
	 * to be used and reduces tracking error relative to EKF states.
	 */

	// Complementary filter gains
	const float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.f);
	const float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.f);

	// calculate down velocity and position tracking errors
	const float vert_vel_err = (_state.vel(2) - output_vert_delayed.vert_vel);
	const float vert_vel_integ_err = (_state.pos(2) - output_vert_delayed.vert_vel_integ);

	// calculate a velocity correction that will be applied to the output state history
	// using a PD feedback tuned to a 5% overshoot
	const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

	applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

	// calculate velocity and position tracking errors
	const Vector3f vel_err(_state.vel - output_delayed.vel);
	const Vector3f pos_err(_state.pos - output_delayed.pos);

	_velocity_error_norm = vel_err.norm();
	_position_error_norm = pos_err.norm();

	// calculate a velocity correction that will be applied to the output state history
	_vel_err_integ += vel_err;
	const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

	// calculate a position correction that will be applied to the output state history
	_pos_err_integ += pos_err;
	const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

	applyCorrectionToOutputBuffer(vel_correction, pos_correction);
}

void OutputPredictor::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
	// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// correct the velocity
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f *
					    next_state.dt;

		// advance the index
		index = (index + 1) % size;
	}

	// update output state to corrected values
	_output_vert_new = _output_vert_buffer.get_newest();

	// reset time delta to zero for the next accumulation of full rate IMU data
	_output_vert_new.dt = 0.f;
}

void OutputPredictor::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// loop through the output filter state history and apply the corrections to the velocity and position states
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// a constant velocity correction is applied
		_output_buffer[index].vel += vel_correction;

		// a constant position correction is applied
		_output_buffer[index].pos += pos_correction;
	}

	// update output state to corrected values
	_output_new = _output_buffer.get_newest();
}

int OutputPredictor::task_spawn(int argc, char *argv[])
{
	OutputPredictor *instance = new OutputPredictor();

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

int OutputPredictor::print_status()
{
	PX4_INFO("dt_imu_avg: %.6f", (double)_dt_imu_avg);
	PX4_INFO("dt_ekf_avg: %.6f", (double)_dt_ekf_avg);
	perf_print_counter(_cycle_perf);
	return 0;
}

int OutputPredictor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OutputPredictor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("output_predictor", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int output_predictor_main(int argc, char *argv[])
{
	return OutputPredictor::main(argc, argv);
}
