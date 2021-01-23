#pragma once

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/output_predictor.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu.h>

#include "RingBuffer.hpp"

using namespace matrix;
using namespace time_literals;

class OutputPredictor : public ModuleBase<OutputPredictor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	OutputPredictor();
	~OutputPredictor() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:

	struct stateSample {
		Quatf  quat_nominal;	///< quaternion defining the rotation from body to earth frame
		Vector3f    vel;	///< NED velocity in earth frame in m/s
		Vector3f    pos;	///< NED position in earth frame in m
	};

	struct outputSample {
		uint64_t    time_us{0};	///< timestamp of the measurement (uSec)
		Quatf  quat_nominal;	///< nominal quaternion describing vehicle attitude
		Vector3f    vel;	///< NED velocity estimate in earth frame (m/sec)
		Vector3f    pos;	///< NED position estimate in earth frame (m/sec)
	};

	struct outputVert {
		uint64_t    time_us{0};		///< timestamp of the measurement (uSec)
		float	    vert_vel;		///< Vertical velocity calculated using alternative algorithm (m/sec)
		float	    vert_vel_integ;	///< Integral of vertical velocity (m)
		float	    dt;			///< delta time (sec)
	};

	struct imuSample {
		uint64_t    time_us{0};		///< timestamp of the measurement (uSec)
		Vector3f    delta_ang;		///< delta angle in body frame (integrated gyro measurements) (rad)
		Vector3f    delta_vel;		///< delta velocity in body frame (integrated accelerometer measurements) (m/sec)
		float       delta_ang_dt;	///< delta angle integration period (sec)
		float       delta_vel_dt;	///< delta velocity integration period (sec)
	};

	void Run() override;

	/*
	 * Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
	 * Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
	 * Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
	 * current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
	 * The inspiration for using a complementary filter to correct for time delays in the EKF
	 * is based on the work by A Khosravian:
	 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
	 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
	*/
	void calculateOutputStates(const imuSample &imu);
	void correctOutputStates(const imuSample &imu);

	/*
	* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
	* down position state at the fusion time horizon using an alternative algorithm to what
	* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
	* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
	* This provides an alternative vertical velocity output that is closer to the first derivative
	* of the position but does degrade tracking relative to the EKF state.
	*/
	void applyCorrectionToVerticalOutputBuffer(float vert_vel_correction);

	/*
	* Calculate corrections to be applied to vel and pos output state history.
	* The vel and pos state history are corrected individually so they track the EKF states at
	* the fusion time horizon. This option provides the most accurate tracking of EKF states.
	*/
	void applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction);

	template<typename T> static constexpr T sq(T x) { return x * x; }

	uORB::Publication<output_predictor_s> _output_predictor_pub{ORB_ID(output_predictor)};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_states_sub{ORB_ID(estimator_states)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

	//calibration::Accelerometer _accel_calibration{};
	//calibration::Gyroscope _gyro_calibration{};

	imuSample _imu_sample_delayed{}; // captures the imu sample on the delayed time horizon

	float _dt_imu_avg{0.01f}; // average imu update period in s
	float _dt_ekf_avg{0.01f}; // average update rate of the ekf

	hrt_abstime _time_last_imu{0};
	hrt_abstime _time_last_estimator_states{0};

	stateSample _state{};

	Vector3f _delta_ang_bias{}; ///< delta angle bias estimate in rad
	Vector3f _delta_vel_bias{}; ///< delta velocity bias estimate in m/s

	// output predictor states
	Vector3f _delta_angle_corr;	///< delta angle correction vector (rad)
	Vector3f _vel_err_integ;	///< integral of velocity tracking error (m)
	Vector3f _pos_err_integ;	///< integral of position tracking error (m.s)

	float _delta_angle_error_norm{0.f};
	float _velocity_error_norm{0.f};
	float _position_error_norm{0.f};

	Dcmf _R_to_earth_now{};		// rotation matrix from body to earth frame at current time

	// Output Predictor
	outputSample _output_new{};		// filter output on the non-delayed time horizon
	outputVert _output_vert_new{};		// vertical filter output on the non-delayed time horizon

	RingBuffer<imuSample> _imu_buffer{24};           // buffer length 12 with default parameters
	RingBuffer<outputSample> _output_buffer{24};
	RingBuffer<outputVert> _output_vert_buffer{24};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	struct {
		// output complementary filter tuning
		float vel_Tau{0.25f};			///< velocity state correction time constant (1/sec)
		float pos_Tau{0.25f};			///< position state correction time constant (1/sec)

		Vector3f imu_pos_body{};		///< xyz position of IMU in body frame (m)
	} _params{};
};




