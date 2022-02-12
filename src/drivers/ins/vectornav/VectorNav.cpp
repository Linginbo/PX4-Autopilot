/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "VectorNav.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

VectorNav::VectorNav(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_accel(DRV_INS_DEVTYPE_VN300),
	_px4_gyro(DRV_INS_DEVTYPE_VN300),
	_px4_mag(DRV_INS_DEVTYPE_VN300),
	_px4_baro(DRV_INS_DEVTYPE_VN300)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	// device::Device::DeviceId device_id;
	// device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TFMINI;
	// device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	// uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	// if (bus_num < 10) {
	// 	device_id.devid_s.bus = bus_num;
	// }

	// _px4_rangefinder.set_device_id(device_id.devid);
	// _px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

VectorNav::~VectorNav()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void VectorNav::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(100_ms);
}

void VectorNav::stop()
{
	ScheduleClear();
}

void VectorNav::asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	/* First make sure we have a binary packet type we expect since there
	 * are many types of binary output types that can be configured. */
	// COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL
	if ((VnUartPacket_type(packet) == PACKETTYPE_BINARY) &&
	    VnUartPacket_isCompatible(packet,
				      COMMONGROUP_NONE,
				      TIMEGROUP_NONE,
				      (ImuGroup)(IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE),
				      GPSGROUP_NONE,
				      ATTITUDEGROUP_NONE,
				      INSGROUP_NONE,
				      GPSGROUP_NONE)) {

		// vec3f ypr = VnUartPacket_extractVec3f(packet);

		// char strConversions[50];
		// str_vec3f(strConversions, ypr);
		// PX4_INFO("Binary Async YPR: %s\n", strConversions);

		if (userData) {
			VectorNav *vn = static_cast<VectorNav *>(userData);

			BinaryGroupType groups = (BinaryGroupType)VnUartPacket_groups(packet);
			size_t curGroupFieldIndex = 0;

			if ((groups & BINARYGROUPTYPE_IMU) != 0) {
				ImuGroup imuGroup = (ImuGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

				if (imuGroup & IMUGROUP_TEMP) {
					//float temperature = VnUartPacket_extractFloat(packet);
				}

				if (imuGroup & IMUGROUP_PRES) {
					float pressure = VnUartPacket_extractFloat(packet);
					vn->PublishBaro(time_now_us, pressure);
				}

				if (imuGroup & IMUGROUP_MAG) {
					vec3f magnetic = VnUartPacket_extractVec3f(packet);
					vn->PublishMag(time_now_us, magnetic.c[0], magnetic.c[1], magnetic.c[2]);
				}

				if (imuGroup & IMUGROUP_ACCEL) {
					vec3f acceleration = VnUartPacket_extractVec3f(packet);
					vn->PublishAccel(time_now_us, acceleration.c[0], acceleration.c[1], acceleration.c[2]);
				}

				if (imuGroup & IMUGROUP_ANGULARRATE) {
					vec3f angularRate = VnUartPacket_extractVec3f(packet);
					vn->PublishGyro(time_now_us, angularRate.c[0], angularRate.c[1], angularRate.c[2]);
				}
			}
		}

		// compositeData->quaternion = VnUartPacket_extractVec4f(packet);
		// compositeData->velocityEstimatedNed = VnUartPacket_extractVec3f(packet);


		// VnCompositeData_processBinaryPacketGps2Group




	}
}

void VectorNav::PublishAccel(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	_px4_accel.update(timestamp_sample, x, y, z);
}

void VectorNav::PublishGyro(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	_px4_gyro.update(timestamp_sample, x, y, z);
}

void VectorNav::PublishMag(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	_px4_mag.update(timestamp_sample, x, y, z);
}

void VectorNav::PublishBaro(const hrt_abstime &timestamp_sample, float pressure)
{
	//_px4_mag.set_temperature(temperature);
	_px4_baro.update(timestamp_sample, pressure);
}

int VectorNav::init()
{
	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a VectorNav sensor using the VnSensor structure. */

	/* First determine which COM port your sensor is attached to and update the
	 * constant below. Also, if you have changed your sensor from the factory
	 * default baudrate of 115200, you will need to update the baudrate
	 * constant below as well. */
	const char SENSOR_PORT[] = "/dev/ttyUSB0"; /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	const uint32_t SENSOR_BAUDRATE = 115200;


	VnSensor_initialize(&_vs);

	VnError error;

	/* Now connect to our sensor. */
	if ((error = VnSensor_connect(&_vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE) {
		PX4_ERR("Error connecting to sensor %d", error);
		return PX4_ERROR;
	}

	/* Let's query the sensor's model number. */
	char modelNumber[30];

	if ((error = VnSensor_readModelNumber(&_vs, modelNumber, sizeof(modelNumber))) != E_NONE) {
		PX4_ERR("Error reading model number %d", error);
		return PX4_ERROR;
	}

	PX4_INFO("Model Number: %s", modelNumber);




	/* For the registers that have more complex configuration options, it is
	 * convenient to read the current existing register configuration, change
	 * only the values of interest, and then write the configuration to the
	 * register. This allows preserving the current settings for the register's
	 * other fields. Below, we change the heading mode used by the sensor. */
	VpeBasicControlRegister vpeReg;

	if ((error = VnSensor_readVpeBasicControl(&_vs, &vpeReg)) != E_NONE) {
		PX4_ERR("Error reading VPE basic control %d", error);
	}

	char strConversions[30] {};
	strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
	printf("Old Heading Mode: %s\n", strConversions);
	vpeReg.headingMode = VNHEADINGMODE_ABSOLUTE;

	if ((error = VnSensor_writeVpeBasicControl(&_vs, vpeReg, true)) != E_NONE) {
		PX4_ERR("Error writing VPE basic control %d", error);
	}

	if ((error = VnSensor_readVpeBasicControl(&_vs, &vpeReg)) != E_NONE) {
		PX4_ERR("Error reading VPE basic control %d", error);
	}

	strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
	printf("New Heading Mode: %s\n", strConversions);

	ImuGroup imu_group = (ImuGroup)((int)IMUGROUP_ACCEL | (int)IMUGROUP_ANGULARRATE);

	BinaryOutputRegister_initialize(
		&_binary_output_400hz,
		ASYNCMODE_BOTH,
		10, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		imu_group,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE);

	if ((error = VnSensor_writeBinaryOutput1(&_vs, &_binary_output_400hz, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 1 %d", error);
		return PX4_ERROR;
	}

	VnSensor_registerAsyncPacketReceivedHandler(&_vs, VectorNav::asciiOrBinaryAsyncMessageReceived, this);

	return PX4_OK;
}

void VectorNav::Run()
{

	// TODO: shutdown






	// fds initialized?
	if (_fd < 0) {
		// open fd
		//_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}
}

void VectorNav::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
