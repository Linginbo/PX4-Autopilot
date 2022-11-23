/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_rate_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Pitch rate proportional gain.
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_P, 0.08f);

/**
 * Pitch rate derivative gain.
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_D, 0.f);

/**
 * Pitch rate integrator gain.
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I, 0.1f);

/**
 * Pitch rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.4f);

/**
 * Roll rate proportional Gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

/**
 * Roll rate derivative Gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_D, 0.00f);

/**
 * Roll rate integrator Gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I, 0.1f);

/**
 * Roll integrator anti-windup
 *
 * The portion of the integrator part in the control surface deflection is limited to this value.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

/**
 * Yaw rate proportional gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05f);

/**
 * Yaw rate derivative gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_D, 0.0f);

/**
 * Yaw rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output. Use this
 * to obtain a tigher response of the controller without introducing
 * noise amplification.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_FF, 0.5f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_FF, 0.5f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

/**
 * Acro body x max rate.
 *
 * This is the rate the controller is trying to achieve if the user applies full roll
 * stick input in acro mode.
 *
 * @min 45
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_X_MAX, 90);

/**
 * Acro body y max rate.
 *
 * This is the body y rate the controller is trying to achieve if the user applies full pitch
 * stick input in acro mode.
 *
 * @min 45
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Y_MAX, 90);

/**
 * Acro body z max rate.
 *
 * This is the body z rate the controller is trying to achieve if the user applies full yaw
 * stick input in acro mode.
 *
 * @min 10
 * @max 180
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Z_MAX, 45);

/**
 * Whether to scale throttle by battery power level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The fixed wing
 * should constantly behave as if it was fully charged with reduced max thrust
 * at lower battery percentages. i.e. if cruise speed is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_BAT_SCALE_EN, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (FW_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ARSP_SCALE_EN, 1);

/**
* Roll trim increment at minimum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMIN, 0.0f);

/**
* Pitch trim increment at minimum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMIN, 0.0f);

/**
* Yaw trim increment at minimum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMIN, 0.0f);

/**
* Roll trim increment at maximum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMAX, 0.0f);

/**
* Pitch trim increment at maximum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMAX, 0.0f);

/**
* Yaw trim increment at maximum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMAX, 0.0f);

/**
 * Roll trim increment for flaps configuration
 *
 * This increment is added to TRIM_ROLL whenever flaps are fully deployed.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_FLPS, 0.0f);

/**
 * Pitch trim increment for flaps configuration
 *
 * This increment is added to the pitch trim whenever flaps are fully deployed.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_FLPS, 0.0f);

/**
 * Pitch trim increment for spoiler configuration
 *
 * This increment is added to the pitch trim whenever spoilers are fully deployed.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_SPOIL, 0.f);

/**
 * Flaps setting during take-off
 *
 * Sets a fraction of full flaps during take-off.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_TO_SCL, 0.0f);

/**
 * Flaps setting during landing
 *
 * Sets a fraction of full flaps during landing.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_LND_SCL, 1.0f);

/**
 * Manual roll scale
 *
 * Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_SC, 1.0f);

/**
 * Manual pitch scale
 *
 * Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_SC, 1.0f);

/**
 * Manual yaw scale
 *
 * Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_Y_SC, 1.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator (rudder, airbrakes, ...) to counteract
 * this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RLL_TO_YAW_FF, 0.0f);
