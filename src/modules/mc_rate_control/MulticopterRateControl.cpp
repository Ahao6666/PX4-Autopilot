/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
//********ADRC*****
#include <math.h>
#include <stdbool.h>
#include <time.h>
//*************file system*********
#include <iostream>
#include <fstream>	// c++文件操作
#include <iomanip> 	// 设置输出格式
using namespace std;
using namespace matrix;
using namespace time_literals;
using math::radians;

#define SIGMA 0.000001f
//******adrc********
#define pi 3.1416f
#define adrc_p 0.005f		//simulation step size
bool b_EnableAtt_LADRC = 0;
ofstream ofile;


//**********
MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	ADRC_Init();
	file_init();
	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

float
MulticopterRateControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);

		// generate the rate setpoint from sticks?
		bool manual_rate_sp = false;

		if (_v_control_mode.flag_control_manual_enabled &&
		    !_v_control_mode.flag_control_altitude_enabled &&
		    !_v_control_mode.flag_control_velocity_enabled &&
		    !_v_control_mode.flag_control_position_enabled) {

			// landing gear controlled from stick inputs if we are in Manual/Stabilized mode
			//  limit landing gear update rate to 50 Hz
			if (hrt_elapsed_time(&_landing_gear.timestamp) > 20_ms) {
				_landing_gear.landing_gear = get_landing_gear_state();
				_landing_gear.timestamp = hrt_absolute_time();
				_landing_gear_pub.publish(_landing_gear);
			}

			if (!_v_control_mode.flag_control_attitude_enabled) {
				manual_rate_sp = true;
			}

			// Check if we are in rattitude mode and the pilot is within the center threshold on pitch and roll
			//  if true then use published rate setpoint, otherwise generate from manual_control_setpoint (like acro)
			if (_v_control_mode.flag_control_rattitude_enabled) {
				manual_rate_sp =
					(fabsf(_manual_control_sp.y) > _param_mc_ratt_th.get()) ||
					(fabsf(_manual_control_sp.x) > _param_mc_ratt_th.get());
			}

		} else {
			_landing_gear_sub.update(&_landing_gear);
		}

		if (manual_rate_sp) {
			if (manual_control_updated) {

				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(_manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-_manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(_manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = _manual_control_sp.z;

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = v_rates_sp.roll;
				_rates_sp(1) = v_rates_sp.pitch;
				_rates_sp(2) = v_rates_sp.yaw;
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from mixer feedback
			if (_motor_limits_sub.updated()) {
				multirotor_motor_limits_s motor_limits;

				if (_motor_limits_sub.copy(&motor_limits)) {
					MultirotorMixer::saturation_status saturation_status;
					saturation_status.value = motor_limits.saturation_status;

					_rate_control.setSaturationStatus(saturation_status);
				}
			}

			// run rate controller--------------const---------
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			//*************ADRC******************
			phi = rates(0) * 3.14 / 180;
			theta = rates(1) * 3.14 / 180;
			psi = rates(2) * 3.14 / 180;
			phi_ref = _rates_sp(0) * 3.14 / 180;
			theta_ref = _rates_sp(1) * 3.14 / 180;
			psi_ref = _rates_sp(2) * 3.14 / 180;

			get_vehicle_status();
			AttiRateADRC_Ctrl();
			float roll_control = AMP_Limit(NLSEFState_Roll.u * 0.01,-0.3,0.3);	//us
			float pitch_control = AMP_Limit(NLSEFState_Pitch.u * 0.01,-0.3,0.3);	//us
			float yaw_control = AMP_Limit(NLSEFState_Yaw.u * 0.01,-0.3,0.3);	//us
			ofile <<now<<","<<NLSEFState_Roll.u<<","<<NLSEFState_Pitch.u<<","<<NLSEFState_Yaw.u<<","
				<<att_control(0)<<","<<att_control(1)<<","<<att_control(2)<<","
				<<roll_control<<","<<pitch_control<<","<<yaw_control<<","
				<<att_q.q[0]<<","<<att_q.q[1]<<","<<att_q.q[2]<<","<<att_q.q[3]<<","
				<<local_pos.x<<","<<local_pos.y<<","<<local_pos.z<<","
				<<local_pos.vx<<","<<local_pos.vy<<","<<local_pos.vz<<","<<endl;
			//******************************************


			// instead the PID control law with ADRC
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(roll_control) ? roll_control : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(pitch_control) ? pitch_control : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(yaw_control) ? yaw_control : 0.0f;
			// actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			// actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			// actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = (float)_landing_gear.landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			// scale effort by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status)) {
						_battery_status_scale = battery_status.scale;	//1.0 <= scale <= 1.3
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

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

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}


/* declaration */
bool b_Enableladrc_test=0;
extern bool b_EnableAtt_LADRC;


/*	PD 	tsÊ±¼ä³£Êý£¬uÉÏÊ±¿ÌÖµ*/
AdrcPD_TypeDef Roll_PD;
AdrcPD_TypeDef Pitch_PD;
AdrcPD_TypeDef Yaw_PD;

/* newfal */
newfal_TypeDef Att_newfal;
newfal_TypeDef Rate_newfal;

/* fal a1Î»ÖÃÖ¸Êý£¬a2ËÙ¶ÈÖ¸Êý*/
fal_TypeDef ESO_fal;
fal_TypeDef NLSEF_fal;

/* TD rËÙ¶ÈÒò×Ó£¬Ò»°ãÎªÆÚÍûÊäÈëµÄ±³±¶Êý£¬cÂË²¨Òò×Ó*/
fhanParas_TypeDef TD_fhanParas_RollRadio;
fhanParas_TypeDef TD_fhanParas_PitchRadio;
fhanParas_TypeDef TD_fhanParas_YawRadio;
/*x1¸ú×ÙµÄÁ¿£¬x2¸ú×ÙµÄËÙ¶ÈÁ¿*/
TDState_TypeDef TDState_RollRadio;
TDState_TypeDef TDState_PitchRadio;
TDState_TypeDef TDState_YawRadio;

/* ESO À©ÕÅ×´Ì¬¹Û²âÆ÷£bb1¡¢b2¡¢b3 ¡ª¡ª ¹Û²âÆ÷ÔöÒæ,b ¡ª¡ª ¿ØÖÆÁ¿¶ÔÓ¦ÓÚ½Ç¼ÓËÙ¶ÈÁ¿µÄÔöÒæ,c ¡ª¡ª ÂË²¨Òò×Ó*/
ESOParas_TypeDef ESOParas_Roll;
ESOParas_TypeDef ESOParas_Pitch;
ESOParas_TypeDef ESOParas_Yaw;
/*z1¡¢z2¡¢z3 ¡ª¡ª ¹Û²âÆ÷¹Û²âµÄÎ»ÖÃ¡¢ËÙ¶È¡¢¼ÓËÙ¶ÈÁ¿*/
ESOState_TypeDef ESOState_Roll;
ESOState_TypeDef ESOState_Pitch;
ESOState_TypeDef ESOState_Yaw;


/* LESO */
/*************************** ADRC ******************
***@arthor:  YZN
***@data:    18/6/21
***@About:   LESO
						 1. function
								linear extend state observer


***
***
***************************************************/
/*wo ¡ª¡ª ¹Û²âÆ÷´ø¿í
	b ¡ª¡ª ¿ØÖÆÁ¿¶ÔÓ¦ÓÚ½Ç¼ÓËÙ¶ÈÁ¿µÄÔöÒæ*/
LESOParas_TypeDef LESOParas_Pitch;
LESOParas_TypeDef LESOParas_Roll;
//z1¡¢z2¡¢z3 ¡ª¡ª ¹Û²âÆ÷¹Û²âµÄÎ»ÖÃ¡¢ËÙ¶È¡¢¼ÓËÙ¶ÈÁ¿
LESOState_TypeDef LESOState_Pitch;

FourthOrder_LESOState_TypeDef FourthOrder_LESOState_Pitch;
FourthOrder_LESOState_TypeDef FourthOrder_LESOState_Roll;

/* NLSEF */
 /*				   b1 ¡ª¡ª Î»ÖÃÎó²î·´À¡ÔöÒæ
						 b2 ¡ª¡ª ËÙ¶ÈÎó²î·´À¡ÔöÒæ
						 b  ¡ª¡ª ¿ØÖÆÁ¿¶ÔÓ¦ÓÚ½Ç¼ÓËÙ¶ÈÁ¿µÄÔöÒæ
						 c  ¡ª¡ª Æ«²îÔöÒæ
						 a1 ¡ª¡ª ±ÈÀý·´À¡fal£¨£©Ö¸Êý²ÎÊý
						 a2 ¡ª¡ª Î¢·Ö·´À¡fal£¨£©Ö¸Êý²ÎÊý
						 u  ¡ª¡ª Êä³öµÄ¿ØÖÆÁ¿
*/
NLSEFState_TypeDef NLSEFState_Roll;
NLSEFState_TypeDef NLSEFState_Pitch;
NLSEFState_TypeDef NLSEFState_Yaw;

/* LSEF
						 b  ¡ª¡ª ¿ØÖÆÁ¿¶ÔÓ¦ÓÚ½Ç¼ÓËÙ¶ÈÁ¿µÄÔöÒæ
						 wc ¡ª¡ª ¿ØÖÆÆ÷´ø¿í
						 u  ¡ª¡ª Êä³öµÄ¿ØÖÆÁ¿
*/

LSEFState_TypeDef LSEFState_Roll;
LSEFState_TypeDef LSEFState_Pitch;
LSEFState_TypeDef LSEFState_Yaw;


static void ADRCPD_Init(void)
{

	Roll_PD.Ts=0.0667f;//0.0667
	Roll_PD.u=0;

	Pitch_PD.Ts=0.0667f;//0.0667
	Pitch_PD.u=0;

	Yaw_PD.Ts=0.0667f;//0.0667
	Yaw_PD.u=0;
}


static void fal_Init(void)
{
	ESO_fal.a1=0.5;
	ESO_fal.a2=0.75;   //0.25

	NLSEF_fal.a1=0.75;  //0.5
	NLSEF_fal.a2=1.25;

}
static void fhan_Init(void)				//fhan_init()
{
	TD_fhanParas_RollRadio.c=10;
	TD_fhanParas_RollRadio.r=pi;

	TD_fhanParas_PitchRadio.c=10;
	TD_fhanParas_PitchRadio.r=pi;

	TD_fhanParas_YawRadio.c=10;
	TD_fhanParas_YawRadio.r=pi;
}

static void TD_Init(void)
{
	TDState_RollRadio.ch=1;
	TDState_RollRadio.x1=0;
	TDState_RollRadio.x2=0;

	TDState_PitchRadio.ch=2;
	TDState_PitchRadio.x1=0;
	TDState_PitchRadio.x2=0;

	TDState_YawRadio.ch=3;
	TDState_YawRadio.x1=0;
	TDState_YawRadio.x2=0;
}

static void ESO_Init(void)
{
	ESOState_Roll.ch=1;
	ESOParas_Roll.b1=200;
	ESOParas_Roll.b2=1767;
	ESOParas_Roll.b3=13420;
	ESOParas_Roll.b=0.2;
	ESOParas_Roll.c=20;  // ¸ÃÖµ´óÓÐÂË²¨×÷ÓÃ
	ESOState_Roll.z1=0;
	ESOState_Roll.z2=0;
	ESOState_Roll.z3=0;

	ESOState_Pitch.ch=2;
	ESOParas_Pitch.b1=200;
	ESOParas_Pitch.b2=1767;
	ESOParas_Pitch.b3=13420;
	ESOParas_Pitch.b=0.2;
	ESOParas_Pitch.c=20;
	ESOState_Pitch.z1=0;
	ESOState_Pitch.z2=0;
	ESOState_Pitch.z3=0;

	LESOParas_Pitch.b=2.0;
	LESOParas_Pitch.wo=60.00;
	LESOState_Pitch.ch=2;
	LESOState_Pitch.z1=0;
	LESOState_Pitch.z2=0;
	LESOState_Pitch.z3=0;

	LESOParas_Roll.b=3;
	LESOParas_Roll.wo=60;

	FourthOrder_LESOState_Pitch.ch=2;
	FourthOrder_LESOState_Pitch.z1=0;
	FourthOrder_LESOState_Pitch.z2=0;
	FourthOrder_LESOState_Pitch.z3=0;
	FourthOrder_LESOState_Pitch.z4=0;

	FourthOrder_LESOState_Roll.ch=1;
	FourthOrder_LESOState_Roll.z1=0;
	FourthOrder_LESOState_Roll.z2=0;
	FourthOrder_LESOState_Roll.z3=0;
	FourthOrder_LESOState_Roll.z4=0;

	ESOState_Yaw.ch=3;
	ESOParas_Yaw.b1=200;
	ESOParas_Yaw.b2=1767;
	ESOParas_Yaw.b3=13420;
	ESOParas_Yaw.b=0.02;
	ESOParas_Yaw.c=20;
	ESOState_Yaw.z1=0;
	ESOState_Yaw.z2=0;
	ESOState_Yaw.z3=0;
}

static void NLSEF_Init(void)
{
	NLSEFState_Roll.ch=1;
	NLSEFState_Roll.b1=120;
	NLSEFState_Roll.b2=120;
	NLSEFState_Roll.b=0.5;
	NLSEFState_Roll.c=5;
	NLSEFState_Roll.u=0;

	NLSEFState_Pitch.ch=2;
	NLSEFState_Pitch.b1=120;
	NLSEFState_Pitch.b2=120;
	NLSEFState_Pitch.b=0.5; //b=100
	NLSEFState_Pitch.c=5;
	NLSEFState_Pitch.u=0;

	NLSEFState_Yaw.ch=3;
	NLSEFState_Yaw.b1=180;
	NLSEFState_Yaw.b2=140;
	NLSEFState_Yaw.b=0.05;
	NLSEFState_Yaw.c=5;
	NLSEFState_Yaw.u=0;
}

static void LSEF_Init(void)
{
	LSEFState_Roll.ch=1;
	LSEFState_Roll.b=3;
	LSEFState_Roll.wc=5.0;
	LSEFState_Roll.u=0;

	LSEFState_Pitch.ch=2;
	LSEFState_Pitch.b=2;
	LSEFState_Pitch.wc=7.0;
	LSEFState_Pitch.u=0;

	LSEFState_Yaw.ch=3;
	LSEFState_Yaw.b=0.05;
	LSEFState_Yaw.wc=5;
	LSEFState_Yaw.u=0;
}

void ADRC_Init(void)
{
	fal_Init();
	ADRCPD_Init();
	fhan_Init();
	TD_Init();
	ESO_Init();//
	NLSEF_Init();
	LSEF_Init();
	warnx("ADRC Init!");
}

float AMP_Limit(float in, float low, float up)
{
	if (in < low)
		in = low;
	else if (in > up)
		in = up;

	return in;
}

void ADRC_LPF(float u,AdrcLPF_TypeDef *para)
{
	para->x1=(para->k*adrc_p*u+para->t*para->x2)/(adrc_p+para->t);
	para->x2=para->x1;

}
float ADRC_PD(float input,AdrcPD_TypeDef *para)
{
	float value;
	value=para->Ts*(input-para->u)/adrc_p;
	value=AMP_Limit(value,-50,50);//
	value+=input;
	para->u=input;
	return value;
}
/* ------------------------- TD -------------------------------*/
static float fhan(float x1,float x2,fhanParas_TypeDef *para)
{
	float h,d,d0,y,a0,r,a;
	float res;
	r=para->r;
	h=para->c*adrc_p;
	d=r*h;
	d0=h*d;

	y=x1+h*x2;
	a0=sqrt(d*d+8*r*fabs(y));


	if(fabs(y)>d0)
	{
		if(y>=0.0f)
			a=x2+(a0-d)/2;
		else
			a=x2-(a0-d)/2;
	}
	else
		a=x2+y/h;

	if(fabs(a)>d)
	{
		if(a>0)
			res=-r;
		else
			res=r;
	}
	else
		res=-r*a/d;

	return res;
}

void TD_Atti(TDState_TypeDef *state,float v,fhanParas_TypeDef *para)
{
	float fh,e;
	float x1,x2;
	x1=state->x1;
	x2=state->x2;
	e=x1-v;
	fh=fhan(e,x2,para);
	state->x1=x1+x2*adrc_p;
	state->x2=x2+fh*adrc_p;
}


/*------------------------  ESO ----------------------------------*/

static float fal(float e,float a,float s,float m)
{
	float abs_value,res;
	abs_value=fabs(e);
	if(abs_value>m)
	{
		if(e>0)
			res=pow(abs_value,a);
		else
			res=-pow(abs_value,a);
	}
	else
		res=e/s;
	return res;
}

void ESO_Atti(const double y,const double u,ESOParas_TypeDef *para,ESOState_TypeDef *state)
{
	double e,fe,fe1,m;//adrc
	double z1,z2,z3;


	m=adrc_p*para->c;
	ESO_fal.s1=pow(m,1-ESO_fal.a1);
	ESO_fal.s2=pow(m,1-ESO_fal.a2);

	z1=state->z1;
	z2=state->z2;
	z3=state->z3;



	e=(double)state->z1-y;
	fe=fal(e,ESO_fal.a1,ESO_fal.s1,m);
	fe1=fal(e,ESO_fal.a2,ESO_fal.s2,m);
	state->z1=z1+(z2-para->b1*e)*adrc_p;
	state->z2=z2+(z3-para->b2*fe+para->b*u)*adrc_p;
	state->z3=z3-para->b3*fe1*adrc_p;

}


void LESO_Atti(const int init,const double y,double u,LESOParas_TypeDef *para,LESOState_TypeDef *state)
{
	double e;//adrc_p
	double b1,b2,b3;
	double z1,z2,z3;
	int count;

	count=init;

	if(count==1)
	{
		z1=y;
		z2=0;
		z3=0;
		u=0;
	}
	else
	{
			z1=state->z1;
		  z2=state->z2;
	  	z3=state->z3;
	}

	b1=3*para->wo;
	b2=3*para->wo*para->wo;
	b3=para->wo*para->wo*para->wo;

	e=state->z1-y;
	state->z1=z1+(z2-b1*e)*adrc_p;
  state->z2=z2+(z3-b2*e+para->b*u)*adrc_p;
	state->z3=z3-b3*e*adrc_p;

}


void FourthOrder_LESO_Atti(const int init,const double y,double u,LESOParas_TypeDef *para,FourthOrder_LESOState_TypeDef *state)
{
	double e;//adrc_p
	double b1,b2,b3,b4;
	double z1,z2,z3,z4;
	int count;

	count=init;

	if(count==1)
	{
		z1=y;
		z2=0;
		z3=0;
		z4=0;
		u=0;
	}
	else
	{
			z1=state->z1;
		  z2=state->z2;
	  	z3=state->z3;
		  z4=state->z4;
	}

	b1=4*para->wo;													//4*wo
	b2=6*para->wo*para->wo;									//6*wo^2
	b3=4*para->wo*para->wo*para->wo;				//4*wo^3
	b4=para->wo*para->wo*para->wo*para->wo;	//wo^4

	e=state->z1-y;
	state->z1=z1+(z2-b1*e)*adrc_p;
  state->z2=z2+(z3-b2*e)*adrc_p;
	state->z3=z3+(z4-b3*e+para->b*u)*adrc_p;//-z3/0.667
	state->z4=z4-b4*e*adrc_p;
}


void NLSEF_Atti(TDState_TypeDef *tdstate,ESOState_TypeDef *esostate,NLSEFState_TypeDef *nlsefstate)
{

	float e1,e2,u1,u2;
	float m;


	m=adrc_p*nlsefstate->c;
	NLSEF_fal.s1=pow(m,1-NLSEF_fal.a1);
	NLSEF_fal.s2=pow(m,1-NLSEF_fal.a2);


	e1=tdstate->x1-esostate->z1;
	e2=tdstate->x2-esostate->z2;


	u1=fal(e1,NLSEF_fal.a1,NLSEF_fal.s1,m);
	u2=fal(e2,NLSEF_fal.a2,NLSEF_fal.s2,m);


	nlsefstate->u=nlsefstate->b1*u1+nlsefstate->b2*u2-esostate->z3/nlsefstate->b;


}


void LSEF_Atti(TDState_TypeDef *tdstate,LESOState_TypeDef *lesostate,LSEFState_TypeDef *lsefstate)
{

	float e1,e2;

	e1=tdstate->x1-lesostate->z1;
	e2=lesostate->z2;

	lsefstate->u=(lsefstate->wc*lsefstate->wc*e1-2*lsefstate->wc*e2-lesostate->z3)/lsefstate->b;

}

void FourthOrder_LSEF_Atti(TDState_TypeDef *tdstate,FourthOrder_LESOState_TypeDef *lesostate,LSEFState_TypeDef *lsefstate)
{

	float e1,e2,e3;

	e1=tdstate->x1-lesostate->z1;
	e2=lesostate->z2;
	e3=lesostate->z3;

	lsefstate->u=(lsefstate->wc*lsefstate->wc*lsefstate->wc*e1-3*lsefstate->wc*lsefstate->wc*e2-3*lsefstate->wc*e3-lesostate->z4)/lsefstate->b;//+lesostate->z3/0.667

}


void  AttiRateADRC_Ctrl(void)
{
	static int Roll_count=0;
	static int Pitch_count=0;

	/* Roll Channel */
	TD_Atti(&TDState_RollRadio,phi_ref,&TD_fhanParas_RollRadio);

	if(b_EnableAtt_LADRC)//ADRCÇÐ»»³É4½×LADRC
	{
		Roll_count++;
		FourthOrder_LESO_Atti(Roll_count,phi,LSEFState_Roll.u,&LESOParas_Roll,&FourthOrder_LESOState_Roll);
		FourthOrder_LSEF_Atti(&TDState_RollRadio,&FourthOrder_LESOState_Roll,&LSEFState_Roll);
	}
	else								//3½×·ÇÏßÐÔADRC
	{
		Roll_count=0;
		ESO_Atti(phi,NLSEFState_Roll.u,&ESOParas_Roll,&ESOState_Roll);
		NLSEF_Atti(&TDState_RollRadio,&ESOState_Roll,&NLSEFState_Roll);
	}

	/* Pitch Channel*/
	TD_Atti(&TDState_PitchRadio,theta_ref,&TD_fhanParas_PitchRadio);

	if(b_EnableAtt_LADRC)// || b_Enableladrc_test
	{
		Pitch_count++;
		//LESO_Atti(count,theta,LSEFState_Pitch.u,&LESOParas_Pitch,&LESOState_Pitch);
		//LSEF_Atti(&TDState_PitchRadio,&LESOState_Pitch,&LSEFState_Pitch);
		FourthOrder_LESO_Atti(Pitch_count,theta,LSEFState_Pitch.u,&LESOParas_Pitch,&FourthOrder_LESOState_Pitch);
		FourthOrder_LSEF_Atti(&TDState_PitchRadio,&FourthOrder_LESOState_Pitch,&LSEFState_Pitch);
	}
	else
	{
		Pitch_count=0;
		ESO_Atti(theta,NLSEFState_Pitch.u,&ESOParas_Pitch,&ESOState_Pitch);
		NLSEF_Atti(&TDState_PitchRadio,&ESOState_Pitch,&NLSEFState_Pitch);
	}

	if(true)	//(!b_EnableDirectPSI)
	{
			/* Yaw Channel */
		TD_Atti(&TDState_YawRadio,psi_ref,&TD_fhanParas_YawRadio);
		ESO_Atti(psi,NLSEFState_Yaw.u,&ESOParas_Yaw,&ESOState_Yaw);
		NLSEF_Atti(&TDState_YawRadio,&ESOState_Yaw,&NLSEFState_Yaw);
	}
	else
	{
		TDState_YawRadio.x1=psi;
		TDState_YawRadio.x2=0;
		ESOState_Yaw.z1=psi;
		ESOState_Yaw.z2=0;
		ESOState_Yaw.z3=0;
		NLSEFState_Yaw.u=0;
	}

	// NLSEFState_Roll.u = AMP_Limit(NLSEFState_Roll.u,-200,200);	//us
	// NLSEFState_Pitch.u = AMP_Limit(NLSEFState_Pitch.u,-150,150);	//us
	// NLSEFState_Yaw.u = AMP_Limit(NLSEFState_Yaw.u,-150,150);	//us

}

void file_init(){
	char s[60];
	struct tm tim;
	time_t now = time(NULL);
	tim = *(localtime(&now));
	strftime(s,60,"control_quantity_%b_%d_%H_%M.csv",&tim);
	string path =  string(s);

	//std::ofstream ofile;
	ofile.open(path.c_str(),ios::out | ios::app);

	//ofile.open("control_actuator.csv", ios::out | ios::app);	//default path is ~/.ros
	if(ofile)
	{
		warnx("file init!");
		ofile <<"time(us)"<<"roll_adrc"<<","<<"pitch_adrc"<<","<<"yaw_adrc"<<","
			<<"roll_pid"<<","<<"pitch_pid"<<","<<"yaw_pid" <<","
			<<"roll_control"<<","<<"pitch_control"<<","<<"yaw_control" <<","
			<<"att_q[0]"<<","<<"att_q[1]"<<","<<"att_q[2]"<<","<<"att_q[3]"<<","
			<<"local_pos.x"<<","<<"local_pos.y"<<","<<"local_pos.z"<<","
			<<"local_pos.vx"<<","<<"local_pos.vy"<<","<<"local_pos.vz"<<"\n";
	}
	vehicle_att_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	vehicle_local_pos_fd = orb_subscribe(ORB_ID(vehicle_local_position));

}
void get_vehicle_status(){
	memset(&att_q, 0 , sizeof(att_q));
	memset(&local_pos, 0 , sizeof(local_pos));
	orb_copy(ORB_ID(vehicle_attitude), vehicle_att_fd, &att_q);
	orb_copy(ORB_ID(vehicle_local_position),vehicle_local_pos_fd, &local_pos);
}
