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
			// get_vehicle_status();
			double control_actuator_roll = AttiRateADRC_Ctrl(rates, _rates_sp, _maybe_landed || _landed);
			float roll_rate_control = AMP_Limit( control_actuator_roll, -1.0, 1.0);	//us
			// float pitch_rate_control = AMP_Limit(NLSEFState_Pitch.u * 0.01,-0.3,0.3);	//us
			// float yaw_rate_control = AMP_Limit(NLSEFState_Yaw.u * 0.01,-0.3,0.3);	//us
			// ofile <<now<<","<<NLSEFState_Roll.u<<","<<NLSEFState_Pitch.u<<","<<NLSEFState_Yaw.u<<","
			// 	<<att_control(0)<<","<<att_control(1)<<","<<att_control(2)<<","
			// 	<<roll_rate_control<<","<<pitch_rate_control<<","<<yaw_rate_control<<","
			// 	<<phi_rate_ref<<","<<TDState_RollRadio.x1<<","<<TDState_RollRadio.x2<<","
			// 	<<phi_rate<<","<<ESOState_Roll.z1<<","<<ESOState_Roll.z2<<","<<ESOState_Roll.z3<<","<<endl;
			//******************************************

			// instead the PID control law with ADRC
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(roll_rate_control) ? roll_rate_control : 0.0f;
			// actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(pitch_rate_control) ? pitch_rate_control : 0.0f;
			// actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(yaw_rate_control) ? yaw_rate_control : 0.0f;
			// actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
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

//-------------------ADRC-------------
/* TD parameters*/
fhanParas_TypeDef TD_fhanParas_RollRadio;
fhanParas_TypeDef TD_fhanParas_PitchRadio;
fhanParas_TypeDef TD_fhanParas_YawRadio;

/*TD states*/
TDState_TypeDef TDState_RollRadio;
TDState_TypeDef TDState_PitchRadio;
TDState_TypeDef TDState_YawRadio;

/* ESO parameters*/
ESOParas_TypeDef ESOParas_Roll;
ESOParas_TypeDef ESOParas_Pitch;
ESOParas_TypeDef ESOParas_Yaw;

/*ESO states*/
ESOState_TypeDef ESOState_Roll;
ESOState_TypeDef ESOState_Pitch;
ESOState_TypeDef ESOState_Yaw;

/* NLSEF */
NLSEFState_TypeDef NLSEFState_Roll;
NLSEFState_TypeDef NLSEFState_Pitch;
NLSEFState_TypeDef NLSEFState_Yaw;

static void fhan_Init(void)
{
	TD_fhanParas_RollRadio.h=0.005;
	TD_fhanParas_RollRadio.r=50;

	TD_fhanParas_PitchRadio.h=0.005;
	TD_fhanParas_PitchRadio.r=50;

	TD_fhanParas_YawRadio.h=0.005;
	TD_fhanParas_YawRadio.r=50;
}

static void TD_Init(void)
{
	TDState_RollRadio.h=0.005;
	TDState_RollRadio.x1=0;
	TDState_RollRadio.x2=0;

	TDState_PitchRadio.h=0.005;
	TDState_PitchRadio.x1=0;
	TDState_PitchRadio.x2=0;

	TDState_YawRadio.h=0.005;
	TDState_YawRadio.x1=0;
	TDState_YawRadio.x2=0;
}

static void ESO_Init(void)
{
	ESOParas_Roll.h = 0.005;
	ESOParas_Roll.b = 7;
	ESOParas_Roll.b1 = 50;
	ESOParas_Roll.b2 = 150;
	ESOParas_Roll.b3 = 800;
	ESOParas_Roll.a1 = 0.7;
	ESOParas_Roll.a2 = 0.1;
	ESOParas_Roll.d = 0.05;

	ESOState_Roll.z1 = 0;
	ESOState_Roll.z2 = 0;
	ESOState_Roll.z3 = 0;

	ESOParas_Pitch.h=0.005;
	ESOParas_Pitch.b=7;
	ESOParas_Pitch.b1=50;
	ESOParas_Pitch.b2=150;
	ESOParas_Pitch.b3=800;
	ESOParas_Pitch.a1=0.7;
	ESOParas_Pitch.a2=0.1;
	ESOParas_Pitch.d=0.05;

	ESOState_Pitch.z1=0;
	ESOState_Pitch.z2=0;
	ESOState_Pitch.z3=0;

	ESOParas_Yaw.h=0.005;
	ESOParas_Yaw.b=7;
	ESOParas_Yaw.b1=50;
	ESOParas_Yaw.b2=150;
	ESOParas_Yaw.b3=800;
	ESOParas_Yaw.a1=0.7;
	ESOParas_Yaw.a2=0.1;
	ESOParas_Yaw.d=0.05;

	ESOState_Yaw.z1=0;
	ESOState_Yaw.z2=0;
	ESOState_Yaw.z3=0;
}

static void NLSEF_Init(void)
{
	NLSEFState_Roll.b1=5;		//120;
	NLSEFState_Roll.b2=2.5;		//120;
	NLSEFState_Roll.b=7;	//0.5;
	NLSEFState_Roll.a1=0.6;
	NLSEFState_Roll.a2=0.9;
	NLSEFState_Roll.d=0.02;
	NLSEFState_Roll.u=0;

	NLSEFState_Pitch.b1=5;
	NLSEFState_Pitch.b2=2.5;
	NLSEFState_Pitch.b=7;
	NLSEFState_Pitch.a1=0.6;
	NLSEFState_Pitch.a2=0.9;
	NLSEFState_Pitch.d=0.02;
	NLSEFState_Pitch.u=0;

	NLSEFState_Yaw.b1=5;
	NLSEFState_Yaw.b2=2.5;
	NLSEFState_Yaw.b=7;
	NLSEFState_Yaw.a1=0.6;
	NLSEFState_Yaw.a2=0.9;
	NLSEFState_Yaw.d=0.02;
	NLSEFState_Yaw.u=0;
}

void ADRC_Init(void)
{
	fhan_Init();
	TD_Init();
	ESO_Init();//
	NLSEF_Init();
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

/* ------------------------- TD -------------------------------*/
static float fhan(float x1,float x2,fhanParas_TypeDef *para)
{
	float h, d, d0, y, a0, r, a;
	float res;
	r = para->r;
	h = para->h;
	d = r * h;
	d0 = h * d;

	y = x1 + h * x2;
	a0 = sqrt(d*d + 8 * r * fabs(y));

	if (fabs(y) > d0)
	{
		if (y >= 0.0f)
			a = x2 + (a0 - d) / 2;
		else
			a = x2 - (a0 - d) / 2;
	}
	else
		a = x2 + y / h;

	if (fabs(a) > d)
	{
		if (a > 0)
			res = -r;
		else
			res = r;
	}
	else
		res = -r * a / d;

	return res;
}

void TD_Atti(TDState_TypeDef *state,float v,fhanParas_TypeDef *para)
{
	float fh;
	float x1, x2;
	x1 = state->x1;
	x2 = state->x2;
	fh = fhan(x1 - v, x2, para);
	state->x1 = x1 + x2 * state->h;
	state->x2 = x2 + fh * state->h;
}


/*------------------------  ESO ----------------------------------*/
void ESO_Atti(const double y,const double u,ESOParas_TypeDef *para,ESOState_TypeDef *state)
{
	double e, fe, fe1;
	double z1, z2, z3;

	z1 = state->z1;
	z2 = state->z2;
	z3 = state->z3;

	e = state->z1 - y;
	if (abs(e) > para->d) {
		fe = pow(abs(e), para->a1) * sign(e);
		fe1 = pow(abs(e), para->a2) * sign(e);
	}
	else {
		fe = e / pow(para->d, (1 - para->a1));
		fe1 = e / pow(para->d, (1 - para->a2));
	}

	state->z1 = z1 + (z2 - para->b1 * e) * para->h;
	state->z2 = z2 + (z3 - para->b2 * fe + para->b * u) * para->h;
	state->z3 = z3 - para->b3 * fe1 * para->h;
}

//----------------------NLSEF--------------------
void NLSEF_Atti(TDState_TypeDef *tdstate,ESOState_TypeDef *esostate,NLSEFState_TypeDef *nlsefstate)
{

	float e1, e2, u1, u2;
	e1 = tdstate->x1 - esostate->z1;
	e2 = tdstate->x2 - esostate->z2;
	if (abs(e1) > nlsefstate->d)
		u1 = pow(abs(e1), nlsefstate->a1) * sign(e1);
	else
		u1 = e1 / pow(nlsefstate->d, 1 - nlsefstate->a1);

	if (abs(e2) > nlsefstate->d)
		u2 = pow(abs(e2), nlsefstate->a2) * sign(e2);
	else
		u2 = e2 / pow(nlsefstate->d, 1 - nlsefstate->a2);
	nlsefstate->u = nlsefstate->b1 * u1 + nlsefstate->b2 * u2 - esostate->z3 / nlsefstate->b;
}

double AttiRateADRC_Ctrl(matrix::Vector3f rate_v, matrix::Vector3f _rates_sp_v, const bool landed)
{
	hrt_abstime now1 = hrt_absolute_time();
	float phi_rate_ref = 0;

	float phi_rate = rate_v(0);
	// theta_rate = rate_v(1);
	// psi_rate = rate_v(2) * 3.14 / 180;

	if (!landed) {
		phi_rate_ref = _rates_sp_v(0);
		// theta_rate_ref = _rates_sp_v(1) * 3.14 / 180;
		// psi_rate_ref = _rates_sp_v(2) * 3.14 / 180;
	}
	/* Roll Channel */
	TD_Atti(&TDState_RollRadio,phi_rate_ref,&TD_fhanParas_RollRadio);
	ESO_Atti(phi_rate,NLSEFState_Roll.u,&ESOParas_Roll,&ESOState_Roll);
	NLSEF_Atti(&TDState_RollRadio,&ESOState_Roll,&NLSEFState_Roll);

	ofile <<now1<<","<<phi_rate<<","<<phi_rate_ref<<","<<NLSEFState_Roll.u<<endl;
	return (double)NLSEFState_Roll.u / 500;

	/* Pitch Channel*/
	// TD_Atti(&TDState_PitchRadio,theta_rate_ref,&TD_fhanParas_PitchRadio);
	// ESO_Atti(theta_rate,NLSEFState_Pitch.u,&ESOParas_Pitch,&ESOState_Pitch);
	// NLSEF_Atti(&TDState_PitchRadio,&ESOState_Pitch,&NLSEFState_Pitch);

	/* Yaw Channel */
	// TD_Atti(&TDState_YawRadio,psi_rate_ref,&TD_fhanParas_YawRadio);
	// ESO_Atti(psi_rate,NLSEFState_Yaw.u,&ESOParas_Yaw,&ESOState_Yaw);
	// NLSEF_Atti(&TDState_YawRadio,&ESOState_Yaw,&NLSEFState_Yaw);

	// NLSEFState_Roll.u = AMP_Limit(NLSEFState_Roll.u,-200,200);	//us
	// NLSEFState_Pitch.u = AMP_Limit(NLSEFState_Pitch.u,-150,150);	//us
	// NLSEFState_Yaw.u = AMP_Limit(NLSEFState_Yaw.u,-150,150);	//us

}

void file_init(){
	char s[60];
	struct tm tim;
	time_t now = time(NULL);
	tim = *(localtime(&now));
	// strftime(s,60,"control_quantity_%b_%d_%H_%M.csv",&tim);
	strftime(s,60,"adrc_roll_%b_%d_%H_%M.csv",&tim);
	string path =  string(s);

	ofile.open(path.c_str(),ios::out | ios::app);

	//ofile.open("control_actuator.csv", ios::out | ios::app);	//default path is ~/.ros
	if(ofile)
	{
		warnx("file init!");
		ofile <<"time(us)"<<","<<"phi_rate"<<","<<"phi_rate_ref"<<","<<"adrc.u"<<endl;
			// <<"roll_pid"<<","<<"pitch_pid"<<","<<"yaw_pid" <<","
			// <<"roll_rate_control"<<","<<"pitch_rate_control"<<","<<"yaw_rate_control" <<","
			// <<"roll_rate_ref"<<","<<"TD_x1"<<","<<"TD_x2"<<","
			// <<"roll_rate"<<","<<"ESO_Z1"<<","<<"ESO_Z2"<<","<<"ESO_Z3"<<"\n";
	}
	vehicle_att_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	vehicle_local_pos_fd = orb_subscribe(ORB_ID(vehicle_local_position));

}
/*
void get_vehicle_status(){
	memset(&att_q, 0 , sizeof(att_q));
	memset(&local_pos, 0 , sizeof(local_pos));
	orb_copy(ORB_ID(vehicle_attitude), vehicle_att_fd, &att_q);
	orb_copy(ORB_ID(vehicle_local_position),vehicle_local_pos_fd, &local_pos);
}
*/
