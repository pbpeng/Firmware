/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "IEKF.hpp"
#include "matrix/filter.hpp"

static const float mag_inclination = 1.0f;
static const float mag_declination = 0;

IEKF::IEKF() :
	_nh(), // node handle
	// sensors
	_sensorAccel(),
	_sensorMag(),
	_sensorBaro(),
	_sensorGps(),
	_sensorAirspeed(),
	_sensorFlow(),
	_sensorSonar(),
	_sensorLidar(),
	_sensorVision(),
	_sensorMocap(),
	// subscriptions
	_subImu(_nh.subscribe("sensor_combined", 0, &IEKF::callbackImu, this)),
	_subGps(_nh.subscribe("vehicle_gps_position", 0, &IEKF::correctGps, this)),
	_subAirspeed(_nh.subscribe("airspeed", 0, &IEKF::correctAirspeed, this)),
	_subFlow(_nh.subscribe("optical_flow", 0, &IEKF::correctFlow, this)),
	_subDistance(_nh.subscribe("distance_sensor", 0, &IEKF::callbackDistance, this)),
	_subVision(_nh.subscribe("vision_position_estimate", 0, &IEKF::correctVision, this)),
	_subMocap(_nh.subscribe("att_pos_mocap", 0, &IEKF::correctMocap, this)),
	// publications
	_pubAttitude(_nh.advertise<vehicle_attitude_s>("vehicle_attitude", 0)),
	_pubLocalPosition(_nh.advertise<vehicle_local_position_s>("vehicle_local_position", 0)),
	_pubGlobalPosition(_nh.advertise<vehicle_global_position_s>("vehicle_global_position", 0)),
	_pubControlState(_nh.advertise<control_state_s>("control_state", 0)),
	_pubEstimatorStatus(_nh.advertise<estimator_status_s>("estimator_status", 0)),
	// data
	_x0(),
	_xMin(),
	_xMax(),
	_P0Diag(),
	_x(),
	_P(),
	_u(),
	_g_n(0, 0, -9.8),
	_B_n(),
	_origin(),
	_baroAsl(0),
	_gpsUSec(0)
{
	// for quaterinons we bound at 2
	// so it has a chance to
	// do a renormalization first
	_xMin(X::q_nb_0) = -2;
	_xMin(X::q_nb_1) = -2;
	_xMin(X::q_nb_2) = -2;
	_xMin(X::q_nb_3) = -2;
	_xMin(X::vel_N) = -100;
	_xMin(X::vel_E) = -100;
	_xMin(X::vel_D) = -100;
	_xMin(X::gyro_bias_bX) = -1.0;
	_xMin(X::gyro_bias_bY) = -1.0;
	_xMin(X::gyro_bias_bZ) = -1.0;
	_xMin(X::accel_scale) = 0.8;
	_xMin(X::pos_N) = -1e9;
	_xMin(X::pos_E) = -1e9;
	_xMin(X::asl) = -1e9;
	_xMin(X::terrain_asl) = -1e6;
	_xMin(X::baro_bias) = -1e6;
	_xMin(X::wind_N) = -100;
	_xMin(X::wind_E) = -100;
	_xMin(X::wind_D) = -100;

	_xMax(X::q_nb_0) = 2;
	_xMax(X::q_nb_1) = 2;
	_xMax(X::q_nb_2) = 2;
	_xMax(X::q_nb_3) = 2;
	_xMax(X::vel_N) = 100;
	_xMax(X::vel_E) = 100;
	_xMax(X::vel_D) = 100;
	_xMax(X::gyro_bias_bX) = 1.0;
	_xMax(X::gyro_bias_bY) = 1.0;
	_xMax(X::gyro_bias_bZ) = 1.0;
	_xMax(X::accel_scale) = 1.5;
	_xMax(X::pos_N) = 1e9;
	_xMax(X::pos_E) = 1e9;
	_xMax(X::asl) = 1e9;
	_xMax(X::terrain_asl) = 1e6;
	_xMax(X::baro_bias) = 1e6;
	_xMax(X::wind_N) = 100;
	_xMax(X::wind_E) = 100;
	_xMax(X::wind_D) = 100;

	// initialize state
	_x0(X::q_nb_0) = 1;
	_x0(X::q_nb_1) = 0;
	_x0(X::q_nb_2) = 0;
	_x0(X::q_nb_3) = 0;
	_x0(X::accel_scale) = 1;
	setX(_x0);

	// initialize covariance
	_P0Diag(Xe::rot_N) = 1;
	_P0Diag(Xe::rot_E) = 1;
	_P0Diag(Xe::rot_D) = 3;
	_P0Diag(Xe::vel_N) = 1e6;
	_P0Diag(Xe::vel_E) = 1e6;
	_P0Diag(Xe::vel_D) = 1e6;
	_P0Diag(Xe::gyro_bias_N) = 1e-5;
	_P0Diag(Xe::gyro_bias_E) = 1e-5;
	_P0Diag(Xe::gyro_bias_D) = 1e-5;
	_P0Diag(Xe::accel_scale) = 1e-5;
	_P0Diag(Xe::pos_N) = 1e6;
	_P0Diag(Xe::pos_E) = 1e6;
	_P0Diag(Xe::asl) = 1e6;
	_P0Diag(Xe::terrain_asl) = 1e6;
	_P0Diag(Xe::baro_bias) = 1e6;
	_P0Diag(Xe::wind_N) = 1e-2;
	_P0Diag(Xe::wind_E) = 1e-2;
	_P0Diag(Xe::wind_D) = 1e-2;
	setP(diag(_P0Diag));

	// initial magnetic field guess
	_B_n = Vector3f(0.21523, 0.00771, -0.42741);
}

Vector<float, X::n> IEKF::dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u)
{
	Quatf q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Vector3f as_n = a_n - _g_n;
	Vector3f gyro_bias_b(_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	Vector3f omega_nb_b(_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	Quatf dq_nb = q_nb * Quatf(0, omega_nb_b_corrected(0),
				   omega_nb_b_corrected(1), omega_nb_b_corrected(2)) * 0.5f;

	Vector<float, X::n> dx;
	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);
	dx(X::vel_N) = as_n(0);
	dx(X::vel_E) = as_n(1);
	dx(X::vel_D) = as_n(2);
	dx(X::gyro_bias_bX) = 0;
	dx(X::gyro_bias_bY) = 0;
	dx(X::gyro_bias_bZ) = 0;
	dx(X::accel_scale) = 0;
	dx(X::pos_N) = x(X::vel_N);
	dx(X::pos_E) = x(X::vel_E);
	dx(X::asl) = -x(X::vel_D);
	dx(X::terrain_asl) = 0;
	dx(X::baro_bias) = 0;
	dx(X::wind_N) = 0;
	dx(X::wind_E) = 0;
	dx(X::wind_D) = 0;
	return dx;
}

void IEKF::callbackImu(const sensor_combined_s *msg)
{
	//ROS_INFO("imu callback");
	_u(U::omega_nb_bX) = msg->gyro_rad[0];
	_u(U::omega_nb_bY) = msg->gyro_rad[1];
	_u(U::omega_nb_bZ) = msg->gyro_rad[2];
	_u(U::accel_bX) = msg->accelerometer_m_s2[0];
	_u(U::accel_bY) = msg->accelerometer_m_s2[1];
	_u(U::accel_bZ) = msg->accelerometer_m_s2[2];

	// predict driven by gyro callback
	if (msg->gyro_integral_dt > 0) {
		predict(msg->gyro_integral_dt);
	};

	// correct  if new data
	correctAccel(msg);

	correctMag(msg);

	correctBaro(msg);
}

void IEKF::correctAccel(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->accelerometer_timestamp_relative;

	if (!_sensorAccel.ready(timestamp, dt)) {
		return;
	}

	// measurement
	Vector3f y_b(
		msg->accelerometer_m_s2[0],
		msg->accelerometer_m_s2[1],
		msg->accelerometer_m_s2[2]);

	// don't correct if accelerating
	float relNormError = (Vector3f(y_b / _x(X::accel_scale)).norm()
			      - _g_n.norm()) / _g_n.norm();

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_g_n = q_nb.conjugate(y_b / _x(X::accel_scale));
	Vector3f r = y_g_n - _g_n;

	// define R
	// worst accel dir change is if accel is normal to gravity,
	// assume this and calculate angle covariance based on accel norm error
	Matrix<float, Y_accel::n, Y_accel::n> R;
	float cov = (1e-1f + relNormError * relNormError) / dt;
	R(Y_accel::accel_bX, Y_accel::accel_bX) = cov;
	R(Y_accel::accel_bY, Y_accel::accel_bY) = cov;
	R(Y_accel::accel_bZ, Y_accel::accel_bZ) = cov;

	// define H
	Matrix<float, Y_accel::n, Xe::n> H;
	Matrix3f tmp = _g_n.unit().hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_accel::accel_bX + i, Xe::rot_N + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_accel::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_accel::n];
	_sensorAccel.setBeta(beta);

	if (beta > 1.0f) {
		ROS_WARN("accel fault: beta %10.4f", double(beta));
	}

	Vector<float, X::n> x = applyErrorCorrection(dxe);
	Quatf q_nb2(x(X::q_nb_0), x(X::q_nb_1),
		    x(X::q_nb_2), x(X::q_nb_3));
	Vector3f r2 = q_nb2.conjugate(y_b / x(X::accel_scale)) - _g_n;

	if (r2.norm() - r.norm() > 1e-2f) {
		ROS_DEBUG("accel non-linear correction used");
		Vector3f rot(dxe(Xe::rot_N), dxe(Xe::rot_E), dxe(Xe::rot_D));
		float angle = rot.norm();
		float angle_max = 0.1f * acosf(y_g_n.dot(_g_n)) / y_g_n.norm() / _g_n.norm();

		if (angle > angle_max) {
			angle = angle_max;
		}

		Vector3f axis = y_g_n.cross(_g_n).unit();
		dxe.setZero();
		dxe(Xe::rot_N) = axis(0) * angle;
		dxe(Xe::rot_E) = axis(1) * angle;
		dxe(Xe::rot_D) = axis(2) * angle;
		x = applyErrorCorrection(dxe);
		setX(x);
		// don't update P, linearization is poor

	} else {
		setX(x);
		setP(_P + dP);
	}
}

void IEKF::correctMag(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (!_sensorMag.ready(timestamp, dt)) {
		return;
	}

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_b = Vector3f(
			       msg->magnetometer_ga[0],
			       msg->magnetometer_ga[1],
			       msg->magnetometer_ga[2]).unit();
	Vector3f yh = _B_n.unit();
	Vector3f y = q_nb.conjugate(y_b);
	Vector3f r = y - yh;

	// define R
	float covNE = 2e-3f / dt;
	// less certain about mag inclination
	float covD = 20e-3f / dt;
	Matrix<float, Y_mag::n, Y_mag::n> R;
	R(Y_mag::mag_N, Y_mag::mag_N) = covNE;
	R(Y_mag::mag_E, Y_mag::mag_E) = covNE;
	R(Y_mag::mag_D, Y_mag::mag_D) = covD;

	// define H
	Matrix<float, Y_mag::n, Xe::n> H;
	Matrix3f tmp = yh.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_mag::mag_N + i, Xe::rot_N + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_mag::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_mag::n];
	_sensorMag.setBeta(beta);

	if (beta > 1.0f) {
		ROS_WARN("mag fault: beta %10.4f", double(beta));
	}

	Vector<float, X::n> x = applyErrorCorrection(dxe);
	Vector3f r2 = Quatf(x(X::q_nb_0), x(X::q_nb_1),
			    x(X::q_nb_2), x(X::q_nb_3)).conjugate(y_b) - yh;

	if (r2.norm() - r.norm() > 1e-2f) {
		ROS_DEBUG("mag non-linear correction used");
		Vector3f rot(dxe(Xe::rot_N), dxe(Xe::rot_E), dxe(Xe::rot_D));
		Vector3f y_xy = Vector3f(y(0), y(1), 0);
		Vector3f yh_xy = Vector3f(yh(0), yh(1), 0);
		float angle = rot.norm();
		float angle_max = 0.1f * acosf(y_xy.dot(yh_xy)) / y_xy.norm() / yh_xy.norm();

		if (angle > angle_max) {
			angle = angle_max;
		}

		Vector3f axis = y_xy.cross(yh_xy).unit();
		dxe.setZero();
		dxe(Xe::rot_N) = axis(0) * angle;
		dxe(Xe::rot_E) = axis(1) * angle;
		dxe(Xe::rot_D) = axis(2) * angle;
		x = applyErrorCorrection(dxe);
		setX(x);
		// don't update P, linearization is poor

	} else {
		setX(x);
		setP(_P + dP);
	}
}

void IEKF::correctBaro(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestamp = msg->timestamp + msg->baro_timestamp_relative;

	if (!_sensorBaro.ready(timestamp, dt)) {
		return;
	}

	// calculate residual
	Vector<float, Y_baro::n> y;
	y(Y_baro::asl) = msg->baro_alt_meter;
	Vector<float, Y_baro::n> yh;
	yh(Y_baro::asl)	= _x(X::asl) + _x(X::baro_bias);
	Vector<float, Y_baro::n> r = y - yh;

	// save pressure altitude info
	_baroAsl = y(0);

	// define R
	Matrix<float, Y_baro::n, Y_baro::n> R;
	R(Y_baro::asl, Y_baro::asl) = 2e-3f / dt;

	// define H
	Matrix<float, Y_baro::n, Xe::n> H;
	H(Y_baro::asl, Xe::asl) = 1;
	H(Y_baro::asl, Xe::baro_bias) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_baro::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_baro::n];
	_sensorBaro.setBeta(beta);

	if (beta > 1.0f) {
		ROS_DEBUG("baro ratio: %10.4f", double(beta));
	}

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}

void IEKF::correctGps(const vehicle_gps_position_s *msg)
{
	// return if no new data
	float dt = 0;

	if (!_sensorGps.ready(msg->timestamp, dt)) {
		return;
	}

	// check for good gps signal
	if (msg->satellites_used < 6 || msg->fix_type < 3) {
		return;
	}

	double lat_deg = msg->lat * 1e-7;
	double lon_deg = msg->lon * 1e-7;
	float alt_m = msg->alt * 1e-3;

	// init global reference
	if (!_origin.xyInitialized()) {
		ROS_INFO("gps origin init lat: %12.6f deg lon: %12.6f deg",
			 double(lat_deg), double(lon_deg));
		_origin.xyInitialize(lat_deg, lon_deg, msg->timestamp);
	}

	// init origin alt
	if (!_origin.altInitialized()) {
		ROS_INFO("gps origin init alt %12.2f m", double(alt_m));
		_origin.altInitialize(alt_m, msg->timestamp);
	}

	// calculate residual
	float gps_pos_N = 0;
	float gps_pos_E = 0;
	_origin.globalToLocalXY(lat_deg, lon_deg, gps_pos_N, gps_pos_E);

	Vector<float, Y_gps::n> y;
	y(Y_gps::pos_N) = gps_pos_N;
	y(Y_gps::pos_E) = gps_pos_E;
	y(Y_gps::asl) = alt_m;
	y(Y_gps::vel_N) = msg->vel_n_m_s;
	y(Y_gps::vel_E) = msg->vel_e_m_s;
	y(Y_gps::vel_D) = msg->vel_d_m_s;

	Vector<float, Y_gps::n> yh;
	yh(Y_gps::pos_N) = _x(X::pos_N);
	yh(Y_gps::pos_E) = _x(X::pos_E);
	yh(Y_gps::asl) = _x(X::asl);
	yh(Y_gps::vel_N) = _x(X::vel_N);
	yh(Y_gps::vel_E) = _x(X::vel_E);
	yh(Y_gps::vel_D) = _x(X::vel_D);

	Vector<float, Y_gps::n> r = y - yh;

	// define R
	Matrix<float, Y_gps::n, Y_gps::n> R;
	R(Y_gps::pos_N, Y_gps::pos_N) = 0.2f / dt;
	R(Y_gps::pos_E, Y_gps::pos_E) = 0.2f / dt;
	R(Y_gps::asl, Y_gps::asl) = 0.2f / dt;
	R(Y_gps::vel_N, Y_gps::vel_N) = 0.2f / dt;
	R(Y_gps::vel_E, Y_gps::vel_E) = 0.2f / dt;
	R(Y_gps::vel_D, Y_gps::vel_D) = 0.2f / dt;

	// define H
	Matrix<float, Y_gps::n, Xe::n> H;
	H(Y_gps::pos_N, Xe::pos_N) = 1;
	H(Y_gps::pos_E, Xe::pos_E) = 1;
	H(Y_gps::asl, Xe::asl) = 1;
	H(Y_gps::vel_N, Xe::vel_N) = 1;
	H(Y_gps::vel_E, Xe::vel_E) = 1;
	H(Y_gps::vel_D, Xe::vel_D) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_gps::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_gps::n];
	_sensorGps.setBeta(beta);

	if (beta > 1.0f) {
		ROS_WARN("gps fault: beta %10.4f", double(beta));
	}

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}

void IEKF::correctAirspeed(const airspeed_s *msg)
{
	// return if no new data
	float dt = 0;

	if (!_sensorAirspeed.ready(msg->timestamp, dt)) {
		return;
	}

	// attitude info
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Dcmf C_nb = q_nb;

	// predicted airspeed
	Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float yh = -wind_rel_b(0); // body -x component aligned with pitot tube
	float vel_N = _x(X::vel_N);
	float vel_E = _x(X::vel_E);
	float vel_D = _x(X::vel_D);
	float wind_N = _x(X::wind_N);
	float wind_E = _x(X::wind_E);
	float wind_D = _x(X::wind_D);

	// abort if too large of an angle on pitot tube
	float wind_angle = acosf(-wind_rel_b(0)) / wind_rel_b.norm();
	static const float wind_angle_max = 30.0 * deg2rad;

	if (wind_angle > wind_angle_max) {
		if (wind_rel_b.norm() > 5.0f) {
			// if airspeed magnitude large warn the user, this could be a problem
			ROS_WARN("relative wind angle too large for pitot tube correction");
		}

		return;
	}

	// measured airspeed
	float y = msg->true_airspeed_unfiltered_m_s;

	Vector<float, 1> r;
	r(0) = y - yh;

	// define R
	Matrix<float, Y_airspeed::n, Y_airspeed::n> R;
	R(Y_airspeed::airspeed, Y_airspeed::airspeed) = 100.0f / dt;

	// define H
	// Note: this measurement is not invariant due to
	// rotation matrix
	Matrix<float, Y_airspeed::n, Xe::n> H;
	float x0 = 2 * C_nb(1, 0);
	float x1 = vel_D - wind_D;
	float x2 = 2 * C_nb(2, 0);
	float x3 = vel_E - wind_E;
	float x4 = 2 * C_nb(0, 0);
	float x5 = vel_N - wind_N;
	H(Y_airspeed::airspeed, Xe::rot_N) = x0 * x1 - x2 * x3;
	H(Y_airspeed::airspeed, Xe::rot_E) = -x1 * x4 + x2 * x5;
	H(Y_airspeed::airspeed, Xe::rot_D) = -x0 * x5 + x3 * x4;
	H(Y_airspeed::airspeed, Xe::vel_N) = C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::vel_E) = C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::vel_D) = C_nb(2, 0);
	H(Y_airspeed::airspeed, Xe::wind_N) = -C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::wind_E) = -C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::wind_D) = -C_nb(2, 0);

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_airspeed::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_airspeed::n];
	_sensorAirspeed.setBeta(beta);

	if (beta > 1.0f) {
		ROS_WARN("airspeed fault: beta %10.4f", double(beta));
	}

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);

}

void IEKF::correctFlow(const optical_flow_s *msg)
{
	// return if no new data
	float dt = 0;

	if (!_sensorFlow.ready(msg->timestamp, dt)) {
		return;
	}

	// state info
	Dcmf C_nb = Quaternion<float>(
			    _x(X::q_nb_0), _x(X::q_nb_1),
			    _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f omega_nb_b(
		_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	float agl = _x(X::asl) - _x(X::terrain_asl);
	float vel_N = _x(X::vel_N);
	float vel_E = _x(X::vel_E);

	// expected measurement
	Vector<float, Y_flow::n> yh;
	yh(0) = omega_nb_b(0) + agl * (C_nb(0, 0) * vel_N + C_nb(0, 1) * vel_E) / C_nb(2, 2);
	yh(1) = omega_nb_b(1) + agl * (C_nb(1, 0) * vel_N + C_nb(1, 1) * vel_E) / C_nb(2, 2);

	// measurement
	Vector<float, Y_flow::n> y;
	float float_dt = msg->integration_timespan / 1.0e6f;
	y(0) = msg->pixel_flow_x_integral / float_dt;
	y(1) = msg->pixel_flow_y_integral / float_dt;

	// residual
	Vector<float, Y_flow::n> r = y - yh;

	// define R
	Matrix<float, Y_flow::n, Y_flow::n> R;
	R(Y_flow::flowX, Y_flow::flowX) = 10.0f / dt;
	R(Y_flow::flowY, Y_flow::flowY) = 10.0f / dt;

	// define H
	// Note: this measurement is not invariant due to
	// rotation matrix
	Matrix<float, Y_flow::n, Xe::n> H;
	float x0 = 2 * agl / C_nb(2, 2) / C_nb(2, 2);
	float x1 = C_nb(2, 1) * C_nb(2, 2);
	float x2 = C_nb(0, 1) * vel_N + C_nb(1, 1) * vel_E;
	float x3 = 1 / C_nb(2, 2);
	float x4 = 2 * agl * x3;
	float x5 = agl * x3;
	float x6 = x2 * x3;
	float x7 = C_nb(2, 0) * C_nb(2, 2);
	float x8 = C_nb(0, 0) * vel_N + C_nb(1, 0) * vel_E;
	float x9 = x3 * x8;
	H(Y_flow::flowX, Xe::rot_N) = -x0 * (C_nb(1, 2) * x2 + vel_E * x1);
	H(Y_flow::flowX, Xe::rot_E) = x0 * (C_nb(0, 2) * x2 + vel_N * x1);
	H(Y_flow::flowX, Xe::rot_D) = x4 * (C_nb(0, 1) * vel_E - C_nb(1, 1) * vel_N);
	H(Y_flow::flowX, Xe::vel_N) = C_nb(0, 1) * x5;
	H(Y_flow::flowX, Xe::vel_E) = C_nb(1, 1) * x5;
	H(Y_flow::flowX, Xe::gyro_bias_N) = -C_nb(0, 0);
	H(Y_flow::flowX, Xe::gyro_bias_E) = -C_nb(1, 0);
	H(Y_flow::flowX, Xe::gyro_bias_D) = -C_nb(2, 0);
	H(Y_flow::flowX, Xe::asl) = x6;
	H(Y_flow::flowX, Xe::terrain_asl) = -x6;
	H(Y_flow::flowY, Xe::rot_N) = -x0 * (C_nb(1, 2) * x8 + vel_E * x7);
	H(Y_flow::flowY, Xe::rot_E) = x0 * (C_nb(0, 2) * x8 + vel_N * x7);
	H(Y_flow::flowY, Xe::rot_D) = x4 * (C_nb(0, 0) * vel_E - C_nb(1, 0) * vel_N);
	H(Y_flow::flowY, Xe::vel_N) = C_nb(0, 0) * x5;
	H(Y_flow::flowY, Xe::vel_E) = C_nb(1, 0) * x5;
	H(Y_flow::flowY, Xe::gyro_bias_N) = -C_nb(0, 1);
	H(Y_flow::flowY, Xe::gyro_bias_E) = -C_nb(1, 1);
	H(Y_flow::flowY, Xe::gyro_bias_D) = -C_nb(2, 1);
	H(Y_flow::flowY, Xe::asl) = x9;
	H(Y_flow::flowY, Xe::terrain_asl) = -x9;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_flow::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_flow::n];
	_sensorFlow.setBeta(beta);

	if (beta > 1.0f) {
		ROS_WARN("flow fault: beta %10.4f", double(beta));
	}

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);


}

void IEKF::callbackDistance(const distance_sensor_s *msg)
{
	// if not pointing down (roll 180 by convention), do not use
	if (msg->orientation != 8) {
		ROS_INFO("distance sensor wrong orientation %d", msg->orientation);
		return;
	}

	// if above max distance don't correct, out of range
	if (msg->current_distance > msg->max_distance) {
		return;
	}

	// if below 0, don't correct and warn
	if (msg->current_distance < 0) {
		ROS_WARN("distance below 0");
		return;
	}

	// call correct correction function based on type
	if (msg->type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) {
		correctSonar(msg);

	} else if (msg->type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) {
		correctLidar(msg);
	}
}

void IEKF::correctSonar(const distance_sensor_s *msg)
{
	//ROS_INFO("correct sonar");
	// return if no new data
	float dt = 0;

	if (!_sensorSonar.ready(msg->timestamp, dt)) {
		return;
	}
}

void IEKF::correctLidar(const distance_sensor_s *msg)
{
	//ROS_INFO("correct lidar");
	// return if no new data
	float dt = 0;

	if (!_sensorLidar.ready(msg->timestamp, dt)) {
		return;
	}

	// attitude info
	Dcmf C_nb = Quaternion<float>(
			    _x(X::q_nb_0), _x(X::q_nb_1),
			    _x(X::q_nb_2), _x(X::q_nb_3));

	// abort if too large of an angle
	if (C_nb(2, 2) < 1e-1f) {
		ROS_INFO("lidar correction aborted, too large of an angle");
		return;
	}

	// expected measurement
	float agl = _x(X::asl) - _x(X::terrain_asl);
	float yh = agl / C_nb(2, 2);

	// measured airspeed
	float y = msg->current_distance;

	Vector<float, 1> r;
	r(0) = y - yh;

	// define R
	Matrix<float, Y_distance_down::n, Y_distance_down::n> R;
	R(Y_distance_down::d, Y_distance_down::d) = 1e-2f / dt;

	// define H
	// Note: this measurement is not invariant due to
	// rotation matrix
	Matrix<float, Y_distance_down::n, Xe::n> H;
	float x0 = 2 * agl / C_nb(2, 2) / C_nb(2, 2);
	float x1 = 1 / C_nb(2, 2);
	H(Y_distance_down::d, Xe::rot_N) = -C_nb(1, 2) * x0;
	H(Y_distance_down::d, Xe::rot_E) = C_nb(0, 2) * x0;
	H(Y_distance_down::d, Xe::asl) = x1;
	H(Y_distance_down::d, Xe::terrain_asl) = -x1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_distance_down::n>(_P, H, R, r, dxe, dP, beta);
	beta /= BETA_TABLE[Y_distance_down::n];
	_sensorLidar.setBeta(beta);

	if (beta > 1.0f) {
		ROS_WARN("distance_down fault: beta %10.4f", double(beta));
	}

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}

void IEKF::correctVision(const vision_position_estimate_s *msg)
{
	//ROS_INFO("callback vision");
	// return if no new data
	float dt = 0;

	if (!_sensorVision.ready(msg->timestamp, dt)) {
		return;
	}
}

void IEKF::correctMocap(const att_pos_mocap_s *msg)
{
	//ROS_INFO("correct mocap");
	// return if no new data
	float dt = 0;

	if (!_sensorMocap.ready(msg->timestamp, dt)) {
		return;
	}
}

void IEKF::predict(float dt)
{
	// rotation rate
	Vector3f omega_nb_b(
		_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	Vector3f gyro_bias_b(
		_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;

	// define process noise matrix
	Matrix<float, Xe::n, Xe::n> Q;
	Q(Xe::rot_N, Xe::rot_N) = 1e-6;
	Q(Xe::rot_E, Xe::rot_E) = 1e-6;
	Q(Xe::rot_D, Xe::rot_D) = 1e-6;
	Q(Xe::vel_N, Xe::vel_N) = 1e-3f;
	Q(Xe::vel_E, Xe::vel_E) = 1e-3f;
	Q(Xe::vel_D, Xe::vel_D) = 1e-3f;
	Q(Xe::gyro_bias_N, Xe::gyro_bias_N) = 1e-10f;
	Q(Xe::gyro_bias_E, Xe::gyro_bias_E) = 1e-10f;
	Q(Xe::gyro_bias_D, Xe::gyro_bias_D) = 1e-10f;
	Q(Xe::accel_scale, Xe::accel_scale) = 1e-6f;
	Q(Xe::pos_N, Xe::pos_N) = 1e-3f;
	Q(Xe::pos_E, Xe::pos_E) = 1e-3f;
	Q(Xe::asl, Xe::asl) = 1e-3f;
	Q(Xe::terrain_asl, Xe::terrain_asl) = 1e-1f;
	Q(Xe::baro_bias, Xe::baro_bias) = 1e-1f;
	Q(Xe::wind_N, Xe::wind_N) = 1e-1f;
	Q(Xe::wind_E, Xe::wind_E) = 1e-1f;
	Q(Xe::wind_D, Xe::wind_D) = 1e-1f;

	// define A matrix
	Matrix<float, Xe::n, Xe::n> A;

	// derivative of rotation error is -0.5 * gyro bias
	A(Xe::rot_N, Xe::Xe::gyro_bias_N) = -0.5;
	A(Xe::rot_E, Xe::Xe::gyro_bias_E) = -0.5;
	A(Xe::rot_D, Xe::Xe::gyro_bias_D) = -0.5;

	// derivative of velocity
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));

	if (fabsf(q_nb.norm() - 1.0f) > 1e-3f) {
		ROS_DEBUG("normalizing quaternion, norm was %6.4f\n", double(q_nb.norm()));
		q_nb.normalize();
		_x(X::q_nb_0) = q_nb(0);
		_x(X::q_nb_1) = q_nb(1);
		_x(X::q_nb_2) = q_nb(2);
		_x(X::q_nb_3) = q_nb(3);
	}

	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f J_a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Matrix<float, 3, 3> a_tmp = -J_a_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::vel_N + i, Xe::rot_N + j) = a_tmp(i, j);
		}

		A(Xe::vel_N + i, Xe::accel_scale) = -J_a_n(i);
	}

	// derivative of gyro bias
	Vector3f J_omega_n = q_nb.conjugate(omega_nb_b_corrected);
	Matrix<float, 3, 3> g_tmp = J_omega_n.hat();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::gyro_bias_N + i, Xe::gyro_bias_N + j) = g_tmp(i, j);
		}
	}

	// derivative of position is velocity
	A(Xe::pos_N, Xe::vel_N) = 1;
	A(Xe::pos_E, Xe::vel_E) = 1;
	A(Xe::asl, Xe::vel_D) = -1;

	// derivative of terrain alt is zero

	// derivative of baro bias is zero

	//ROS_INFO("A:");
	//for (int i=0;i<Xe::n; i++) {
	//for (int j=0;j<Xe::n; j++) {
	//printf("%10.3f, ", double(A(i, j)));
	//}
	//printf("\n");
	//}

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = dt;
	Vector<float, X::n> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, X::n> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	//ROS_INFO("dx predict \n");
	//dx.print();
	setX(_x + dx);

	// propgate covariance using euler integration
	Matrix<float, Xe::n, Xe::n> dP = (A * _P + _P * A.T() + Q) * dt;
	setP(_P + dP);

	//ROS_INFO("P:");
	//_P.print();
}

Vector<float, X::n> IEKF::applyErrorCorrection(const Vector<float, Xe::n> &d_xe)
{
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1), _x(X::q_nb_2), _x(X::q_nb_3));
	Quatf d_q_nb = Quatf(0,
			     d_xe(Xe::rot_N), d_xe(Xe::rot_E), d_xe(Xe::rot_D)) * q_nb;
	//ROS_INFO("d_q_nb");
	//d_q_nb.print();
	Vector3f d_gyro_bias_b = q_nb.conjugate_inversed(
					 Vector3f(d_xe(Xe::gyro_bias_N),
							 d_xe(Xe::gyro_bias_E),
							 d_xe(Xe::gyro_bias_D)));

	// linear term correction is the same
	// as the error correction
	Vector<float, X::n> x = _x;
	x(X::q_nb_0) += d_q_nb(0);
	x(X::q_nb_1) += d_q_nb(1);
	x(X::q_nb_2) += d_q_nb(2);
	x(X::q_nb_3) += d_q_nb(3);
	x(X::vel_N) += d_xe(Xe::vel_N);
	x(X::vel_E) += d_xe(Xe::vel_E);
	x(X::vel_D) += d_xe(Xe::vel_D);
	x(X::gyro_bias_bX) += d_gyro_bias_b(0);
	x(X::gyro_bias_bY) += d_gyro_bias_b(1);
	x(X::gyro_bias_bZ) +=  d_gyro_bias_b(2);
	x(X::accel_scale) += _x(X::accel_scale) * d_xe(Xe::accel_scale);
	x(X::pos_N) += d_xe(Xe::pos_N);
	x(X::pos_E) += d_xe(Xe::pos_E);
	x(X::asl) += d_xe(Xe::asl);
	x(X::terrain_asl) += d_xe(Xe::terrain_asl);
	x(X::baro_bias) += d_xe(Xe::baro_bias);
	x(X::wind_N) += d_xe(Xe::wind_N);
	x(X::wind_E) += d_xe(Xe::wind_E);
	x(X::wind_D) += d_xe(Xe::wind_D);
	return x;
}

void IEKF::setP(const SquareMatrix<float, Xe::n> &P)
{
	_P = P;

	for (int i = 0; i < Xe::n; i++) {
		// only operate on upper triangle, then copy to lower

		// don't allow NaN or large numbers
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				ROS_WARN("P(%d, %d) NaN, resetting", i, j);

				if (i == j) {
					_P(i, j) = _P0Diag(i);

				} else {
					_P(i, j) = 0;
				}
			}

			if (_P(i, j) > 1e6f) {
				// upper bound
				_P(i, j) = 1e6f;
			}
		}

		// force non-negative diagonal
		if (_P(i, i) < 0) {
			ROS_WARN("P(%d, %d) < 0, setting to P0 val", i, i, double(0));
			_P(i, i) = _P0Diag(i);
		}

		// force symmetry, copy uppper triangle to lower
		for (int j = 0; j < i; j++) {
			_P(j, i) = _P(i, j);
		}
	}
}

void IEKF::setX(const Vector<float, X::n> &x)
{
	// set private state
	_x = x;

	for (int i = 0; i < X::n; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			ROS_WARN("x(%d) NaN, setting to %10.4f", i, double(_x0(i)));
			_x(i) = _x0(i);
		}

		if (_x(i) < _xMin(i)) {
			ROS_WARN("x(%d) < lower bound, saturating", i);
			_x(i) = _xMin(i);

		} else if (_x(i) > _xMax(i)) {
			ROS_WARN("x(%d) > upper bound, saturating", i);
			_x(i) = _xMax(i);
		}
	}
}

void IEKF::publish()
{
	//ROS_INFO("x:");
	//_x.print();

	//ROS_INFO("P:");
	//_P.diag().print();

	float eph = sqrt(_P(Xe::pos_N, Xe::pos_N) + _P(Xe::pos_E, Xe::pos_E));
	float epv = _P(Xe::asl, Xe::asl);
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Euler<float> euler_nb = q_nb;
	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	ros::Time now = ros::Time::now();

	// predicted airspeed
	Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float airspeed = -wind_rel_b(0); // body -x component aligned with pitot tube

	bool attitudeValid = sqrtf(_P(Xe::rot_N, Xe::rot_N)
				   + _P(Xe::rot_E, Xe::rot_E)
				   + _P(Xe::rot_D, Xe::rot_D)) < 0.1f;

	bool velocityValid = sqrtf(_P(Xe::vel_N, Xe::vel_N)
				   + _P(Xe::vel_E, Xe::vel_E)
				   + _P(Xe::vel_D, Xe::vel_D)) < 1.0f;

	bool positionValid = sqrtf(_P(Xe::pos_N, Xe::pos_N)
				   + _P(Xe::pos_E, Xe::pos_E)
				   + _P(Xe::asl, Xe::asl)) < 1.0f;

	// publish attitude
	if (attitudeValid) {
		vehicle_attitude_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.rollspeed = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitchspeed = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yawspeed = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		_pubAttitude.publish(msg);
	}

	// publish local position
	if (_origin.xyInitialized() && _origin.altInitialized() && positionValid && velocityValid) {
		vehicle_local_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.xy_valid = positionValid;
		msg.z_valid = positionValid;
		msg.v_xy_valid = velocityValid;
		msg.v_z_valid = velocityValid;
		msg.x = _x(X::pos_N);
		msg.y = _x(X::pos_E);
		msg.z = -(_x(X::asl) - _origin.getAlt());
		msg.delta_xy[0] = 0;
		msg.delta_xy[1] = 0;
		msg.delta_z = 0;
		msg.vx = _x(X::vel_N);
		msg.vy = _x(X::vel_E);
		msg.vz = _x(X::vel_D);
		msg.delta_vxy[0] = 0;
		msg.delta_vxy[1] = 0;
		msg.delta_vz = 0;
		msg.xy_reset_counter = 0;
		msg.z_reset_counter = 0;
		msg.vxy_reset_counter = 0;
		msg.vz_reset_counter = 0;
		msg.yaw = euler_nb(2);
		msg.xy_global = _origin.xyInitialized();
		msg.z_global = _origin.altInitialized();
		msg.ref_timestamp = _origin.getXYTimestamp();
		msg.ref_lat = _origin.getLatDeg();
		msg.ref_lon = _origin.getLonDeg();
		msg.ref_alt = _origin.getAlt();
		msg.dist_bottom = _x(X::asl) - _x(X::terrain_asl);
		msg.dist_bottom_rate = -_x(X::vel_D);
		msg.surface_bottom_timestamp = 0;
		msg.dist_bottom_valid = true;
		msg.eph = eph;
		msg.epv = epv;
		_pubLocalPosition.publish(msg);
	}

	// publish global position
	if (_origin.xyInitialized() && _origin.altInitialized() && positionValid && velocityValid) {
		double lat_deg = 0;
		double lon_deg = 0;
		_origin.northEastToLatLon(_x(X::pos_N), _x(X::pos_E), lat_deg, lon_deg);
		//ROS_INFO("alt %10.4f m", double(alt_m));
		vehicle_global_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.time_utc_usec = _gpsUSec;
		msg.lat = lat_deg;
		msg.lon = lon_deg;
		msg.alt = _x(X::asl);
		msg.delta_lat_lon[0] = 0;
		msg.delta_lat_lon[1] = 0;
		msg.delta_alt = 0;
		msg.lat_lon_reset_counter = 0;
		msg.alt_reset_counter = 0;
		msg.vel_n = _x(X::vel_N);
		msg.vel_e = _x(X::vel_E);
		msg.vel_d = _x(X::vel_D);
		msg.yaw = euler_nb(2);
		msg.eph = eph;
		msg.epv = epv;
		msg.terrain_alt = _x(X::terrain_asl);
		msg.terrain_alt_valid = true;
		msg.dead_reckoning = false;
		msg.pressure_alt = _baroAsl;
		_pubGlobalPosition.publish(msg);
	}

	// publish control state
	{
		// specific acceleration
		control_state_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.x_acc = a_n(0);
		msg.y_acc = a_n(1);
		msg.z_acc = a_n(2);
		msg.x_vel = _x(X::vel_N);
		msg.y_vel = _x(X::vel_E);
		msg.z_vel = _x(X::vel_D);
		msg.x_pos = _x(X::pos_N);
		msg.y_pos = _x(X::pos_E);
		// TODO, option for agl or asl
		msg.z_pos = -(_x(X::asl) - _x(X::terrain_asl));
		msg.airspeed = airspeed;
		msg.airspeed_valid = true;
		msg.vel_variance[0] = _P(Xe::vel_N, Xe::vel_N);
		msg.vel_variance[1] = _P(Xe::vel_E, Xe::vel_E);
		msg.vel_variance[2] = _P(Xe::vel_D, Xe::vel_D);
		msg.pos_variance[0] = _P(Xe::pos_N, Xe::pos_N);
		msg.pos_variance[1] = _P(Xe::pos_E, Xe::pos_E);
		msg.pos_variance[2] = _P(Xe::asl, Xe::asl);
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.delta_q_reset[0] = 0;
		msg.delta_q_reset[1] = 0;
		msg.delta_q_reset[2] = 0;
		msg.delta_q_reset[3] = 0;
		msg.quat_reset_counter = 0;
		msg.roll_rate = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitch_rate = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yaw_rate = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		msg.horz_acc_mag = 0;
		_pubControlState.publish(msg);
	}

	// estimator status
	{
		estimator_status_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.vibe[0] = 0; // TODO
		msg.vibe[1] = 0; // TODO
		msg.vibe[2] = 0; // TODO
		msg.n_states = X::n;

		for (int i = 0; i < X::n; i++) {
			msg.states[i] = _x(i);
		}

		for (int i = 0; i < Xe::n; i++) {
			msg.covariances[i] = _P(i, i);
		}

		// XXX
		// this isn't really general and is
		// tailored to EKF2 so just dumping
		// data in the best field names available
		msg.gps_check_fail_flags = 0; // TODO
		msg.control_mode_flags = 0; // TODO
		msg.filter_fault_flags = 0; // TODO
		msg.pos_horiz_accuracy = eph;
		msg.pos_vert_accuracy = epv;
		msg.innovation_check_flags = 0; // TODO
		msg.mag_test_ratio = _sensorMag.getBeta();
		msg.vel_test_ratio = 0; // TODO
		msg.pos_test_ratio = _sensorGps.getBeta();
		msg.hgt_test_ratio = _sensorAccel.getBeta();
		msg.tas_test_ratio = _sensorAirspeed.getBeta();
		msg.hagl_test_ratio = 0; // TODO
		msg.solution_status_flags = 0; // TODO
		_pubEstimatorStatus.publish(msg);
	}
}
