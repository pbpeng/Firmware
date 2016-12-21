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

#include "ros/ros.hpp"
#include "matrix/math.hpp"
#include <lib/geo/geo.h>

// local includes
#include "Origin.hpp"
#include "Sensor.hpp"
#include "constants.hpp"

// subscriptions
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>

// publications
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/estimator_status.h>

using namespace matrix;

/**
 * Main class for invariant extended kalman filter
 *
 * inspired by: https://hal.archives-ouvertes.fr/hal-00494342/document
 *
 * Also see python directory for simulation and python version.
 */
class IEKF
{
public:
	IEKF();
	Vector<float, X::n> dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u);
	bool ok() { return _nh.ok(); }
	void callbackImu(const sensor_combined_s *msg);
	void correctAccel(const sensor_combined_s *msg);
	void correctMag(const sensor_combined_s *msg);
	void correctBaro(const sensor_combined_s *msg);
	void correctGps(const vehicle_gps_position_s *msg);
	void correctAirspeed(const airspeed_s *msg);
	void correctFlow(const optical_flow_s *msg);
	void callbackDistance(const distance_sensor_s *msg);
	void correctSonar(const distance_sensor_s *msg);
	void correctLidar(const distance_sensor_s *msg);
	void correctVision(const vision_position_estimate_s *msg);
	void correctMocap(const att_pos_mocap_s *msg);
	void predict(float dt);
	Vector<float, X::n> applyErrorCorrection(const Vector<float, Xe::n> &d_xe);
	void setP(const SquareMatrix<float, Xe::n> &P);
	void setX(const Vector<float, X::n> &x);
	void publish();

private:
	ros::NodeHandle _nh;

	// sensors
	Sensor _sensorAccel;
	Sensor _sensorMag;
	Sensor _sensorBaro;
	Sensor _sensorGps;
	Sensor _sensorAirspeed;
	Sensor _sensorFlow;
	Sensor _sensorSonar;
	Sensor _sensorLidar;
	Sensor _sensorVision;
	Sensor _sensorMocap;

	// subscriptions
	ros::Subscriber _subImu;
	ros::Subscriber _subGps;
	ros::Subscriber _subAirspeed;
	ros::Subscriber _subFlow;
	ros::Subscriber _subDistance;
	ros::Subscriber _subVision;
	ros::Subscriber _subMocap;

	// publishers
	ros::Publisher _pubAttitude;
	ros::Publisher _pubLocalPosition;
	ros::Publisher _pubGlobalPosition;
	ros::Publisher _pubControlState;
	ros::Publisher _pubEstimatorStatus;

	// data
	Vector<float, X::n> _x0; 		// initial state vector
	Vector<float, X::n> _xMin; 		// lower bound vector
	Vector<float, X::n> _xMax; 		// upper bound vector
	Vector<float, Xe::n> _P0Diag; 	// initial state diagonal
	Vector<float, X::n> _x; 		// state vector
	SquareMatrix<float, Xe::n> _P; 	// covariance matrix
	Vector<float, U::n> _u; 		// input vector
	Vector3f _g_n; 					// expected gravity in navigation frame
	Vector3f _B_n; 					// expected magnetic field in navigation frame
	Origin _origin; 				// origin of local coordinate system
	float _baroAsl; 				// pressure altitude from baro
	uint64_t _gpsUSec; 				// gps internal timestamp
};
