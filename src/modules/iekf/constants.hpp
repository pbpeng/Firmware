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

#pragma once

const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;

/**
 * Note that structs are used instead of enums
 * to allow use in arrays without casting
 * and to keep size small
 */

/**
 * State enum
 */
struct X {
	static const uint8_t q_nb_0 = 0;
	static const uint8_t q_nb_1 = 1;
	static const uint8_t q_nb_2 = 2;
	static const uint8_t q_nb_3 = 3;
	static const uint8_t vel_N = 4;
	static const uint8_t vel_E = 5;
	static const uint8_t vel_D = 6;
	static const uint8_t gyro_bias_bX = 7;
	static const uint8_t gyro_bias_bY = 8;
	static const uint8_t gyro_bias_bZ = 9;
	static const uint8_t accel_scale = 10;
	static const uint8_t pos_N = 11;
	static const uint8_t pos_E = 12;
	static const uint8_t asl = 13;
	static const uint8_t terrain_asl = 14;
	static const uint8_t baro_bias = 15;
	static const uint8_t wind_N = 16;
	static const uint8_t wind_E = 17;
	static const uint8_t wind_D = 18;
	static const uint8_t n = 19;
};

/**
 * Error state enum
 * used for linearization
 *
 * Note gyro bias in navigation frame
 */
struct Xe {
	static const uint8_t rot_N = 0;
	static const uint8_t rot_E = 1;
	static const uint8_t rot_D = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t gyro_bias_N = 6;
	static const uint8_t gyro_bias_E = 7;
	static const uint8_t gyro_bias_D = 8;
	static const uint8_t accel_scale = 9;
	static const uint8_t pos_N = 10;
	static const uint8_t pos_E = 11;
	static const uint8_t asl = 12;
	static const uint8_t terrain_asl = 13;
	static const uint8_t baro_bias = 14;
	static const uint8_t wind_N = 15;
	static const uint8_t wind_E = 16;
	static const uint8_t wind_D = 17;
	static const uint8_t n = 18;
};

/**
 * Input enum
 */
struct U {
	static const uint8_t omega_nb_bX = 0;
	static const uint8_t omega_nb_bY = 1;
	static const uint8_t omega_nb_bZ = 2;
	static const uint8_t accel_bX = 3;
	static const uint8_t accel_bY = 4;
	static const uint8_t accel_bZ = 5;
	static const uint8_t n = 6;
};

/**
 * Accel measurement enum
 */
struct Y_accel {
	static const uint8_t accel_bX = 0;
	static const uint8_t accel_bY = 1;
	static const uint8_t accel_bZ = 2;
	static const uint8_t n = 3;
};

/**
 * GPS measurement
 */
struct Y_gps {
	static const uint8_t pos_N = 0;
	static const uint8_t pos_E = 1;
	static const uint8_t asl = 2;
	static const uint8_t vel_N = 3;
	static const uint8_t vel_E = 4;
	static const uint8_t vel_D = 5;
	static const uint8_t n = 6;
};

/**
 * Baro measurement
 */
struct Y_baro {
	static const uint8_t asl = 0;
	static const uint8_t n = 1;
};

/**
 * Magnetometer measurement
 *
 * The filter treats the error
 * in the navigation frame
 * (north, east, down) even though the
 * field is measured in the body
 * frame.
 */
struct Y_mag {
	static const uint8_t mag_N = 0;
	static const uint8_t mag_E = 1;
	static const uint8_t mag_D = 2;
	static const uint8_t n = 3;
};

/**
 * Airspeed measurement
 */
struct Y_airspeed {
	static const uint8_t airspeed = 0;
	static const uint8_t n = 1;
};

/**
 * Optical flow measurement
 */
struct Y_flow {
	static const uint8_t flowX = 0;
	static const uint8_t flowY = 1;
	static const uint8_t n = 2;
};

/**
 * Distance down measurement
 */
struct Y_distance_down {
	static const uint8_t d = 0;
	static const uint8_t n = 1;
};

static const float BETA_TABLE[] = {
	0,
	8.82050518214,
	12.094592431,
	13.9876612368,
	16.0875642296,
	17.8797700658,
	19.6465647819,
	21.3802576894,
	23.0806434845,
	24.6673803845,
	26.1487953661,
	27.6350821245,
	29.6565383703,
	31.2211113844,
	32.7673547211,
	34.2967756977,
	35.6906782236,
	37.0724753352,
	38.4549693067,
	39.836592699,
};
