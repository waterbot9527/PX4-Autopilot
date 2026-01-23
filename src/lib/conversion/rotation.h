/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file rotation.h
 *
 * Vector rotation library
 */

#pragma once

#include <stdint.h>

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

/**
 * Enum for board and external compass rotations.
 * This enum maps from board attitude to airframe attitude.
 */
enum Rotation : uint8_t {
	ROTATION_NONE                = 0,
	ROTATION_YAW_45              = 1,
	ROTATION_YAW_90              = 2,
	ROTATION_YAW_135             = 3,
	ROTATION_YAW_180             = 4,
	ROTATION_YAW_225             = 5,
	ROTATION_YAW_270             = 6,
	ROTATION_YAW_315             = 7,
	ROTATION_ROLL_180            = 8,
	ROTATION_ROLL_180_YAW_45     = 9,
	ROTATION_ROLL_180_YAW_90     = 10,
	ROTATION_ROLL_180_YAW_135    = 11,
	ROTATION_PITCH_180           = 12,
	ROTATION_ROLL_180_YAW_225    = 13,
	ROTATION_ROLL_180_YAW_270    = 14,
	ROTATION_ROLL_180_YAW_315    = 15,
	ROTATION_ROLL_90             = 16,
	ROTATION_ROLL_90_YAW_45      = 17,
	ROTATION_ROLL_90_YAW_90      = 18,
	ROTATION_ROLL_90_YAW_135     = 19,
	ROTATION_ROLL_270            = 20,
	ROTATION_ROLL_270_YAW_45     = 21,
	ROTATION_ROLL_270_YAW_90     = 22,
	ROTATION_ROLL_270_YAW_135    = 23,
	ROTATION_PITCH_90            = 24,
	ROTATION_PITCH_270           = 25,
	ROTATION_PITCH_180_YAW_90    = 26,
	ROTATION_PITCH_180_YAW_270   = 27,
	ROTATION_ROLL_90_PITCH_90    = 28,
	ROTATION_ROLL_180_PITCH_90   = 29,
	ROTATION_ROLL_270_PITCH_90   = 30,
	ROTATION_ROLL_90_PITCH_180   = 31,
	ROTATION_ROLL_270_PITCH_180  = 32,
	ROTATION_ROLL_90_PITCH_270   = 33,
	ROTATION_ROLL_180_PITCH_270  = 34,
	ROTATION_ROLL_270_PITCH_270  = 35,
	ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
	ROTATION_ROLL_90_YAW_270          = 37,
	ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
	ROTATION_PITCH_315                = 39,
	ROTATION_ROLL_90_PITCH_315        = 40,
	//新加的旋转定义
	ROTATION_PITCH_45                 = 41,
	ROTATION_PITCH_315_YAW_90         = 42,
	ROTATION_PITCH_315_YAW_180        = 43,
	ROTATION_PITCH_315_YAW_270        = 44,
	ROTATION_PITCH_225                = 45,
	ROTATION_PITCH_225_YAW_90         = 46,
	ROTATION_PITCH_225_YAW_180        = 47,
	ROTATION_PITCH_225_YAW_270        = 48,
	ROTATION_PITCH_135                = 49,
	ROTATION_PITCH_135_YAW_90         = 50,
	ROTATION_PITCH_135_YAW_180        = 51,
	ROTATION_PITCH_135_YAW_270        = 52,
	ROTATION_PITCH_45_YAW_90          = 53,
	ROTATION_PITCH_45_YAW_180         = 54,
	ROTATION_PITCH_45_YAW_270         = 55,
	ROTATION_ROLL_180_PITCH_225       = 56,
	ROTATION_ROLL_180_PITCH_225_YAW_90  = 57,
	ROTATION_ROLL_180_PITCH_225_YAW_180 = 58,
	ROTATION_ROLL_180_PITCH_225_YAW_270 = 59,
	ROTATION_ROLL_180_PITCH_135       = 60,
	ROTATION_ROLL_180_PITCH_135_YAW_90  = 61,
	ROTATION_ROLL_180_PITCH_135_YAW_180 = 62,
	ROTATION_ROLL_180_PITCH_135_YAW_270 = 63,
	ROTATION_ROLL_356_PITCH_45       = 64,// Note: This should be ROTATION_ROLL_180_PITCH_45
	ROTATION_ROLL_180_PITCH_45_YAW_90  = 65,
	ROTATION_ROLL_180_PITCH_45_YAW_180 = 66,
	ROTATION_ROLL_180_PITCH_45_YAW_270 = 67,
	ROTATION_ROLL_90_PITCH_225        = 68,
	ROTATION_ROLL_90_PITCH_225_YAW_90  = 69,
	ROTATION_ROLL_90_PITCH_225_YAW_180 = 70,
	ROTATION_ROLL_90_PITCH_225_YAW_270 = 71,
	ROTATION_ROLL_90_PITCH_135        = 72,
	ROTATION_ROLL_90_PITCH_135_YAW_90  = 73,
	ROTATION_ROLL_90_PITCH_135_YAW_180 = 74,
	ROTATION_ROLL_90_PITCH_135_YAW_270 = 75,
	ROTATION_ROLL_90_PITCH_45         = 76,
	ROTATION_ROLL_90_PITCH_45_YAW_90  = 77,
	ROTATION_ROLL_90_PITCH_45_YAW_180 = 78,
	ROTATION_ROLL_90_PITCH_45_YAW_270 = 79,
	ROTATION_ROLL_270_PITCH_225       = 80,
	ROTATION_ROLL_270_PITCH_225_YAW_90  = 81,
	ROTATION_ROLL_270_PITCH_225_YAW_180 = 82,
	ROTATION_ROLL_270_PITCH_225_YAW_270 = 83,
	ROTATION_ROLL_270_PITCH_135       = 84,
	ROTATION_ROLL_270_PITCH_135_YAW_90  = 85,
	ROTATION_ROLL_270_PITCH_135_YAW_180 = 86,
	ROTATION_ROLL_270_PITCH_135_YAW_270 = 87,
	ROTATION_ROLL_270_PITCH_45        = 88,
	ROTATION_ROLL_270_PITCH_45_YAW_90  = 89,
	ROTATION_ROLL_270_PITCH_45_YAW_180 = 90,
	ROTATION_ROLL_270_PITCH_45_YAW_270 = 91,
	ROTATION_PITCH_270_YAW_90         = 92,
	ROTATION_PITCH_270_YAW_180        = 93,
	ROTATION_PITCH_270_YAW_270        = 94,
	ROTATION_PITCH_315_ROLL_270       = 95,
	ROTATION_MAX,  // This will be 96

	// Rotation Enum reserved for custom rotation using Euler Angles
	ROTATION_CUSTOM                  = 100
};

struct rot_lookup_t {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
};

static constexpr rot_lookup_t rot_lookup[ROTATION_MAX] = {
	{  0,   0,   0 },
	{  0,   0,  45 },
	{  0,   0,  90 },
	{  0,   0, 135 },
	{  0,   0, 180 },
	{  0,   0, 225 },
	{  0,   0, 270 },
	{  0,   0, 315 },
	{180,   0,   0 },
	{180,   0,  45 },
	{180,   0,  90 },
	{180,   0, 135 },
	{  0, 180,   0 },
	{180,   0, 225 },
	{180,   0, 270 },
	{180,   0, 315 },
	{ 90,   0,   0 },
	{ 90,   0,  45 },
	{ 90,   0,  90 },
	{ 90,   0, 135 },
	{270,   0,   0 },
	{270,   0,  45 },
	{270,   0,  90 },
	{270,   0, 135 },
	{  0,  90,   0 },
	{  0, 270,   0 },
	{  0, 180,  90 },
	{  0, 180, 270 },
	{ 90,  90,   0 },
	{180,  90,   0 },
	{270,  90,   0 },
	{ 90, 180,   0 },
	{270, 180,   0 },
	{ 90, 270,   0 },
	{180, 270,   0 },
	{270, 270,   0 },
	{ 90, 180,  90 },
	{ 90,   0, 270 },
	{ 90,  68, 293 },
	{  0, 315,   0 },
	{ 90, 315,   0 },
	{  0,  45,   0 },//新加的旋转定义
	{  0, 315,  90 },
	{  0, 315, 180 },
	{  0, 315, 270 },
	{  0, 225,   0 },
	{  0, 225,  90 },
	{  0, 225, 180 },
	{  0, 225, 270 },
	{  0, 135,   0 },
	{  0, 135,  90 },
	{  0, 135, 180 },
	{  0, 135, 270 },
	{  0,  45,  90 },
	{  0,  45, 180 },
	{  0,  45, 270 },
	{180, 225,   0 },
	{180, 225,  90 },
	{180, 225, 180 },
	{180, 225, 270 },
	{180, 135,   0 },
	{180, 135,  90 },
	{180, 135, 180 },
	{180, 135, 270 },
	{180,   0,   0 },//数值更改
	{180,  45,  90 },
	{180,  45, 180 },
	{180,  45, 270 },
	{ 90, 225,   0 },
	{ 90, 225,  90 },
	{ 90, 225, 180 },
	{ 90, 225, 270 },
	{ 90, 135,   0 },
	{ 90, 135,  90 },
	{ 90, 135, 180 },
	{ 90, 135, 270 },
	{ 90,  45,   0 },
	{ 90,  45,  90 },
	{ 90,  45, 180 },
	{ 90,  45, 270 },
	{270, 225,   0 },
	{270, 225,  90 },
	{270, 225, 180 },
	{270, 225, 270 },
	{270, 135,   0 },
	{270, 135,  90 },
	{270, 135, 180 },
	{270, 135, 270 },
	{270,  45,   0 },
	{270,  45,  90 },
	{270,  45, 180 },
	{270,  45, 270 },
	{  0, 270,  90 },
	{  0, 270, 180 },
	{  0, 270, 270 },
	{315, 270,   0 },
};

/**
 * Get the rotation matrix
 */
__EXPORT matrix::Dcmf get_rot_matrix(enum Rotation rot);

/**
 * Get the rotation quaternion
 */
__EXPORT matrix::Quatf get_rot_quaternion(enum Rotation rot);

/**
 * rotate a 3 element int16_t vector in-place
 */
__EXPORT void rotate_3i(enum Rotation rot, int16_t &x, int16_t &y, int16_t &z);

/**
 * rotate a 3 element float vector in-place
 */
__EXPORT void rotate_3f(enum Rotation rot, float &x, float &y, float &z);

template<typename T>
static bool rotate_3(enum Rotation rot, T &x, T &y, T &z)
{
	switch (rot) {
	case ROTATION_NONE:
		return true;

	case ROTATION_YAW_90: {
			T tmp = x;
			x = math::negate(y);
			y = tmp;
		}

		return true;

	case ROTATION_YAW_180: {
			x = math::negate(x);
			y = math::negate(y);
		}

		return true;

	case ROTATION_YAW_270: {
			T tmp = x;
			x = y;
			y = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_180: {
			y = math::negate(y);
			z = math::negate(z);
		}

		return true;

	case ROTATION_ROLL_180_YAW_90:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_270: {
			T tmp = x;
			x = y;
			y = tmp;
			z = math::negate(z);
		}

		return true;

	case ROTATION_PITCH_180: {
			x = math::negate(x);
			z = math::negate(z);
		}

		return true;

	case ROTATION_ROLL_180_YAW_270:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_90: {
			T tmp = x;
			x = math::negate(y);
			y = math::negate(tmp);
			z = math::negate(z);
		}

		return true;

	case ROTATION_ROLL_90: {
			T tmp = z;
			z = y;
			y = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_90_YAW_90: {
			T tmp = x;
			x = z;
			z = y;
			y = tmp;
		}

		return true;

	case ROTATION_ROLL_270: {
			T tmp = z;
			z = math::negate(y);
			y = tmp;
		}

		return true;

	case ROTATION_ROLL_270_YAW_90: {
			T tmp = x;
			x = math::negate(z);
			z = math::negate(y);
			y = tmp;
		}

		return true;

	case ROTATION_PITCH_90: {
			T tmp = z;
			z = math::negate(x);
			x = tmp;
		}

		return true;

	case ROTATION_PITCH_270: {
			T tmp = z;
			z = x;
			x = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_180_PITCH_270: {
			T tmp = z;
			z = x;
			x = tmp;
			y = math::negate(y);
		}

		return true;

	case ROTATION_ROLL_90_YAW_270: {
			T tmp = x;
			x = math::negate(z);
			z = y;
			y = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_90_PITCH_90: {
			T tmp = x;
			x = y;
			y = math::negate(z);
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_180_PITCH_90: {
			T tmp = x;
			x = math::negate(z);
			y = math::negate(y);
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_270_PITCH_90: {
			T tmp = x;
			x = math::negate(y);
			y = z;
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_90_PITCH_180: {
			T tmp = y;
			x = math::negate(x);
			y = math::negate(z);
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_270_PITCH_180: {
			T tmp = y;
			x = math::negate(x);
			y = z;
			z = tmp;
		}

		return true;

	case ROTATION_ROLL_90_PITCH_270: {
			T tmp = x;
			x = math::negate(y);
			y = math::negate(z);
			z = tmp;
		}

		return true;

	case ROTATION_ROLL_270_PITCH_270: {
			T tmp = x;
			x = y;
			y = z;
			z = tmp;
		}

		return true;

	case ROTATION_ROLL_90_PITCH_180_YAW_90: {
			T tmp = x;
			x = z;
			z = math::negate(y);
			y = math::negate(tmp);
		}

		return true;

	default:
		break;
	}

	return false;
}
