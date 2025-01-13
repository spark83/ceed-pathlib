/*!
 * \file cdHelperMethods.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */

#ifndef __ATLHELPERMETHODS_H__
#define __ATLHELPERMETHODS_H__

#include <stdlib.h>
#include "cdTypes.h"

constexpr float k_PI = 3.14159265358979323846f;

namespace ceed::ai::path {

	template<class T>
	inline T Interpolate(f32 s, const T& x0, const T& x1) {
		auto result = (x1 - x0);
		result *= s;

		return x0 + result;
	}

	inline float RadToDeg(float rad) {
		return rad * 180.0f / k_PI;
	}

	inline float DegToRad(float deg) {
		return deg * k_PI / 180.0f;
	}
}

#endif
