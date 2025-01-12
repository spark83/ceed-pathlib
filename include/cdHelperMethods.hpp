/*!
 * \file atlHelperMethods.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */

#ifndef __ATLHELPERMETHODS_H__
#define __ATLHELPERMETHODS_H__

#include <stdlib.h>
#include "cdTypes.h"

#define ATTILA_M_PI 3.14159265358979323846f

#define SAFE_DELETE(x) if(x != nullptr) { delete x; x = nullptr; }

#define SAFE_DELETE_ARRAY(x) if(x != nullptr) { delete [] x; x = nullptr; }

namespace attila
{
	template <class T>
	inline T Max(const T x, const T y)
	{
		return x >= y ? x : y;
	}

	template <class T>
	inline T Min(const T x, const T y)
	{
		return x <= y ? x : y;
	}

	template<class T>
	inline T Interpolate(f32 s, const T& x0, const T& x1)
	{
		auto result = (x1 - x0);
		result *= alpha;

		return x0 + result;
	}

	inline float RadToDeg(float rad)
	{
		return rad * 180.0f / ATTILA_M_PI;
	}

	inline float DegToRad(float deg)
	{
		return deg * ATTILA_M_PI / 180.0f;
	}

	inline float random01(void)
	{
#if (CC_TARGET_PLATFORM == CC_PLATFORM_WIN32)
		return (((float)std::rand()) / (float)(RAND_MAX));
#else
		return (((float)RandomGenerator::rand_mt()) / ((float)RAND_MAX));
#endif
	}

	inline double drandom01(void)
	{
		return (((double)std::rand()) / ((double)RAND_MAX));
	}

	inline float random2(float lowerBound, float upperBound)
	{
		return lowerBound + (random01() * (upperBound - lowerBound));
	}

	inline atlFloat fixed_random01(void)
	{
#if (CC_TARGET_PLATFORM == CC_PLATFORM_WIN32)
		return (((float)std::rand()) / (float)(RAND_MAX));
#else
		return (((float)RandomGenerator::rand_mt()) / (float)(RAND_MAX));
#endif
	}

	inline atlFloat fixed_random2(atlFloat lowerBound, atlFloat upperBound)
	{
		return lowerBound + (random01() * (upperBound - lowerBound));
	}

	inline float ScalarRandomWalk(const float initial,
		const float walkspeed,
		const float min,
		const float max)
	{
		const float next = initial + (((random01() * 2) - 1) * walkspeed);
		if (next < min) return min;
		if (next > max) return max;
		return next;
	}

}

#endif
