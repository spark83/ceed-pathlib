/*!
 * \file cdHeuristics.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */

#ifndef _CDHEURISTICS_H_
#define _CDHEURISTICS_H_

#include "cdHelperMethods.h"

namespace ceed::ai::path {
	template <typename T>
	inline T ManhattanDistance(const T p_iSrcX, const T p_iSrcY, const T p_iDstX, const T p_iDstY) {
		return abs(p_iSrcX - p_iDstX) + abs(p_iSrcY - p_iDstY);
	}

	template <typename T>
	inline T DiagonalDistance(const T p_iSrcX, const T p_iSrcY, const T p_iDstX, const T p_iDstY) {
		return Max(abs(p_iSrcX - p_iDstX), abs(p_iSrcY - p_iDstY));
	}

	template <typename T>
	inline T EuclideanDistance(const T p_iSrcX, const T p_iSrcY, const T p_iDstX, const T p_iDstY) {
		return (T)sqrt((p_iSrcX - p_iDstX)*(p_iSrcX - p_iDstX) + (p_iSrcY - p_iDstY)*(p_iSrcY - p_iDstY));
	}

	template <typename T>
	inline T EuclideanDistanceSquared(const T p_iSrcX, const T p_iSrcY, const T p_iDstX, const T p_iDstY) {
		return (T)((p_iSrcX - p_iDstX)*(p_iSrcX - p_iDstX) + (p_iSrcY - p_iDstY)*(p_iSrcY - p_iDstY));
	}

	template <class T>
	inline T CrossProduct(const T p_iStartX, const T p_iStartY,
		const T p_iSrcX, const T p_iSrcY,
		const T p_iDstX, const T p_iDstY) {
		T Dx1 = p_iSrcX - p_iDstX;
		T Dy1 = p_iSrcY - p_iDstY;
		T Dx2 = p_iStartX - p_iDstX;
		T Dy2 = p_iStartY - p_iDstY;
		return abs(Dx1 * Dy2 - Dx2 * Dy1);
	}
}

#endif
