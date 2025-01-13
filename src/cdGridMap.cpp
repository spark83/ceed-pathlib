
/*!
 * \file cdGridMap.cpp
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#include <float.h>

#include "cdHeuristics.hpp"
#include "cdGridMap.hpp"

namespace {
constexpr f32 kPTMRatio = 32;
}

namespace ceed::ai::path {

//------------------------------------------------------------------------------------------------//

cdGridMap::cdGridMap(cdGridCellList& cells, int cols, int rows, cdPoint2f& dimension)
	: cdJumpStartMap(fastdelegate::MakeDelegate(this, &cdGridMap::CellCollides)
	, fastdelegate::MakeDelegate(this, &cdGridMap::GetHeuristics)
	, fastdelegate::MakeDelegate(this, &cdGridMap::GetMovementCost), 0, rows, cols)
	, m_ArraySize(cols * rows)
	, m_MapDimension(dimension)
	, m_MapHalfDimension(dimension)
	, m_Cells(cells) {
	m_MapHalfDimension /= 2;
	m_TileSize.x = m_MapDimension.x / m_NumCols;
	m_TileSize.y = m_MapDimension.y / m_NumRows;
	m_TileHalfSize = m_TileSize;
	m_TileHalfSize /= 2;
}

//------------------------------------------------------------------------------------------------//

bool cdGridMap::CellCollides(const cdGridCoord &cell) const {
	if (cell.X < 0 || cell.Y < 0 || cell.X >= m_NumCols || cell.Y >= m_NumRows)
		return true;

	auto idx = cell.Y * m_NumCols + cell.X;

	if (idx >= m_ArraySize || idx < 0)
	{
		return true;
	}

	return m_Cells[idx].Type == cdGridCell::CellType::BLOCKED ? true : false;
}

//------------------------------------------------------------------------------------------------//

f32 cdGridMap::GetHeuristics(const cdGridCoord& cell1,
	const cdGridCoord& cell2, const std::vector<cdGridCoord>& cellList) {
	f32 bestSolution = FLT_MAX;

	for (auto i = cellList.begin(); i != cellList.end(); ++i) {
		auto cell = &(*i);
		f32 result =
			ManhattanDistance(static_cast<f32>(cell1.X),
			static_cast<f32>(cell1.Y),
			static_cast<f32>(cell->X),
			static_cast<f32>(cell->Y));

		if (m_TieType == 0) {
			result *= f32(1.01);
		} else {
			f32 addingTieVal =
				CrossProduct(static_cast<f32>(cell2.X),
				static_cast<f32>(cell2.Y),
				static_cast<f32>(cell1.X),
				static_cast<f32>(cell1.Y),
				static_cast<f32>(cell->X),
				static_cast<f32>(cell->Y)) * f32(0.001f);
			result += addingTieVal;
		}

		if (bestSolution > result) {
			bestSolution = result;
		}
	}

	return bestSolution;
}

//------------------------------------------------------------------------------------------------//

f32 cdGridMap::GetMovementCost(const cdGridCoord& c1, const cdGridCoord& c2) {
	cdGridCoord diff;
	diff.X = abs(c2.X - c1.X);
	diff.Y = abs(c2.Y - c2.Y);

	if (diff.X != 0 && diff.Y != 0) {
		return 1.5f;
	}

	return 1;
}

//------------------------------------------------------------------------------------------------//

cdGridCoord cdGridMap::GetCellCoord(const cdPoint2f& position) {
	return cdGridCoord(static_cast<int>(position.x / m_TileSize.x),
		static_cast<int>(position.y / m_TileSize.y));
}

//------------------------------------------------------------------------------------------------//

cdPoint2f cdGridMap::GetCellPosition(const cdGridCoord& coord) {
	cdPoint2f cellPos;
	cellPos.x = m_TileSize.x * static_cast<f32>(coord.X) + m_TileHalfSize.x;
	cellPos.y = m_TileSize.y * static_cast<f32>(coord.Y) + m_TileHalfSize.y;
	cellPos -= m_MapHalfDimension;

	return cellPos;
}

//------------------------------------------------------------------------------------------------//

void cdGridMap::ComputeWorldPaths(const std::vector<cdGridCoord> &cellPaths,
	std::vector<cdPoint2f> &worldPaths) {
	const auto len = (int)cellPaths.size();
	for (int it = len - 1; it >= 0; --it) {
		worldPaths.push_back(GetCellPosition(cellPaths[it]));
	}
}

}
