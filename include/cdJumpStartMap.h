/*!
 * \file cdJumpStartMap.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _CDJUMPSTARTMAP_H_
#define _CDJUMPSTARTMAP_H_

#include "cdAStarMap.h"
#include "cdAstar.h"

namespace ceed::ai::path {
	enum atlGridCellType : char {
		EMPTY = '.',
		BLOCKED = '#'
	};

	struct cdGridCoord {
		int X, Y;

		inline cdGridCoord(int x = 0, int y = 0) : X(x), Y(y) { }
		inline cdGridCoord(const cdGridCoord& cell) : X(cell.X), Y(cell.Y) { }

		inline void operator =(const cdGridCoord& cell) {
			X = cell.X;
			Y = cell.Y;
		}

		inline bool operator ==(const cdGridCoord& cell) const {
			return (X == cell.X) && (Y == cell.Y);
		}
	};

	class atlJumpStartMap : public atlAStarMap<cdGridCoord> {
		private:
			struct Direction {
				int X, Y;
				inline Direction(int x = 0, int y = 0)
				: X(x)
				, Y(y) {}
			};

			static constexpr Direction DirectionList[8];

		protected:
			int m_NumCols;
			int m_NumRows;
			int m_ArraySize;

		protected:

			atlJumpStartMap(CellColFunc cellFunc,
				HeuristicsFunc heuFunc,
				MovementCostFunc movFunc,
				int tieType,
				int rows = 0,
				int cols = 0);

			inline virtual ~atlJumpStartMap() override {}

			void Prune(const cdAStar<cdGridCoord>* astar,
				const cdNode<cdGridCoord> & current,
				std::vector<cdGridCoord> & result);

			bool Jump(const cdGridCoord & current,
				int xDir, int yDir,
				const cdGridCoord& start,
				const std::vector<cdGridCoord>& end,
				cdGridCoord& resultNode);

		public:

			bool GetSucessorList(const cdAStar<cdGridCoord>* astar,
				const cdNode<cdGridCoord>& current,
				const cdGridCoord& start,
				const std::vector<cdGridCoord>& end,
				std::vector<cdGridCoord>& adjcentList);
	};
}

#endif
