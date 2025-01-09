/*!
 * \file atlJumpStartMap.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _ATLJUMPSTARTMAP_H_
#define _ATLJUMPSTARTMAP_H_

#include "atlAStarMap.h"

namespace attila
{
	enum atlGridCellType : char
	{
		EMPTY = '.',
		BLOCKED = '#'
	};

	struct atlGridCoord
	{
		int X, Y;

		inline atlGridCoord(int x = 0, int y = 0) : X(x), Y(y) { }
		inline atlGridCoord(const atlGridCoord &cell) : X(cell.X), Y(cell.Y) { }

		// Operator =.
		inline void operator =(const atlGridCoord &cell)
		{
			X = cell.X;
			Y = cell.Y;
		}

		// Operator ==.
		inline bool operator ==(const atlGridCoord &cell) const
		{
			return (X == cell.X) && (Y == cell.Y);
		}
	};

	class atlJumpStartMap : public atlAStarMap<atlGridCoord>
	{
		private:

			struct Direction
			{
				int X, Y;

				inline Direction(int x = 0, int y = 0) : X(x), Y(y)
				{
				}
			};

			static const Direction DirectionList[8];

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
			
			inline virtual ~atlJumpStartMap(void)
			{

			}

			void Prune(const atlAStar<atlGridCoord>* astar,
				const atlNode<atlGridCoord> & current,
				std::vector< atlGridCoord > & result);

			bool Jump(const atlGridCoord & current,
				int xDir, int yDir,
				const atlGridCoord & start,
				const std::vector<atlGridCoord> & end,
				atlGridCoord &resultNode);

		public:

			bool GetSucessorList(const atlAStar<atlGridCoord>* astar,
				const atlNode<atlGridCoord>& current,
				const atlGridCoord& start,
				const std::vector<atlGridCoord>& end,
				std::vector<atlGridCoord>& adjcentList);
	};
}

#endif
