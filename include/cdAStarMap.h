/*!
 * \file atlAStarMap.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _ATLASTARMAP_H_
#define _ATLASTARMAP_H_

#include <vector>
#include "atlAStar.h"
#include "../External/FastDelegate.h"

namespace attila
{
	template <typename CELL>
	class atlAStarMap
	{
		friend class atlAStar<CELL>;

		public:

			using CellColFunc = fastdelegate::FastDelegate1<const CELL&, bool>;
			using SucessorFunc = fastdelegate::FastDelegate5<const atlAStar<CELL>*,
				const atlNode<CELL>&,
				const CELL&,
				const std::vector<CELL>&,
				std::vector<CELL>&,
				bool>;
			using HeuristicsFunc = fastdelegate::FastDelegate3< const CELL&,
				const CELL&,
				const std::vector<CELL>&,
				atlFloat >;
			using MovementCostFunc = fastdelegate::FastDelegate2< const CELL&,
				const CELL&,
				atlFloat >;

		protected:
			int m_TieType;

			SucessorFunc GetSucessors;
			HeuristicsFunc Heuristics;
			MovementCostFunc MovementCost;

		public:

			CellColFunc Collides;

		public:

			inline atlAStarMap(CellColFunc cellFunc, 
				SucessorFunc sucFunc,
				HeuristicsFunc heuFunc,
				MovementCostFunc movFunc,
				int tieType = 0)
				: m_TieType(tieType)
				, GetSucessors(sucFunc)
				, Heuristics(heuFunc)
				, MovementCost(movFunc)
				, Collides(cellFunc)
			{
			}

			inline virtual ~atlAStarMap(void)
			{
			}

			inline void SetTieType(int tieType) { m_TieType = tieType; }
			inline int GetTieType(void) const { return m_TieType; }
	};
}

#endif
