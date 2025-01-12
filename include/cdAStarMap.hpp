/*!
 * \file atlAStarMap.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _CDASTARMAP_H_
#define _CDASTARMAP_H_

#include <vector>
#include "cdAstar.h"
#include "FastDelegate.h"

namespace ceed::ai::path {
	template <typename NODE>
	class atlAStarMap {
		friend class cdAStar<NODE>;
		public:
			using CellColFunc = fastdelegate::FastDelegate1<const NODE&, bool>;
			using SucessorFunc = fastdelegate::FastDelegate5<const cdAStar<NODE>*,
				const cdNode<NODE>&,
				const NODE&,
				const std::vector<NODE>&,
				std::vector<NODE>&,
				bool>;
			using HeuristicsFunc = fastdelegate::FastDelegate3< const NODE&,
				const NODE&,
				const std::vector<NODE>&,
				f32 >;
			using MovementCostFunc = fastdelegate::FastDelegate2< const NODE&,
				const NODE&,
				f32 >;

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
			, Collides(cellFunc) {}

			inline virtual ~atlAStarMap(void) {}

			inline void SetTieType(int tieType) { m_TieType = tieType; }
			inline int GetTieType() const { return m_TieType; }
	};
}

#endif
