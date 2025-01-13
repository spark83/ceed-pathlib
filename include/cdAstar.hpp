/*!
 * \file cdAStar.hpp
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _CDASTAR_HPP_
#define _CDASTAR_HPP_

#include <vector>
#include <functional>
#include <queue>

#include "cdTypes.h"

namespace ceed::ai::path {
	template <typename NODE> class cdAStarMap;

	template <typename NODE>
	struct cdNode {
		NODE NodePos;
		s32 ParentIdx;
		f32 GValue;
		f32 HValue;

		inline cdNode()
		: ParentIdx(-1)
		, GValue(0)
		, HValue(0) {}

		inline cdNode(const NODE& pos,
			f32 gVal = 0,
			f32 hVal = 0,
			s32 parentIdx = -1)
		: NodePos(pos)
		, ParentIdx(parentIdx)
		, GValue(gVal)
		, HValue(hVal) {}

		inline cdNode(const cdNode& node)
			: NodePos(node.NodePos)
			, ParentIdx(node.ParentIdx)
			, GValue(node.GValue)
			, HValue(node.HValue) {}

		inline f32 GetScore() const {
			return GValue + HValue;
		}

		inline bool operator < (const cdNode& n1) const {
			return (GValue + HValue) < (n1.GValue + n1.HValue);
		}

		inline bool operator > (const cdNode& n1) const {
			return (GValue + HValue) > (n1.GValue + n1.HValue);
		}

		inline bool operator == (const cdNode& n1) const {
			return NodePos == n1.NodePos;
		}
	};

	template <typename CELL, size_t ListSize = 40000>
	class cdAStar {
		public:

			using cdMovePath = std::vector<CELL>;
			using GREATER = std::greater<cdNode<CELL>>;

		private:

			// Open lists and closed lists.
			std::vector<cdNode<CELL>> m_OpenList; // Nodes that are in the open set.
			std::vector<cdNode<CELL>> m_ClosedList; // Nodes that are visited.

			// STL thingie that compares stuff.
			GREATER m_Compare;

		private:

			bool IsInOpenList(const cdNode<CELL>& node) const {
				// Find the node first.
				typename std::vector<cdNode<CELL>>::const_iterator i =
					find(m_OpenList.begin(), m_OpenList.end(), node);

				// If in the list return true or else return false.
				return i == m_OpenList.end() ? false : true;
			}

			bool IsInClosedList(const cdNode<CELL>& node) const {
				// Find the node first.
				typename std::vector<cdNode<CELL>>::const_iterator i =
					find(m_ClosedList.begin(), m_ClosedList.end(), node);

				// If in the list return true or else return false.
				return i == m_ClosedList.end() ? false : true;
			}

		public:

			cdAStar() {
				m_OpenList.reserve(ListSize);
				m_ClosedList.reserve(ListSize);
			}

			bool GetNodeFromClosedList(const int idx, cdNode<CELL>& node) const {
				auto listSize = static_cast<int>(m_ClosedList.size());
				if (idx < listSize) {
					node = m_ClosedList[idx];
					return true;
				}
				return false;
			}

			bool FindPath(const CELL &start,
				const std::vector<CELL>& endPts,
				cdAStarMap<CELL>* pMap,
				cdMovePath& resultPath) {

				static std::vector<CELL> adjcentList;
				m_ClosedList.clear();
				m_OpenList.clear();

				auto compare = m_Compare;

				// Parent looking for his/her children...
				m_OpenList.push_back(cdNode<CELL>(start, 0, 0));
				push_heap(m_OpenList.begin(), m_OpenList.end(), compare);

				// While the open list is not empty.
				while (!m_OpenList.empty()) {
					// Current node = node from open list with the lowest cost
					cdNode<CELL> current = m_OpenList.front();

					// Binary heap is good for A* so do the binary heap thing.
					pop_heap(m_OpenList.begin(), m_OpenList.end(), compare);
					m_OpenList.pop_back();

					// For each destination points check if the path is found...
					for (typename std::vector<CELL>::const_iterator end = endPts.begin();
						end != endPts.end(); ++end) {
						// Path is found.
						if (current == cdNode<CELL>((*end), 0, 0)) {
							// Finally parent met his/her child.
							m_ClosedList.push_back(current);

							// But children doesn't know where his/her parents are coming from...
							// So here you do the search.
							// Note: I could've made this function to return closed list but I like it this way somehow.
							int idx = static_cast<int>(m_ClosedList.size()) - 1;
							while (m_ClosedList[idx].ParentIdx != -1) {
								resultPath.push_back(m_ClosedList[idx].NodePos);
								idx = m_ClosedList[idx].ParentIdx;
							}
							resultPath.push_back(m_ClosedList[idx].NodePos);

							// Yay I know where my parents are. Thank you A* :D.
							return true;
						}
					}

					// Move current node to the closed list.
					m_ClosedList.push_back(current);

					// You want to get lists of adjcent nodes that are around the current guy.
					adjcentList.clear();
					if (pMap->GetSucessors(this, current, start, endPts, adjcentList)) {
						// For each adjacent nodes do the following.
						for (typename std::vector<CELL>::iterator i = adjcentList.begin();
							i != adjcentList.end(); ++i) {
							// If there is no barrier in the position and if the node is not in the closed list.
							// do the following.
							if (!pMap->Collides(*i) && !IsInClosedList(cdNode<CELL>(*i, 0, 0))) {
								// Makde new node. (one of the adjacent node)
								cdNode<CELL> newNode;
								newNode.NodePos = (*i); // Assign a position
								// Find out who is this node's parent.
								newNode.ParentIdx = static_cast<int>(m_ClosedList.size()) - 1;
								// Calculate cost.
								newNode.GValue =
									current.GValue +
									pMap->MovementCost(current.NodePos, newNode.NodePos);
								// Calculate heruristics.
								newNode.HValue = pMap->Heuristics(newNode.NodePos, start, endPts);

								typename std::vector<cdNode<CELL>>::iterator newNodePos =
									find(m_OpenList.begin(), m_OpenList.end(), newNode);

								// If that node is not in the openlist.
								if (newNodePos == m_OpenList.end()) {
									// Put it into open list and do the binary heap thing.
									m_OpenList.push_back(newNode);
									push_heap(m_OpenList.begin(), m_OpenList.end(), compare);
								} else {
									// So if this path is better.
									if (current.GValue +
										m_ClosedList[current.ParentIdx].GValue <
										newNodePos->GValue) {
										// Then change the parent of the node to the current node and recalculate the
										// G and F scores of the square.
										newNodePos->ParentIdx = (int)m_ClosedList.size() - 1;
										newNodePos->GValue = current.GValue +
											pMap->MovementCost(current.NodePos,
											newNodePos->NodePos);

										// Finally do the binary heap thing to make thing sorted.
										push_heap(m_OpenList.begin(), newNodePos + 1, compare);
									}
								}
							}
						}
					}
				}

				return false;
			}

			bool FindPath(const CELL& start,
				const CELL& end,
				cdAStarMap<CELL>* pMap,
				cdMovePath& resultPath) {
				static std::vector<CELL> endList;
				endList.clear();
				endList.push_back(end);

				return FindPath(start, endList, pMap, resultPath);
			}
	};
}

#endif
