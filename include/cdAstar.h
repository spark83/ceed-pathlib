/*!
 * \file atlAStar.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _ATLASTAR_H_
#define _ATLASTAR_H_

#include <vector>
#include <functional>
#include <queue>
#include "../Util/atlTypes.h"

namespace attila
{
	template <typename CELL> class atlAStarMap;

	template <typename CELL>
	struct atlNode
	{
		CELL NodePos;
		atlInt ParentIdx;
		atlFloat GValue;
		atlFloat HValue;

		inline atlNode(void)
			: ParentIdx(-1)
			, GValue(0)
			, HValue(0)
		{
		}

		inline atlNode(const CELL &pos,
			atlFloat gVal = 0,
			atlFloat hVal = 0,
			atlInt parentIdx = -1)
			: NodePos(pos)
			, ParentIdx(parentIdx)
			, GValue(gVal)
			, HValue(hVal)
		{
		}

		inline atlNode(const atlNode& node)
			: NodePos(node.NodePos)
			, ParentIdx(node.ParentIdx)
			, GValue(node.GValue)
			, HValue(node.HValue)
		{

		}

		inline ~atlNode(void)
		{

		}

		inline atlFloat GetScore(void) const
		{
			return GValue + HValue;
		}

		inline bool operator < (const atlNode & n1) const
		{
			return (GValue + HValue) < (n1.GValue + n1.HValue);
		}

		inline bool operator > (const atlNode & n1) const
		{
			return (GValue + HValue) > (n1.GValue + n1.HValue);
		}

		inline bool operator == (const atlNode & n1) const
		{
			return NodePos == n1.NodePos;
		}
	};

	template <typename CELL>
	class atlAStar
	{
		public:

			using atlMovePath = std::vector<CELL>;
			using GREATER = std::greater<atlNode<CELL>>;

		private:

			// Open lists and closed lists.
			std::vector< atlNode<CELL> > m_OpenList; // Nodes that are in the open set.
			std::vector< atlNode<CELL> > m_ClosedList; // Nodes that are visited.

			// STL thingie that compares stuff.
			GREATER m_Compare;

		private:

			inline bool IsInOpenList(const atlNode<CELL> &node) const
			{
				// Find the node first.
				typename std::vector<atlNode<CELL>>::const_iterator i =
					find(m_OpenList.begin(), m_OpenList.end(), node);

				// If in the list return true or else return false.
				return i == m_OpenList.end() ? false : true;
			}

			inline bool IsInClosedList(const atlNode<CELL> &node) const
			{
				// Find the node first.
				typename std::vector<atlNode<CELL>>::const_iterator i =
					find(m_ClosedList.begin(), m_ClosedList.end(), node);

				// If in the list return true or else return false.
				return i == m_ClosedList.end() ? false : true;
			}

		public:

			inline atlAStar(void)
			{

			}

			inline ~atlAStar(void)
			{

			}

			inline bool GetNodeFromClosedList(const int idx, atlNode<CELL> &node) const
			{
				auto listSize = static_cast<int>(m_ClosedList.size());

				if (idx < listSize)
				{
					node = m_ClosedList[idx];
					return true;
				}

				return false;
			}

			inline bool FindPath(const CELL &start,
				const std::vector<CELL> &endPts,
				atlAStarMap<CELL> *pMap,
				atlMovePath &resultPath)
			{
				// This is the pseudo code I got from the book called "AI for Game Developers"
				//
				// A* pseudo code 
				// Add the starting node to the open list
				// while the open list is not empty
				// {
				//	current node = node from open list with the lowest cost
				//	if current node = goal node then
				//		path complete
				//	else
				//		move current node to the closed list
				//		examine each node adjacent to the current node
				//		for each adjacent node
				//			if it isn't on the openlist and isn't on the closed list and it isn't an obstacle then
				//				move it to open list and calculate cost.

				// Pseudo code I got from www.policyalmanac.org/games/aStarTutorial.htm
				// 1) Add the starting square (or node) to the open list. 
				// 2) Repeat the following:
				//		a) Look for the lowest F cost square on the open list. We refer to this as the current square.
				//		b) Switch it to the closed list. 
				//		c) For each of the 8 squares adjacent to this current square ?
				//			-If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following.            
				//			-If it isn't on the open list, add it to the open list. Make the current square the parent of 
				//			this square. Record the F, G, and H costs of the square.  
				//			-If it is on the open list already, check to see if this path to that square is better, using G 
				//			cost as the measure. A lower G cost means that this is a better path. If so, change the parent 
				//			of the square to the current square, and recalculate the G and F scores of the square. If you 
				//			are keeping your open list sorted by F score, you may need to resort the list to account for 
				//			the change. 
				//		d) Stop when you:
				//		Add the target square to the closed list, in which case the path has been found (see note below), or 
				//		Fail to find the target square, and the open list is empty. In this case, there is no path.    
				// 3) Save the path. Working backwards from the target square, go from each square to its parent square until you reach the starting square. That is your path. 

				// My A* code is mixture of above two pseudo codes and plus extra stuff I added on my own.

				static std::vector<CELL> adjcentList;
				m_ClosedList.clear();
				m_OpenList.clear();

				auto compare = m_Compare;

				// Parent looking for his/her children... 
				m_OpenList.push_back(atlNode<CELL>(start, 0, 0));
				push_heap(m_OpenList.begin(), m_OpenList.end(), compare);

				// While the open list is not empty.
				while (!m_OpenList.empty())
				{
					// Current node = node from open list with the lowest cost
					atlNode<CELL> current = m_OpenList.front();

					// Binary heap is good for A* so do the binary heap thing.
					pop_heap(m_OpenList.begin(), m_OpenList.end(), compare);
					m_OpenList.pop_back();

					// For each destination points check if the path is found...
					for (typename std::vector<CELL>::const_iterator end = endPts.begin();
						end != endPts.end(); ++end)
					{
						// Path is found. 
						if (current == atlNode<CELL>((*end), 0, 0))
						{
							// Finally parent met his/her child.
							m_ClosedList.push_back(current);

							// But children doesn't know where his/her parents are coming from...
							// So here you do the search.
							// Note: I could've made this function to return closed list but I like it this way somehow.
							int idx = static_cast<int>(m_ClosedList.size()) - 1;
							while (m_ClosedList[idx].ParentIdx != -1)
							{
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
					if (pMap->GetSucessors(this, current, start, endPts, adjcentList))
					{
						// For each adjacent nodes do the following.
						for (typename std::vector<CELL>::iterator i = adjcentList.begin();
							i != adjcentList.end(); ++i)
						{
							// If there is no barrier in the position and if the node is not in the closed list.
							// do the following.
							if (!pMap->Collides(*i) && !IsInClosedList(atlNode<CELL>(*i, 0, 0)))
							{
								// Makde new node. (one of the adjacent node)
								atlNode<CELL> newNode;
								newNode.NodePos = (*i); // Assign a position
								// Find out who is this node's parent.
								newNode.ParentIdx = static_cast<int>(m_ClosedList.size()) - 1;
								// Calculate cost.
								newNode.GValue =
									current.GValue +
									pMap->MovementCost(current.NodePos, newNode.NodePos);
								// Calculate heruristics.
								newNode.HValue = pMap->Heuristics(newNode.NodePos, start, endPts);

								typename std::vector<atlNode<CELL>>::iterator newNodePos =
									find(m_OpenList.begin(), m_OpenList.end(), newNode);

								// If that node is not in the openlist.
								if (newNodePos == m_OpenList.end())
								{
									// Put it into open list and do the binary heap thing.
									m_OpenList.push_back(newNode);
									push_heap(m_OpenList.begin(), m_OpenList.end(), compare);
								}
								// This code below is kinda hard to explain so I will copy and paste the pesudo code again.
								// -If it is on the open list already, check to see if this path to that square is 
								// better, using G cost as the measure. A lower G cost means that this is a better 
								// path. If so, change the parent of the square to the current square, and recalculate 
								// the G and F scores of the square. If you are keeping your open list sorted by F 
								// score, you may need to resort the list to account for the change. 
								else
								{
									// So if this path is better.
									if (current.GValue +
										m_ClosedList[current.ParentIdx].GValue <
										newNodePos->GValue)
									{
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

			inline bool FindPath(const CELL &start,
				const CELL &end,
				atlAStarMap<CELL> *pMap,
				atlMovePath &resultPath)
			{
				static std::vector<CELL> endList;
				endList.clear();
				endList.push_back(end);

				return FindPath(start, endList, pMap, resultPath);
			}
	};
}

#endif
