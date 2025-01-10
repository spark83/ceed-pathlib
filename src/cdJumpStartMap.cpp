/*!
 * \file cdJumpStartMap.h
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */

#include "../Util/atlHelperMethods.h"
#include "cdJumpStartMap.h"

namespace ceed::ai::path {
	//------------------------------------------------------------------------------------------------//

	const cdJumpStartMap::Direction cdJumpStartMap::DirectionList[8] =
	{
		cdJumpStartMap::Direction(0, 1),
		cdJumpStartMap::Direction(1, 0),
		cdJumpStartMap::Direction(0, -1),
		cdJumpStartMap::Direction(-1, 0),
		cdJumpStartMap::Direction(1, 1),
		cdJumpStartMap::Direction(1, -1),
		cdJumpStartMap::Direction(-1, -1),
		cdJumpStartMap::Direction(-1, 1),
	};

	//------------------------------------------------------------------------------------------------//

	cdJumpStartMap::cdJumpStartMap(CellColFunc cellFunc,
		HeuristicsFunc heuFunc,
		MovementCostFunc movFunc,
		int tieType,
		int rows,
		int cols)
		: atlAStarMap<atlGridCoord>(cellFunc,
		fastdelegate::MakeDelegate(this, &cdJumpStartMap::GetSucessorList),
		heuFunc,
		movFunc,
		tieType)
		, m_NumCols(cols)
		, m_NumRows(rows)
		, m_ArraySize(cols * rows)
	{

	}

	//------------------------------------------------------------------------------------------------//

	void cdJumpStartMap::Prune(const atlAStar<atlGridCoord>* astar,
		const atlNode<atlGridCoord> & current,
		std::vector< atlGridCoord > & result)
	{
		auto curParentIdx = current.ParentIdx;
		auto curNodePos = current.NodePos;

		if (curParentIdx == -1)
		{
			auto pDirLists = DirectionList;
			for (int i = 0; i < 8; ++i)
			{
				atlGridCoord coord;
				coord.X = curNodePos.X + pDirLists[i].X;
				coord.Y = curNodePos.Y + pDirLists[i].Y;

				if (Collides(coord) == false)
				{
					result.push_back(coord);
				}
			}
		}
		else
		{
			atlNode<atlGridCoord> parentNode;

			if (astar->GetNodeFromClosedList(curParentIdx, parentNode) == true)
			{
				auto parentNodePos = parentNode.NodePos;

				auto xDiff = curNodePos.X - parentNodePos.X;
				auto yDiff = curNodePos.Y - parentNodePos.Y;

				auto xDir = Min(Max(-1, xDiff), 1);
				auto yDir = Min(Max(-1, yDiff), 1);

				if (xDir != 0 && yDir != 0)
				{
					atlGridCoord nodePos;
					nodePos.X = curNodePos.X;
					nodePos.Y = curNodePos.Y + yDir;
					if (Collides(nodePos) == false)
					{
						result.push_back(nodePos);
					}
					nodePos.X = curNodePos.X + xDir;
					nodePos.Y = curNodePos.Y;
					if (Collides(nodePos) == false)
					{
						result.push_back(nodePos);
					}
					nodePos.X = curNodePos.X + xDir;
					nodePos.Y = curNodePos.Y + yDir;
					if (Collides(nodePos) == false)
					{
						result.push_back(nodePos);
					}
					if (Collides(atlGridCoord(curNodePos.X - xDir, curNodePos.Y)) == true &&
						Collides(atlGridCoord(curNodePos.X - xDir, curNodePos.Y + yDir)) == false)
					{
						nodePos.X = curNodePos.X - xDir;
						nodePos.Y = curNodePos.Y + yDir;
						result.push_back(nodePos);
					}
					if (Collides(atlGridCoord(curNodePos.X, curNodePos.Y - yDir)) == true &&
						Collides(atlGridCoord(curNodePos.X + xDir, curNodePos.Y - yDir)) == false)
					{
						nodePos.X = curNodePos.X + xDir;
						nodePos.Y = curNodePos.Y - yDir;
						result.push_back(nodePos);
					}
				}
				else
				{
					if (xDir == 0)
					{
						atlGridCoord nodePos;
						nodePos.X = curNodePos.X;
						nodePos.Y = curNodePos.Y + yDir;

						if (Collides(nodePos) == false)
						{
							result.push_back(nodePos);

							if (Collides(atlGridCoord(curNodePos.X - 1, curNodePos.Y)) == true)
							{
								nodePos.X = curNodePos.X - 1;
								nodePos.Y = curNodePos.Y + yDir;
								result.push_back(nodePos);
							}

							if (Collides(atlGridCoord(curNodePos.X + 1, curNodePos.Y)) == true)
							{
								nodePos.X = curNodePos.X + 1;
								nodePos.Y = curNodePos.Y + yDir;
								result.push_back(nodePos);
							}
						}
					}
					else
					{
						atlGridCoord nodePos;
						nodePos.X = curNodePos.X + xDir;
						nodePos.Y = curNodePos.Y;
						if (Collides(nodePos) == false)
						{
							result.push_back(nodePos);

							if (Collides(atlGridCoord(curNodePos.X, curNodePos.Y - 1)) == true)
							{
								nodePos.X = curNodePos.X + xDir;
								nodePos.Y = curNodePos.Y - 1;
								result.push_back(nodePos);
							}

							if (Collides(atlGridCoord(curNodePos.X, curNodePos.Y + 1)) == true)
							{
								nodePos.X = curNodePos.X + xDir;
								nodePos.Y = curNodePos.Y + 1;
								result.push_back(nodePos);
							}
						}
					}
				}
			}
		}
	}

	//------------------------------------------------------------------------------------------------//

	bool cdJumpStartMap::Jump(const atlGridCoord & current,
		int xDir, int yDir,
		const atlGridCoord & start,
		const std::vector<atlGridCoord> & end,
		atlGridCoord &resultNode)
	{
		auto nextNode = current;
		nextNode.X += xDir;
		nextNode.Y += yDir;

		auto stepNodePos = nextNode;

		if (Collides(stepNodePos) == true)
		{
			return false;
		}

		for (auto jumpCoord : end)
		{
			if (jumpCoord.X == stepNodePos.X &&
				jumpCoord.Y == stepNodePos.Y)
			{
				resultNode = jumpCoord;
				return true;
			}
		}

		auto nextNodePos = stepNodePos;

		if (xDir != 0 && yDir != 0)
		{
			atlGridCoord tmpResult;

			while (1)
			{
				if ((Collides(atlGridCoord(nextNodePos.X - xDir, nextNodePos.Y + yDir)) == false &&
					Collides(atlGridCoord(nextNodePos.X - xDir, nextNodePos.Y)) == true) ||
					(Collides(atlGridCoord(nextNodePos.X + xDir, nextNodePos.Y - yDir)) == false &&
					Collides(atlGridCoord(nextNodePos.X, nextNodePos.Y - yDir)) == true))
				{
					resultNode = nextNodePos;
					return true;
				}

				if (Jump(nextNodePos, xDir, 0, start, end, tmpResult) == true ||
					Jump(nextNodePos, 0, yDir, start, end, tmpResult) == true)
				{
					resultNode = nextNodePos;
					return true;
				}

				nextNodePos.X += xDir;
				nextNodePos.Y += yDir;

				if (Collides(nextNodePos) == true)
				{
					return false;
				}

				for (auto jumpCoord : end)
				{
					if (jumpCoord.X == nextNodePos.X &&
						jumpCoord.Y == nextNodePos.Y)
					{
						resultNode = jumpCoord;
						return true;
					}
				}
			}
		}
		else
		{
			if (xDir == 0)
			{
				while (1)
				{
					if ((Collides(atlGridCoord(nextNodePos.X - 1, nextNodePos.Y)) == true &&
						Collides(atlGridCoord(nextNodePos.X - 1, nextNodePos.Y + yDir)) == false) ||
						(Collides(atlGridCoord(nextNodePos.X + 1, nextNodePos.Y)) == true &&
						Collides(atlGridCoord(nextNodePos.X + 1, nextNodePos.Y + yDir)) == false))
					{
						resultNode = nextNodePos;
						return true;
					}

					nextNodePos.Y += yDir;

					if (Collides(nextNodePos) == true)
					{
						return false;
					}

					for (auto jumpCoord : end)
					{
						if (jumpCoord.X == nextNodePos.X &&
							jumpCoord.Y == nextNodePos.Y)
						{
							resultNode = jumpCoord;
							return true;
						}
					}
				}
			}
			else if (yDir == 0)
			{
				while (1)
				{
					if ((Collides(atlGridCoord(nextNodePos.X, nextNodePos.Y + 1)) == true &&
						Collides(atlGridCoord(nextNodePos.X + xDir, nextNodePos.Y + 1)) == false) ||
						(Collides(atlGridCoord(nextNodePos.X, nextNodePos.Y - 1)) == true &&
						Collides(atlGridCoord(nextNodePos.X + xDir, nextNodePos.Y - 1)) == false))
					{
						resultNode = nextNodePos;
						return true;
					}

					nextNodePos.X += xDir;

					if (Collides(nextNodePos) == true)
					{
						return false;
					}

					for (auto jumpCoord : end)
					{
						if (jumpCoord.X == nextNodePos.X &&
							jumpCoord.Y == nextNodePos.Y)
						{
							resultNode = jumpCoord;
							return true;
						}
					}
				}
			}
			else
			{
				return false;
			}
		}

		return Jump(nextNode, xDir, yDir, start, end, resultNode);
	}

	//------------------------------------------------------------------------------------------------//

	bool cdJumpStartMap::GetSucessorList(const atlAStar<atlGridCoord>* astar,
		const atlNode<atlGridCoord>& current,
		const atlGridCoord& start,
		const std::vector<atlGridCoord>& end,
		std::vector<atlGridCoord>& adjcentList)
	{
		static std::vector< atlGridCoord > nearNodes;
		nearNodes.clear();

		Prune(astar, current, nearNodes);

		if (nearNodes.empty())
		{
			return false;
		}

		atlGridCoord resultNode;

		auto currentNodePos = current.NodePos;
		for (auto i = nearNodes.begin(); i != nearNodes.end(); ++i)
		{
			auto nodePos = *i;

			auto xDiff = nodePos.X - currentNodePos.X;
			auto yDiff = nodePos.Y - currentNodePos.Y;

			auto xDir = Min(Max(-1, xDiff), 1);
			auto yDir = Min(Max(-1, yDiff), 1);

			if (Jump(currentNodePos, xDir, yDir, start, end, resultNode) == true)
			{
				adjcentList.push_back(resultNode);
			}
		}

		if (adjcentList.empty() == false)
		{
			return true;
		}

		return false;
	}

	//------------------------------------------------------------------------------------------------//
}
