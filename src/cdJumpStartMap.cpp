/*!
 * \file cdJumpStartMap.cpp
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */

#include <algorithm>

#include "cdHelperMethods.hpp"
#include "cdJumpStartMap.hpp"

namespace {
struct Direction {
    int X, Y;
    constexpr Direction(int x = 0, int y = 0)
        : X(x), Y(y) {}
};

constexpr std::array<Direction, 8> DirectionList = {{
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0},
    {1, 1},
    {1, -1},
    {-1, -1},
    {-1, 1}
}};

}

namespace ceed::ai::path {

	//------------------------------------------------------------------------------------------------//

	cdJumpStartMap::cdJumpStartMap(CellColFunc cellFunc,
		HeuristicsFunc heuFunc,
		MovementCostFunc movFunc,
		int tieType,
		int rows,
		int cols)
		: cdAStarMap<cdGridCoord>(cellFunc,
			fastdelegate::MakeDelegate(this, &cdJumpStartMap::GetSucessorList),
			heuFunc,
			movFunc,
			tieType)
		, m_NumCols(cols)
		, m_NumRows(rows)
		, m_ArraySize(cols * rows) {
	}

	//------------------------------------------------------------------------------------------------//

	void cdJumpStartMap::Prune(const cdAStar<cdGridCoord>* astar,
		const cdNode<cdGridCoord> & current,
		std::vector< cdGridCoord > & result){
		auto curParentIdx = current.ParentIdx;
		auto curNodePos = current.NodePos;

		if (curParentIdx == -1){
			for (int i = 0; i < 8; ++i){
				cdGridCoord coord;
				coord.X = curNodePos.X + DirectionList[i].X;
				coord.Y = curNodePos.Y + DirectionList[i].Y;

				if (Collides(coord) == false)
				{
					result.push_back(coord);
				}
			}
		} else {
			cdNode<cdGridCoord> parentNode;

			if (astar->GetNodeFromClosedList(curParentIdx, parentNode) == true) {
				auto parentNodePos = parentNode.NodePos;

				auto xDiff = curNodePos.X - parentNodePos.X;
				auto yDiff = curNodePos.Y - parentNodePos.Y;

				auto xDir = std::min(std::max(-1, xDiff), 1);
				auto yDir = std::min(std::max(-1, yDiff), 1);

				if (xDir != 0 && yDir != 0) {
					cdGridCoord nodePos;
					nodePos.X = curNodePos.X;
					nodePos.Y = curNodePos.Y + yDir;
					if (Collides(nodePos) == false)
					{
						result.push_back(nodePos);
					}
					nodePos.X = curNodePos.X + xDir;
					nodePos.Y = curNodePos.Y;
					if (Collides(nodePos) == false) {
						result.push_back(nodePos);
					}
					nodePos.X = curNodePos.X + xDir;
					nodePos.Y = curNodePos.Y + yDir;
					if (Collides(nodePos) == false) {
						result.push_back(nodePos);
					}
					if (Collides(cdGridCoord(curNodePos.X - xDir, curNodePos.Y)) == true &&
						Collides(cdGridCoord(curNodePos.X - xDir, curNodePos.Y + yDir)) == false) {
						nodePos.X = curNodePos.X - xDir;
						nodePos.Y = curNodePos.Y + yDir;
						result.push_back(nodePos);
					}
					if (Collides(cdGridCoord(curNodePos.X, curNodePos.Y - yDir)) == true &&
						Collides(cdGridCoord(curNodePos.X + xDir, curNodePos.Y - yDir)) == false) {
						nodePos.X = curNodePos.X + xDir;
						nodePos.Y = curNodePos.Y - yDir;
						result.push_back(nodePos);
					}
				} else {
					if (xDir == 0) {
						cdGridCoord nodePos;
						nodePos.X = curNodePos.X;
						nodePos.Y = curNodePos.Y + yDir;

						if (Collides(nodePos) == false) {
							result.push_back(nodePos);

							if (Collides(cdGridCoord(curNodePos.X - 1, curNodePos.Y)) == true) {
								nodePos.X = curNodePos.X - 1;
								nodePos.Y = curNodePos.Y + yDir;
								result.push_back(nodePos);
							}

							if (Collides(cdGridCoord(curNodePos.X + 1, curNodePos.Y)) == true) {
								nodePos.X = curNodePos.X + 1;
								nodePos.Y = curNodePos.Y + yDir;
								result.push_back(nodePos);
							}
						}
					}
					else {
						cdGridCoord nodePos;
						nodePos.X = curNodePos.X + xDir;
						nodePos.Y = curNodePos.Y;
						if (Collides(nodePos) == false) {
							result.push_back(nodePos);

							if (Collides(cdGridCoord(curNodePos.X, curNodePos.Y - 1)) == true) {
								nodePos.X = curNodePos.X + xDir;
								nodePos.Y = curNodePos.Y - 1;
								result.push_back(nodePos);
							}

							if (Collides(cdGridCoord(curNodePos.X, curNodePos.Y + 1)) == true) {
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

	bool cdJumpStartMap::Jump(const cdGridCoord & current,
		int xDir, int yDir,
		const cdGridCoord & start,
		const std::vector<cdGridCoord> & end,
		cdGridCoord &resultNode) {

		auto nextNode = current;
		nextNode.X += xDir;
		nextNode.Y += yDir;

		auto stepNodePos = nextNode;

		if (Collides(stepNodePos)) {
			return false;
		}

		for (auto jumpCoord : end) {
			if (jumpCoord.X == stepNodePos.X &&
				jumpCoord.Y == stepNodePos.Y) {
				resultNode = jumpCoord;
				return true;
			}
		}

		auto nextNodePos = stepNodePos;

		if (xDir != 0 && yDir != 0) {
			cdGridCoord tmpResult;

			while (1) {
				if ((Collides(cdGridCoord(nextNodePos.X - xDir, nextNodePos.Y + yDir)) == false &&
					Collides(cdGridCoord(nextNodePos.X - xDir, nextNodePos.Y)) == true) ||
					(Collides(cdGridCoord(nextNodePos.X + xDir, nextNodePos.Y - yDir)) == false &&
					Collides(cdGridCoord(nextNodePos.X, nextNodePos.Y - yDir)) == true)) {
					resultNode = nextNodePos;
					return true;
				}

				if (Jump(nextNodePos, xDir, 0, start, end, tmpResult) == true ||
					Jump(nextNodePos, 0, yDir, start, end, tmpResult) == true) {
					resultNode = nextNodePos;
					return true;
				}

				nextNodePos.X += xDir;
				nextNodePos.Y += yDir;

				if (Collides(nextNodePos) == true) {
					return false;
				}

				for (auto jumpCoord : end) {
					if (jumpCoord.X == nextNodePos.X &&
						jumpCoord.Y == nextNodePos.Y) {
						resultNode = jumpCoord;
						return true;
					}
				}
			}
		} else {
			if (xDir == 0) {
				while (1) {
					if ((Collides(cdGridCoord(nextNodePos.X - 1, nextNodePos.Y)) == true &&
						Collides(cdGridCoord(nextNodePos.X - 1, nextNodePos.Y + yDir)) == false) ||
						(Collides(cdGridCoord(nextNodePos.X + 1, nextNodePos.Y)) == true &&
						Collides(cdGridCoord(nextNodePos.X + 1, nextNodePos.Y + yDir)) == false)) {
						resultNode = nextNodePos;
						return true;
					}

					nextNodePos.Y += yDir;

					if (Collides(nextNodePos) == true) {
						return false;
					}

					for (auto jumpCoord : end) {
						if (jumpCoord.X == nextNodePos.X &&
							jumpCoord.Y == nextNodePos.Y) {
							resultNode = jumpCoord;
							return true;
						}
					}
				}
			} else if (yDir == 0) {
				while (1) {
					if ((Collides(cdGridCoord(nextNodePos.X, nextNodePos.Y + 1)) == true &&
						Collides(cdGridCoord(nextNodePos.X + xDir, nextNodePos.Y + 1)) == false) ||
						(Collides(cdGridCoord(nextNodePos.X, nextNodePos.Y - 1)) == true &&
						Collides(cdGridCoord(nextNodePos.X + xDir, nextNodePos.Y - 1)) == false)) {
						resultNode = nextNodePos;
						return true;
					}

					nextNodePos.X += xDir;

					if (Collides(nextNodePos) == true) {
						return false;
					}

					for (auto jumpCoord : end) {
						if (jumpCoord.X == nextNodePos.X &&
							jumpCoord.Y == nextNodePos.Y) {
							resultNode = jumpCoord;
							return true;
						}
					}
				}
			} else {
				return false;
			}
		}

		return Jump(nextNode, xDir, yDir, start, end, resultNode);
	}

	//------------------------------------------------------------------------------------------------//

	bool cdJumpStartMap::GetSucessorList(const cdAStar<cdGridCoord>* astar,
		const cdNode<cdGridCoord>& current,
		const cdGridCoord& start,
		const std::vector<cdGridCoord>& end,
		std::vector<cdGridCoord>& adjcentList) {
		static std::vector< cdGridCoord > nearNodes;
		nearNodes.clear();

		Prune(astar, current, nearNodes);

		if (nearNodes.empty()) {
			return false;
		}

		cdGridCoord resultNode;

		auto currentNodePos = current.NodePos;
		for (auto i = nearNodes.begin(); i != nearNodes.end(); ++i) {
			auto nodePos = *i;

			auto xDiff = nodePos.X - currentNodePos.X;
			auto yDiff = nodePos.Y - currentNodePos.Y;

			auto xDir = std::min(std::max(-1, xDiff), 1);
			auto yDir = std::min(std::max(-1, yDiff), 1);

			if (Jump(currentNodePos, xDir, yDir, start, end, resultNode) == true) {
				adjcentList.push_back(resultNode);
			}
		}

		if (adjcentList.empty() == false) {
			return true;
		}

		return false;
	}

	//------------------------------------------------------------------------------------------------//
}
