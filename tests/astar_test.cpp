#include <gtest/gtest.h>
#include "cdAStar.hpp"
#include "cdJumpStartMap.hpp"
#include "cdGridMap.hpp"

#include <gtest/gtest.h>
#include "cdGridMap.hpp"

using namespace ceed::ai::path;

TEST(CdGridMapTest, ConstructorInitialization) {
    cdGridCellList cells(100, cdGridCell());
    cdPoint2f dimension(10, 10);
    cdGridMap gridMap(cells, 10, 10, dimension);

    EXPECT_EQ(gridMap.GetNumCols(), 10);
    EXPECT_EQ(gridMap.GetNumRows(), 10);
    EXPECT_EQ(gridMap.GetDimension().x, 10);
    EXPECT_EQ(gridMap.GetDimension().y, 10);
}

TEST(CdGridMapTest, CellCollision) {
    cdGridCellList cells(100, cdGridCell()); 
    cells[22].Type = cdGridCell::CellType::BLOCKED;
    cdPoint2f dimension(10, 10);
    cdGridMap gridMap(cells, 10, 10, dimension);

    cdGridCoord blockedCoord(2, 2);
    cdGridCoord emptyCoord(1, 1);

    EXPECT_TRUE(gridMap.CellCollides(blockedCoord));
    EXPECT_FALSE(gridMap.CellCollides(emptyCoord));
}

TEST(CdGridMapTest, GetCellCoordAndPosition) {
    cdGridCellList cells(100, cdGridCell());
    cdPoint2f dimension(100, 100);
    cdGridMap gridMap(cells, 10, 10, dimension);

    cdPoint2f position(1, 2);
    cdGridCoord coord = gridMap.GetCellCoord(position);
    EXPECT_EQ(coord.X, 0);
    EXPECT_EQ(coord.Y, 0);

    cdPoint2f backPosition = gridMap.GetCellPosition(coord);
    EXPECT_EQ(backPosition.x, -45.0f);
    EXPECT_EQ(backPosition.y, -45.0f);
}

TEST(CdGridMapTest, HeuristicsCalculation) {
    cdGridCellList cells(100, cdGridCell());
    cdPoint2f dimension(10, 10);
    cdGridMap gridMap(cells, 10, 10, dimension);

    cdGridCoord start(0, 0);
    cdGridCoord goal(3, 4);

    std::vector<cdGridCoord> goals = {goal};

    f32 heuristics = gridMap.GetHeuristics(start, goal, goals);
    EXPECT_GT(heuristics, 0);
}

TEST(CdGridMapTest, MovementCostCalculation) {
    cdGridCellList cells(100, cdGridCell());
    cdPoint2f dimension(10, 10);
    cdGridMap gridMap(cells, 10, 10, dimension);

    cdGridCoord start(0, 0);
    cdGridCoord neighbor(1, 0);

    f32 cost = gridMap.GetMovementCost(start, neighbor);
    EXPECT_GT(cost, 0);
}

TEST(CdGridMapTest, ComputeWorldPaths) {
    cdGridCellList cells(100, cdGridCell()); 
    cdPoint2f dimension(100, 100);
    cdGridMap gridMap(cells, 10, 10, dimension);

    std::vector<cdGridCoord> cellPaths = {{0, 0}, {4, 4}, {9, 9}};
    std::vector<cdPoint2f> worldPaths;

    gridMap.ComputeWorldPaths(cellPaths, worldPaths);

    ASSERT_EQ(worldPaths.size(), cellPaths.size());
    EXPECT_EQ(worldPaths[0].x, 45.0f);
    EXPECT_EQ(worldPaths[0].y, 45.0f);
    EXPECT_EQ(worldPaths[1].x, -5.0f);
    EXPECT_EQ(worldPaths[1].y, -5.0f);
    EXPECT_EQ(worldPaths[2].x, -45.0f);
    EXPECT_EQ(worldPaths[2].y, -45.0f);
}

TEST(CdGridMapTest, PathFindingEmptyMap) {
    cdGridCellList cells(100, cdGridCell());
    cdPoint2f dimension(10, 10);
    cdGridMap gridMap(cells, 10, 10, dimension);

    cdAStar<cdGridCoord> aStar;
    std::vector<cdGridCoord> resultPath;

    cdGridCoord start(0, 0);
    cdGridCoord end(5, 5);

    bool found = aStar.FindPath(start, end, &gridMap, resultPath);
    EXPECT_TRUE(found);

    EXPECT_EQ(resultPath.size(), 2);
}

TEST(CdGridMapTest, PathFindingBlockedMap) {
    cdGridCellList cells(100, cdGridCell(cdGridCell::CellType::BLOCKED));
    cdPoint2f dimension(10, 10);
    cdGridMap gridMap(cells, 10, 10, dimension);

    cdAStar<cdGridCoord> aStar;
    std::vector<cdGridCoord> resultPath;

    cdGridCoord start(0, 0);
    cdGridCoord end(5, 5);

    bool found = aStar.FindPath(start, end, &gridMap, resultPath);
    EXPECT_FALSE(found);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
