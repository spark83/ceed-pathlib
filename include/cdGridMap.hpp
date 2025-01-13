/*!
 * \file cdGridMap.hpp
 * Copyright (c) Punch First 2014 - 2016 All rights reserved.
 */
#ifndef _CDGRIDMAP_HPP_
#define _CDGRIDMAP_HPP_

#include <vector>
#include "cdJumpStartMap.hpp"

namespace ceed::ai::path {

template <typename T>
struct cdPoint2 {
    T x, y;

    // Constructor
    cdPoint2(T x = 0, T y = 0) : x(x), y(y) {}

    cdPoint2 operator+(const cdPoint2& other) const {
        return cdPoint2(x + other.x, y + other.y);
    }

    cdPoint2 operator-(const cdPoint2& other) const {
        return cdPoint2(x - other.x, y - other.y);
    }

    cdPoint2 operator*(s32 scalar) const {
        return cdPoint2(x * scalar, y * scalar);
    }

    cdPoint2 operator/(s32 scalar) const {
        return cdPoint2(x / scalar, y / scalar);
    }

    cdPoint2& operator+=(const cdPoint2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    cdPoint2& operator-=(const cdPoint2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    cdPoint2& operator*=(T scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    cdPoint2& operator/=(T scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    cdPoint2 operator-() const {
        return cdPoint2(-x, -y);
    }

    cdPoint2 operator+() const {
        return *this;
    }

    bool operator==(const cdPoint2& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const cdPoint2& other) const {
        return !(*this == other);
    }
};

using cdPoint2f = cdPoint2<f32>;
using cdPoint2i = cdPoint2<s32>;

struct cdGridCell {
    enum CellType : char {
        EMPTY,
        BLOCKED
    };

    inline cdGridCell(CellType type = CellType::EMPTY)
    : Type(type)
    , Threat(0.0f) {}

    inline cdGridCell(const cdGridCell& cell)
    : Type(cell.Type)
    , Threat(cell.Threat) {}

    CellType Type;
    f32 Threat;
};

using cdGridCellList = std::vector<cdGridCell>;

class cdGridMap : public cdJumpStartMap {
    private:

        int m_ArraySize;

        cdPoint2f m_TileSize;
        cdPoint2f m_TileHalfSize;
        cdPoint2f m_MapDimension;
        cdPoint2f m_MapHalfDimension;

        cdGridCellList m_Cells;

    public:

        cdGridMap(cdGridCellList& cells, int cols, int rows, cdPoint2f& dimension);

        bool CellCollides(const cdGridCoord &) const;

        f32 GetHeuristics(const cdGridCoord &,
            const cdGridCoord &,
            const std::vector<cdGridCoord> &);
        f32 GetMovementCost(const cdGridCoord &,
            const cdGridCoord &);

        cdGridCoord GetCellCoord(const cdPoint2f& position);
        cdPoint2f GetCellPosition(const cdGridCoord& coord);
        void ComputeWorldPaths(const std::vector<cdGridCoord>& cellPaths,
            std::vector<cdPoint2f>& worldPaths);

        inline int GetNumCols(void) const {
            return m_NumCols;
        }

        inline int GetNumRows(void) const {
            return m_NumRows;
        }

        inline const cdPoint2f& GetDimension(void) const {
            return m_MapDimension;
        }
};
}

#endif
