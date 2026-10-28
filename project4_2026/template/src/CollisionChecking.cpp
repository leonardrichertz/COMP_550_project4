///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Leonard Richertz, Agustin Zakalik
//////////////////////////////////////

#include "CollisionChecking.h"
#include <algorithm>
#include <cmath>


namespace
{
struct Point
{
    double x, y;
};

struct Line
{
    Point start, end;
};

struct Range
{
    double start, end;
};

bool between(double x, double x_start, double x_end)
{
    return (x >= x_start && x <= x_end) || (x <= x_start && x >= x_end);
}

bool rangesOverlap(Range range1, Range range2)
{
    double start1 = std::min(range1.start, range1.end);
    double end1 = std::max(range1.start, range1.end);
    double start2 = std::min(range2.start, range2.end);
    double end2 = std::max(range2.start, range2.end);

    return start1 <= end2 && start2 <= end1;
}

// Check whether two finite line segments intersect
bool segmentsIntersect(const Line &a, const Line &b)
{
    double ax = a.end.x - a.start.x;
    double ay = a.end.y - a.start.y;
    double bx = b.end.x - b.start.x;
    double by = b.end.y - b.start.y;

    double dx = b.start.x - a.start.x;
    double dy = b.start.y - a.start.y;

    double denominator = ax * by - ay * bx;

    // Parallel lines
    if (denominator == 0.0)
    {
        // Parallel but not on the same line
        if (dx * ay - dy * ax != 0.0)
        {
            return false;
        }

        // Lines are on the same line, check if the segments overlap
        return rangesOverlap({a.start.x, a.end.x}, {b.start.x, b.end.x}) &&
               rangesOverlap({a.start.y, a.end.y}, {b.start.y, b.end.y});
    }

    // Find where the two lines intersect
    double t = (dx * by - dy * bx) / denominator;
    double u = (dx * ay - dy * ax) / denominator;

    // The intersection must lie inside both finite line segments
    return between(t, 0.0, 1.0) && between(u, 0.0, 1.0);
}
} // namespace

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const Rectangle &rect : obstacles)
    {
        bool insideX = (x >= rect.x) && (x <= rect.x + rect.width);
        bool insideY = (y >= rect.y) && (y <= rect.y + rect.height);
        if (insideX && insideY)
        {
            return false;
        }
    }
    return true;
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const Rectangle &rect : obstacles)
    {

        // closest point on the rectangle to the circle's center
        double closestX = std::max(rect.x, std::min(x, rect.x + rect.width));
        double closestY = std::max(rect.y, std::min(y, rect.y + rect.height));

        // Calculate the distance between the circle's center and this closest point
        double distanceX = x - closestX;
        double distanceY = y - closestY;

        // Contact with the obstacle boundary is also a collision.
        if ((distanceX * distanceX + distanceY * distanceY) <= (radius * radius))
        {
            return false;
        }
    }
    // The circle is not intersecting any obstacles
    return true;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Calculate the corners of the square after rotation
    double c = std::cos(theta);
    double s = std::sin(theta);
    double halfSide = sideLength / 2.0;

    // Pre rotation corners of the square centered at the origin
    Point square[4] = {
        {-halfSide, -halfSide},
        {halfSide, -halfSide},
        {halfSide, halfSide},
        {-halfSide, halfSide}
    };

    // Rotate each corner and translate it to the square's position
    for (int i = 0; i < 4; ++i)
    {
        double localX = square[i].x;
        double localY = square[i].y;

        square[i].x = c * localX - s * localY + x;
        square[i].y = s * localX + c * localY + y;
    }
    // now we have the square's corners in the world frame

    for (const Rectangle &rect : obstacles)
    {
        Point rectCorners[4] = {
            {rect.x, rect.y},
            {rect.x + rect.width, rect.y},
            {rect.x + rect.width, rect.y + rect.height},
            {rect.x, rect.y + rect.height}
        };

        // Check every edge of the square against every edge of the obstacle
        for (int i = 0; i < 4; ++i)
        {
            Line squareEdge = {square[i], square[(i + 1) % 4]};

            for (int j = 0; j < 4; ++j)
            {
                Line obstacleEdge = {rectCorners[j], rectCorners[(j + 1) % 4]};

                if (segmentsIntersect(squareEdge, obstacleEdge))
                {
                    return false;
                }
            }
        }

        // Check if the square is completely inside the rectangle
        bool squareInsideX = between(square[0].x, rect.x, rect.x + rect.width);
        bool squareInsideY = between(square[0].y, rect.y, rect.y + rect.height);

        if (squareInsideX && squareInsideY)
        {
            return false;
        }

        // Check if the rectangle is completely inside the square
        // Transform one rectangle corner back into the square's local frame
        double dx = rect.x - x;
        double dy = rect.y - y;

        double localX = c * dx + s * dy;
        double localY = -s * dx + c * dy;

        if (between(localX, -halfSide, halfSide) &&
            between(localY, -halfSide, halfSide))
        {
            return false;
        }
    }

    return true;
}
