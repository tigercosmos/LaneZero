/*
 * Copyright (c) 2025, LaneZero Contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cmath>
#include <vector>

namespace LaneZero
{

/**
 * Axis-Aligned Bounding Box (AABB) for simple collision detection.
 * The box is aligned with the world coordinate system (not rotated).
 * This provides fast collision detection but may be conservative for rotated vehicles.
 */
struct AxisAlignedBoundingBox
{
    double min_x_m = 0.0;
    double min_y_m = 0.0;
    double max_x_m = 0.0;
    double max_y_m = 0.0;

    AxisAlignedBoundingBox() = default;

    AxisAlignedBoundingBox(double min_x, double min_y, double max_x, double max_y)
        : min_x_m(min_x)
        , min_y_m(min_y)
        , max_x_m(max_x)
        , max_y_m(max_y)
    {
    }

    /**
     * Create an AABB from a center point, dimensions, and rotation angle.
     * The AABB will be axis-aligned and contain the entire rotated rectangle.
     *
     * @param center_x_m X coordinate of the center point in world frame
     * @param center_y_m Y coordinate of the center point in world frame
     * @param length_m Length of the object (along its local x-axis)
     * @param width_m Width of the object (along its local y-axis)
     * @param yaw_rad Rotation angle in radians (yaw)
     */
    static AxisAlignedBoundingBox from_rotated_rectangle(
        double center_x_m,
        double center_y_m,
        double length_m,
        double width_m,
        double yaw_rad)
    {
        double cos_yaw = std::cos(yaw_rad);
        double sin_yaw = std::sin(yaw_rad);

        double half_length = length_m / 2.0;
        double half_width = width_m / 2.0;

        std::vector<std::pair<double, double>> corners;
        corners.push_back({half_length, half_width});
        corners.push_back({half_length, -half_width});
        corners.push_back({-half_length, half_width});
        corners.push_back({-half_length, -half_width});

        double min_x = center_x_m;
        double max_x = center_x_m;
        double min_y = center_y_m;
        double max_y = center_y_m;

        for (auto const & corner : corners)
        {
            double local_x = corner.first;
            double local_y = corner.second;

            double world_x = center_x_m + (local_x * cos_yaw - local_y * sin_yaw);
            double world_y = center_y_m + (local_x * sin_yaw + local_y * cos_yaw);

            min_x = std::min(min_x, world_x);
            max_x = std::max(max_x, world_x);
            min_y = std::min(min_y, world_y);
            max_y = std::max(max_y, world_y);
        }

        return AxisAlignedBoundingBox(min_x, min_y, max_x, max_y);
    }

    /**
     * Check if this AABB intersects with another AABB.
     *
     * @param other The other AABB to check against
     * @return true if the AABBs overlap, false otherwise
     */
    bool intersects(AxisAlignedBoundingBox const & other) const
    {
        return !(max_x_m < other.min_x_m ||
                 min_x_m > other.max_x_m ||
                 max_y_m < other.min_y_m ||
                 min_y_m > other.max_y_m);
    }

    /**
     * Get the center point of the AABB.
     *
     * @return A pair containing (center_x, center_y)
     */
    std::pair<double, double> center() const
    {
        return {(min_x_m + max_x_m) / 2.0, (min_y_m + max_y_m) / 2.0};
    }

    /**
     * Get the width of the AABB (extent in x direction).
     *
     * @return Width in meters
     */
    double width() const
    {
        return max_x_m - min_x_m;
    }

    /**
     * Get the height of the AABB (extent in y direction).
     *
     * @return Height in meters
     */
    double height() const
    {
        return max_y_m - min_y_m;
    }

    /**
     * Get the area of the AABB.
     *
     * @return Area in square meters
     */
    double area() const
    {
        return width() * height();
    }

    /**
     * Expand the AABB by a margin in all directions.
     *
     * @param margin_m Margin to add in meters
     * @return A new expanded AABB
     */
    AxisAlignedBoundingBox expand(double margin_m) const
    {
        return AxisAlignedBoundingBox(
            min_x_m - margin_m,
            min_y_m - margin_m,
            max_x_m + margin_m,
            max_y_m + margin_m);
    }

    /**
     * Check if a point is inside the AABB.
     *
     * @param x_m X coordinate of the point
     * @param y_m Y coordinate of the point
     * @return true if the point is inside, false otherwise
     */
    bool contains_point(double x_m, double y_m) const
    {
        return x_m >= min_x_m && x_m <= max_x_m &&
               y_m >= min_y_m && y_m <= max_y_m;
    }
}; /* end struct AxisAlignedBoundingBox */

} /* end namespace LaneZero */
