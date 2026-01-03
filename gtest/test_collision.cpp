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

#include <gtest/gtest.h>
#include <LaneZero/collision/CollisionDetection.h>
#include <cmath>

using namespace LaneZero;

TEST(AxisAlignedBoundingBoxTest, DefaultConstructor)
{
    AxisAlignedBoundingBox aabb;
    EXPECT_DOUBLE_EQ(aabb.min_x_m, 0.0);
    EXPECT_DOUBLE_EQ(aabb.min_y_m, 0.0);
    EXPECT_DOUBLE_EQ(aabb.max_x_m, 0.0);
    EXPECT_DOUBLE_EQ(aabb.max_y_m, 0.0);
}

TEST(AxisAlignedBoundingBoxTest, ParameterizedConstructor)
{
    AxisAlignedBoundingBox aabb(1.0, 2.0, 3.0, 4.0);
    EXPECT_DOUBLE_EQ(aabb.min_x_m, 1.0);
    EXPECT_DOUBLE_EQ(aabb.min_y_m, 2.0);
    EXPECT_DOUBLE_EQ(aabb.max_x_m, 3.0);
    EXPECT_DOUBLE_EQ(aabb.max_y_m, 4.0);
}

TEST(AxisAlignedBoundingBoxTest, FromRotatedRectangleNoRotation)
{
    AxisAlignedBoundingBox aabb = AxisAlignedBoundingBox::from_rotated_rectangle(
        10.0, 20.0, 4.0, 2.0, 0.0);

    EXPECT_NEAR(aabb.min_x_m, 8.0, 1e-6);
    EXPECT_NEAR(aabb.max_x_m, 12.0, 1e-6);
    EXPECT_NEAR(aabb.min_y_m, 19.0, 1e-6);
    EXPECT_NEAR(aabb.max_y_m, 21.0, 1e-6);
}

TEST(AxisAlignedBoundingBoxTest, FromRotatedRectangle90Degrees)
{
    AxisAlignedBoundingBox aabb = AxisAlignedBoundingBox::from_rotated_rectangle(
        10.0, 20.0, 4.0, 2.0, M_PI / 2.0);

    EXPECT_NEAR(aabb.min_x_m, 9.0, 1e-6);
    EXPECT_NEAR(aabb.max_x_m, 11.0, 1e-6);
    EXPECT_NEAR(aabb.min_y_m, 18.0, 1e-6);
    EXPECT_NEAR(aabb.max_y_m, 22.0, 1e-6);
}

TEST(AxisAlignedBoundingBoxTest, FromRotatedRectangle45Degrees)
{
    AxisAlignedBoundingBox aabb = AxisAlignedBoundingBox::from_rotated_rectangle(
        0.0, 0.0, 4.0, 2.0, M_PI / 4.0);

    EXPECT_NEAR(aabb.min_x_m, -2.12132, 0.001);
    EXPECT_NEAR(aabb.max_x_m, 2.12132, 0.001);
    EXPECT_NEAR(aabb.min_y_m, -2.12132, 0.001);
    EXPECT_NEAR(aabb.max_y_m, 2.12132, 0.001);
}

TEST(AxisAlignedBoundingBoxTest, IntersectsTrue)
{
    AxisAlignedBoundingBox aabb1(0.0, 0.0, 2.0, 2.0);
    AxisAlignedBoundingBox aabb2(1.0, 1.0, 3.0, 3.0);
    EXPECT_TRUE(aabb1.intersects(aabb2));
    EXPECT_TRUE(aabb2.intersects(aabb1));
}

TEST(AxisAlignedBoundingBoxTest, IntersectsFalse)
{
    AxisAlignedBoundingBox aabb1(0.0, 0.0, 2.0, 2.0);
    AxisAlignedBoundingBox aabb2(3.0, 3.0, 5.0, 5.0);
    EXPECT_FALSE(aabb1.intersects(aabb2));
    EXPECT_FALSE(aabb2.intersects(aabb1));
}

TEST(AxisAlignedBoundingBoxTest, IntersectsEdgeCase)
{
    AxisAlignedBoundingBox aabb1(0.0, 0.0, 2.0, 2.0);
    AxisAlignedBoundingBox aabb2(2.0, 2.0, 4.0, 4.0);
    EXPECT_TRUE(aabb1.intersects(aabb2));
}

TEST(AxisAlignedBoundingBoxTest, Center)
{
    AxisAlignedBoundingBox aabb(1.0, 2.0, 5.0, 8.0);
    auto center = aabb.center();
    EXPECT_DOUBLE_EQ(center.first, 3.0);
    EXPECT_DOUBLE_EQ(center.second, 5.0);
}

TEST(AxisAlignedBoundingBoxTest, WidthAndHeight)
{
    AxisAlignedBoundingBox aabb(1.0, 2.0, 5.0, 8.0);
    EXPECT_DOUBLE_EQ(aabb.width(), 4.0);
    EXPECT_DOUBLE_EQ(aabb.height(), 6.0);
}

TEST(AxisAlignedBoundingBoxTest, Area)
{
    AxisAlignedBoundingBox aabb(1.0, 2.0, 5.0, 8.0);
    EXPECT_DOUBLE_EQ(aabb.area(), 24.0);
}

TEST(AxisAlignedBoundingBoxTest, Expand)
{
    AxisAlignedBoundingBox aabb(1.0, 2.0, 3.0, 4.0);
    AxisAlignedBoundingBox expanded = aabb.expand(0.5);
    EXPECT_DOUBLE_EQ(expanded.min_x_m, 0.5);
    EXPECT_DOUBLE_EQ(expanded.min_y_m, 1.5);
    EXPECT_DOUBLE_EQ(expanded.max_x_m, 3.5);
    EXPECT_DOUBLE_EQ(expanded.max_y_m, 4.5);
}

TEST(AxisAlignedBoundingBoxTest, ContainsPointTrue)
{
    AxisAlignedBoundingBox aabb(0.0, 0.0, 4.0, 4.0);
    EXPECT_TRUE(aabb.contains_point(2.0, 2.0));
    EXPECT_TRUE(aabb.contains_point(0.0, 0.0));
    EXPECT_TRUE(aabb.contains_point(4.0, 4.0));
}

TEST(AxisAlignedBoundingBoxTest, ContainsPointFalse)
{
    AxisAlignedBoundingBox aabb(0.0, 0.0, 4.0, 4.0);
    EXPECT_FALSE(aabb.contains_point(-1.0, 2.0));
    EXPECT_FALSE(aabb.contains_point(2.0, 5.0));
    EXPECT_FALSE(aabb.contains_point(5.0, 5.0));
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
