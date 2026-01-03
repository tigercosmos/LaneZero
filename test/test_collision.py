# -*- coding: UTF-8 -*-
#
# Copyright (c) 2025, LaneZero Contributors
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import LaneZero as lz


def test_aabb_default_constructor():
    aabb = lz.AxisAlignedBoundingBox()
    assert aabb.min_x_m == 0.0
    assert aabb.min_y_m == 0.0
    assert aabb.max_x_m == 0.0
    assert aabb.max_y_m == 0.0


def test_aabb_parameterized_constructor():
    aabb = lz.AxisAlignedBoundingBox(1.0, 2.0, 3.0, 4.0)
    assert aabb.min_x_m == 1.0
    assert aabb.min_y_m == 2.0
    assert aabb.max_x_m == 3.0
    assert aabb.max_y_m == 4.0


def test_aabb_from_rotated_rectangle_no_rotation():
    aabb = lz.AxisAlignedBoundingBox.from_rotated_rectangle(10.0, 20.0, 4.0, 2.0, 0.0)
    assert abs(aabb.min_x_m - 8.0) < 1e-6
    assert abs(aabb.max_x_m - 12.0) < 1e-6
    assert abs(aabb.min_y_m - 19.0) < 1e-6
    assert abs(aabb.max_y_m - 21.0) < 1e-6


def test_aabb_from_rotated_rectangle_90_degrees():
    aabb = lz.AxisAlignedBoundingBox.from_rotated_rectangle(
        10.0, 20.0, 4.0, 2.0, math.pi / 2.0
    )
    assert abs(aabb.min_x_m - 9.0) < 1e-6
    assert abs(aabb.max_x_m - 11.0) < 1e-6
    assert abs(aabb.min_y_m - 18.0) < 1e-6
    assert abs(aabb.max_y_m - 22.0) < 1e-6


def test_aabb_from_rotated_rectangle_45_degrees():
    aabb = lz.AxisAlignedBoundingBox.from_rotated_rectangle(
        0.0, 0.0, 4.0, 2.0, math.pi / 4.0
    )
    assert abs(aabb.min_x_m - (-2.12132)) < 0.001
    assert abs(aabb.max_x_m - 2.12132) < 0.001
    assert abs(aabb.min_y_m - (-2.12132)) < 0.001
    assert abs(aabb.max_y_m - 2.12132) < 0.001


def test_aabb_intersects_true():
    aabb1 = lz.AxisAlignedBoundingBox(0.0, 0.0, 2.0, 2.0)
    aabb2 = lz.AxisAlignedBoundingBox(1.0, 1.0, 3.0, 3.0)
    assert aabb1.intersects(aabb2)
    assert aabb2.intersects(aabb1)


def test_aabb_intersects_false():
    aabb1 = lz.AxisAlignedBoundingBox(0.0, 0.0, 2.0, 2.0)
    aabb2 = lz.AxisAlignedBoundingBox(3.0, 3.0, 5.0, 5.0)
    assert not aabb1.intersects(aabb2)
    assert not aabb2.intersects(aabb1)


def test_aabb_intersects_edge_case():
    aabb1 = lz.AxisAlignedBoundingBox(0.0, 0.0, 2.0, 2.0)
    aabb2 = lz.AxisAlignedBoundingBox(2.0, 2.0, 4.0, 4.0)
    assert aabb1.intersects(aabb2)


def test_aabb_center():
    aabb = lz.AxisAlignedBoundingBox(1.0, 2.0, 5.0, 8.0)
    center = aabb.center()
    assert center[0] == 3.0
    assert center[1] == 5.0


def test_aabb_width_and_height():
    aabb = lz.AxisAlignedBoundingBox(1.0, 2.0, 5.0, 8.0)
    assert aabb.width() == 4.0
    assert aabb.height() == 6.0


def test_aabb_area():
    aabb = lz.AxisAlignedBoundingBox(1.0, 2.0, 5.0, 8.0)
    assert aabb.area() == 24.0


def test_aabb_expand():
    aabb = lz.AxisAlignedBoundingBox(1.0, 2.0, 3.0, 4.0)
    expanded = aabb.expand(0.5)
    assert expanded.min_x_m == 0.5
    assert expanded.min_y_m == 1.5
    assert expanded.max_x_m == 3.5
    assert expanded.max_y_m == 4.5


def test_aabb_contains_point_true():
    aabb = lz.AxisAlignedBoundingBox(0.0, 0.0, 4.0, 4.0)
    assert aabb.contains_point(2.0, 2.0)
    assert aabb.contains_point(0.0, 0.0)
    assert aabb.contains_point(4.0, 4.0)


def test_aabb_contains_point_false():
    aabb = lz.AxisAlignedBoundingBox(0.0, 0.0, 4.0, 4.0)
    assert not aabb.contains_point(-1.0, 2.0)
    assert not aabb.contains_point(2.0, 5.0)
    assert not aabb.contains_point(5.0, 5.0)


def test_vehicle_get_aabb():
    vehicle = lz.Vehicle(1, lz.VehicleType.Car, 100.0, 30.0, 1, 4.5, 2.0)

    physics_state = lz.RigidBodyState()
    physics_state.x_m = 10.0
    physics_state.y_m = 20.0
    physics_state.yaw_rad = 0.0
    vehicle.set_physics_state(physics_state)

    aabb = vehicle.get_aabb()
    assert abs(aabb.min_x_m - 7.75) < 1e-6
    assert abs(aabb.max_x_m - 12.25) < 1e-6
    assert abs(aabb.min_y_m - 19.0) < 1e-6
    assert abs(aabb.max_y_m - 21.0) < 1e-6


def test_vehicle_collision_detection():
    vehicle1 = lz.Vehicle(1, lz.VehicleType.Car, 100.0, 30.0, 1, 4.5, 2.0)
    vehicle2 = lz.Vehicle(2, lz.VehicleType.Car, 105.0, 30.0, 1, 4.5, 2.0)

    state1 = lz.RigidBodyState()
    state1.x_m = 10.0
    state1.y_m = 20.0
    state1.yaw_rad = 0.0
    vehicle1.set_physics_state(state1)

    state2 = lz.RigidBodyState()
    state2.x_m = 12.0
    state2.y_m = 20.0
    state2.yaw_rad = 0.0
    vehicle2.set_physics_state(state2)

    assert vehicle1.check_collision_with(vehicle2)
    assert vehicle2.check_collision_with(vehicle1)


def test_vehicle_no_collision():
    vehicle1 = lz.Vehicle(1, lz.VehicleType.Car, 100.0, 30.0, 1, 4.5, 2.0)
    vehicle2 = lz.Vehicle(2, lz.VehicleType.Car, 105.0, 30.0, 1, 4.5, 2.0)

    state1 = lz.RigidBodyState()
    state1.x_m = 10.0
    state1.y_m = 20.0
    state1.yaw_rad = 0.0
    vehicle1.set_physics_state(state1)

    state2 = lz.RigidBodyState()
    state2.x_m = 20.0
    state2.y_m = 30.0
    state2.yaw_rad = 0.0
    vehicle2.set_physics_state(state2)

    assert not vehicle1.check_collision_with(vehicle2)
    assert not vehicle2.check_collision_with(vehicle1)


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
