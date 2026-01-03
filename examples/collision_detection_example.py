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

"""
Example demonstrating AABB collision detection in LaneZero.

This example shows how to:
1. Create axis-aligned bounding boxes from vehicle positions
2. Check for collisions between vehicles using AABB
3. Use AABB utility functions (expand, center, contains_point, etc.)
"""

import LaneZero as lz


def main():
    print("LaneZero AABB Collision Detection Example")
    print("=" * 60)

    print("\n1. Creating two vehicles")
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

    print(f"  Vehicle 1 at ({state1.x_m:.1f}, {state1.y_m:.1f}) m")
    print(f"  Vehicle 2 at ({state2.x_m:.1f}, {state2.y_m:.1f}) m")

    print("\n2. Getting AABBs for vehicles")
    aabb1 = vehicle1.get_aabb()
    aabb2 = vehicle2.get_aabb()
    print(
        f"  Vehicle 1 AABB: x=[{aabb1.min_x_m:.2f}, {aabb1.max_x_m:.2f}], y=[{aabb1.min_y_m:.2f}, {aabb1.max_y_m:.2f}]"
    )
    print(
        f"  Vehicle 2 AABB: x=[{aabb2.min_x_m:.2f}, {aabb2.max_x_m:.2f}], y=[{aabb2.min_y_m:.2f}, {aabb2.max_y_m:.2f}]"
    )

    print("\n3. Checking for collision")
    collision = vehicle1.check_collision_with(vehicle2)
    print(f"  Collision detected: {collision}")
    print(f"  AABBs intersect: {aabb1.intersects(aabb2)}")

    print("\n4. Moving vehicle 2 away")
    state2.x_m = 20.0
    state2.y_m = 30.0
    vehicle2.set_physics_state(state2)

    aabb2_new = vehicle2.get_aabb()
    collision_new = vehicle1.check_collision_with(vehicle2)
    print(f"  Vehicle 2 new position: ({state2.x_m:.1f}, {state2.y_m:.1f}) m")
    print(f"  Collision detected: {collision_new}")
    print(f"  AABBs intersect: {aabb1.intersects(aabb2_new)}")

    print("\n5. Demonstrating AABB utility functions")

    print("  AABB center:")
    center = aabb1.center()
    print(f"    ({center[0]:.2f}, {center[1]:.2f}) m")

    print("  AABB dimensions:")
    print(f"    Width: {aabb1.width():.2f} m")
    print(f"    Height: {aabb1.height():.2f} m")
    print(f"    Area: {aabb1.area():.2f} m²")

    print("  Expanded AABB (0.5m margin):")
    expanded = aabb1.expand(0.5)
    print(
        f"    x=[{expanded.min_x_m:.2f}, {expanded.max_x_m:.2f}], y=[{expanded.min_y_m:.2f}, {expanded.max_y_m:.2f}]"
    )

    print("  Point containment:")
    point_x, point_y = 10.0, 20.0
    contains = aabb1.contains_point(point_x, point_y)
    print(f"    Point ({point_x}, {point_y}) is inside AABB: {contains}")

    print("\n6. Creating AABB from rotated rectangle")
    import math

    aabb_rotated = lz.AxisAlignedBoundingBox.from_rotated_rectangle(
        0.0, 0.0, 4.0, 2.0, math.pi / 4.0
    )
    print("  Rectangle (4x2 m) rotated 45 degrees:")
    print(
        f"    AABB: x=[{aabb_rotated.min_x_m:.2f}, {aabb_rotated.max_x_m:.2f}], y=[{aabb_rotated.min_y_m:.2f}, {aabb_rotated.max_y_m:.2f}]"
    )

    print("\n" + "=" * 60)
    print("Example completed successfully!")


if __name__ == "__main__":
    main()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
