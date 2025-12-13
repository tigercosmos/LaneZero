#!/usr/bin/env python3
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
Example demonstrating the use of two_road.json map with LaneZero simulation.

This example loads a map with two connected roads (R1 and R2) and simulates
vehicle traffic traveling through both road segments.

Usage:
    python two_road_example.py              # Run without viewer
    python two_road_example.py --viewer     # Run with viewer
"""

import os
import sys
import argparse
import LaneZero as lz


def main():
    parser = argparse.ArgumentParser(description="LaneZero Two-Road Map Example")
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Enable visualization of the simulation",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("LaneZero Two-Road Map Example")
    print("=" * 60)

    map_path = os.path.join(
        os.path.dirname(__file__), "..", "data", "map", "two_road.json"
    )
    map_path = os.path.abspath(map_path)
    print(f"\n1. Loading map from: {map_path}")

    if not os.path.exists(map_path):
        print(f"Error: Map file not found at {map_path}")
        return

    simulation_map = lz.Map()
    simulation_map.load_from_file(map_path)
    print("   Map loaded successfully")

    roads = simulation_map.roads()
    print(f"\n2. Map contains {len(roads)} roads:")
    for road in roads:
        print(
            f"   Road {road.id}: length={road.length:.1f}m, "
            f"speed_limit={road.speed_limit if road.speed_limit else 'N/A'}"
        )

    simulation = lz.Simulation()
    simulation.simulation_map = simulation_map
    print("\n3. Created simulation with loaded map")

    print("\n4. Creating vehicles on Road R1:")
    vehicle1 = lz.Vehicle(
        id=1,
        type=lz.VehicleType.Car,
        position=5.0,
        velocity=10.0,
        lane_id=1,
        length=4.5,
        width=2.0,
    )
    vehicle1.acceleration_mps2 = 2.0
    print(
        f"   Vehicle 1: Car at position={vehicle1.position_s_m:.1f}m, "
        f"velocity={vehicle1.velocity_mps:.1f}m/s"
    )

    vehicle2 = lz.Vehicle(
        id=2,
        type=lz.VehicleType.Car,
        position=20.0,
        velocity=12.0,
        lane_id=-1,
        length=4.5,
        width=2.0,
    )
    vehicle2.acceleration_mps2 = 1.5
    print(
        f"   Vehicle 2: Car at position={vehicle2.position_s_m:.1f}m, "
        f"velocity={vehicle2.velocity_mps:.1f}m/s"
    )

    vehicle3 = lz.Vehicle(
        id=3,
        type=lz.VehicleType.Truck,
        position=35.0,
        velocity=8.0,
        lane_id=1,
        length=12.0,
        width=2.5,
    )
    vehicle3.acceleration_mps2 = 0.8
    print(
        f"   Vehicle 3: Truck at position={vehicle3.position_s_m:.1f}m, "
        f"velocity={vehicle3.velocity_mps:.1f}m/s"
    )

    simulation.add_vehicle_copy(vehicle1)
    simulation.add_vehicle_copy(vehicle2)
    simulation.add_vehicle_copy(vehicle3)

    initial_vehicles = simulation.get_vehicles()
    print(f"\n5. Added {len(initial_vehicles)} vehicles to simulation")

    has_collision = simulation.check_collision()
    collision_status = "COLLISION DETECTED" if has_collision else "No collisions"
    print(f"\n6. Initial collision check: {collision_status}")

    duration = 15.0
    delta_t = 1.0 / 60.0
    if args.viewer:
        print("\n7. Starting simulation with viewer...")
        if not lz.viewer.enable:
            print("   ERROR: Viewer is not available. Build with Qt support.")
            sys.exit(1)

        viewer = lz.viewer.SimulationViewer(
            simulation, title="Two-Road Simulation Viewer"
        )

        print(f"   Running simulation for {duration} seconds with dt={delta_t}s")
        print("   Close the viewer window to stop the simulation.")
        viewer.run(duration_s=duration, delta_t_s=delta_t)
    else:
        print("\n7. Running simulation for 15 seconds...")
        simulation.run(duration_s=duration, delta_t_s=delta_t)

    final_vehicles = simulation.get_vehicles()
    print(f"\n8. Simulation completed at time: {simulation.current_time_s:.2f}s")

    print("\n9. Final vehicle states:")
    for vehicle in final_vehicles:
        vehicle_type = "Car" if vehicle.type == lz.VehicleType.Car else "Truck"
        print(
            f"   Vehicle {vehicle.id}: {vehicle_type}, "
            f"position={vehicle.position_s_m:.1f}m, "
            f"velocity={vehicle.velocity_mps:.1f}m/s, "
            f"lane={vehicle.current_lane_id}"
        )

    final_collision = simulation.check_collision()
    final_status = "COLLISION DETECTED" if final_collision else "No collisions"
    print(f"\n10. Final collision check: {final_status}")

    print("\n" + "=" * 60)
    print("Two-road simulation completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
