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
Example demonstrating lane keeping simulation with planning-based flow.

This example shows the new simulation architecture where:
1. An ego vehicle is controlled by a planning system
2. Other vehicles use simple control logic
3. The simulation follows: get_world_state -> update_goal -> plan -> apply

Usage:
    python lane_keeping_example.py              # Run without viewer
    python lane_keeping_example.py --viewer     # Run with viewer
"""

import argparse
import LaneZero as lz


def main():
    parser = argparse.ArgumentParser(description="LaneZero Lane Keeping Example")
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Enable visualization of the simulation",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("LaneZero Lane Keeping Simulation Example")
    print("=" * 60)

    simulation = lz.Simulation()
    print("\n1. Created simulation instance")

    simulation.simulation_map.initialize_highway(num_lanes=3, lane_length=2000.0)
    lanes = simulation.simulation_map.roads()[0].lane_sections[0].lanes
    print(f"2. Initialized highway with {len(lanes)} lanes")

    print("\n3. Creating ego vehicle (controlled by planner):")
    ego_vehicle = lz.Vehicle(
        id=0,
        type=lz.VehicleType.Car,
        position=5.0,
        velocity=25.0,
        lane_id=1,
        length=4.5,
        width=2.0,
    )
    print(
        f"   Ego vehicle: position={ego_vehicle.position_s_m:.1f}m, "
        f"velocity={ego_vehicle.velocity_mps:.1f}m/s, "
        f"lane={ego_vehicle.current_lane_id}"
    )

    ego_vehicle.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)
    simulation.set_ego_vehicle(ego_vehicle)
    print("   Set as ego vehicle for planning-based control")
    print("   Physics engine: BicycleModel enabled")

    print("\n4. Spawning surrounding traffic:")
    simulation.spawn_traffic(num_vehicles=5)
    vehicles = simulation.get_vehicles()
    for vehicle in vehicles:
        if vehicle.id != 0:
            vehicle.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)

    print(f"   Total vehicles in simulation: {len(vehicles)}")
    print("   (Traffic vehicles use simple control logic)")
    print("   Physics engine: BicycleModel enabled for all vehicles")

    print("\n5. Getting initial world state:")
    world_state = simulation.get_world_state()
    print(f"   Current time: {world_state.current_time_s}s")
    print(
        f"   Ego vehicle in world state: "
        f"position={world_state.ego_vehicle.position_s_m:.1f}m"
    )
    print(f"   Other vehicles: {len(world_state.get_other_vehicles())}")

    duration = 10.0
    delta_t = 1 / 60.0

    if args.viewer:
        print(f"\n6. Starting simulation with viewer for {duration}s...")
        if not lz.viewer.enable:
            print("   ERROR: Viewer is not available. Build with Qt support.")
            return

        viewer = lz.viewer.SimulationViewer(simulation, title="Lane Keeping Simulation")
        print("   Close the viewer window to stop the simulation.")
        print("   Watch the ego vehicle maintain its lane!")
        viewer.run(duration_s=duration, delta_t_s=delta_t)
    else:
        print(f"\n6. Running simulation for {duration} seconds...")
        print("   Ego vehicle uses planning system for lane keeping")
        simulation.run(duration_s=duration, delta_t_s=delta_t)

    final_vehicles = simulation.get_vehicles()
    print(f"\n7. Simulation completed at time: {simulation.current_time_s:.2f}s")

    print("\n8. Final vehicle states:")
    for idx, vehicle in enumerate(final_vehicles[:6]):
        vehicle_type = "Car" if vehicle.type == lz.VehicleType.Car else "Truck"
        ego_marker = " (EGO)" if vehicle.id == 0 else ""
        print(
            f"   Vehicle {vehicle.id}: {vehicle_type}{ego_marker}, "
            f"position={vehicle.position_s_m:.1f}m, "
            f"velocity={vehicle.velocity_mps:.1f}m/s, "
            f"lane={vehicle.current_lane_id}"
        )

    ego_final = None
    for vehicle in final_vehicles:
        if vehicle.id == 0:
            ego_final = vehicle
            break

    if ego_final:
        print("\n9. Ego vehicle performance:")
        print(f"   Initial lane: {ego_vehicle.current_lane_id}")
        print(f"   Final lane: {ego_final.current_lane_id}")
        print(
            f"   Distance traveled: "
            f"{ego_final.position_s_m - ego_vehicle.position_s_m:.1f}m"
        )
        print(
            f"   Lane maintained: "
            f"{'YES' if ego_final.current_lane_id == ego_vehicle.current_lane_id else 'NO'}"
        )

    has_collision = simulation.check_collision()
    collision_status = "COLLISION DETECTED" if has_collision else "No collisions"
    print(f"\n10. Collision check: {collision_status}")

    print("\n" + "=" * 60)
    print("Lane keeping simulation completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
