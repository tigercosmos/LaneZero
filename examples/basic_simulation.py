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
Example usage of LaneZero traffic simulation
"""

import LaneZero as lz


def main():
    print("=" * 60)
    print("LaneZero Traffic Simulation Example")
    print("=" * 60)

    # Create a simulation
    simulation = lz.Simulation()
    print("\n1. Created simulation instance")

    # Initialize a highway with 3 lanes, each 2000 meters long
    simulation.simulation_map.initialize_highway(
        num_lanes=3,
        lane_length=2000.0
    )
    lanes = simulation.simulation_map.roads()[0].lane_sections[0].lanes
    print(f"2. Initialized highway with {len(lanes)} lanes")

    # Display lane information
    for lane in lanes:
        speed_limit_info = f"{lane.speed_limit:.2f} m/s" if lane.speed_limit else "N/A"
        print(f"   Lane {lane.lane_id}: type={lane.type}, "
              f"speed limit={speed_limit_info}")

    # Create some vehicles manually
    print("\n3. Creating vehicles:")

    car1 = lz.Vehicle(
        id=1,
        type=lz.VehicleType.Car,
        position=0.0,
        velocity=25.0,
        lane_id=0,
        length=4.5,
        width=2.0
    )
    car1.acceleration_mps2 = 1.0
    print(f"   Car 1: position={car1.position_s_m:.1f}m, "
          f"velocity={car1.velocity_mps:.1f}m/s, "
          f"lane={car1.current_lane_id}")

    truck1 = lz.Vehicle(
        id=2,
        type=lz.VehicleType.Truck,
        position=100.0,
        velocity=20.0,
        lane_id=1,
        length=12.0,
        width=2.5
    )
    truck1.acceleration_mps2 = 0.5
    print(f"   Truck 1: position={truck1.position_s_m:.1f}m, "
          f"velocity={truck1.velocity_mps:.1f}m/s, "
          f"lane={truck1.current_lane_id}")

    # Add vehicles to simulation
    simulation.add_vehicle_copy(car1)
    simulation.add_vehicle_copy(truck1)

    # Spawn additional random traffic
    simulation.spawn_traffic(num_vehicles=5)
    vehicles = simulation.get_vehicles()
    print(f"\n4. Added {len(vehicles)} total vehicles to simulation")

    # Check initial collision status
    has_collision = simulation.check_collision()
    if has_collision:
        collision_status = 'COLLISION DETECTED'
    else:
        collision_status = 'No collisions'
    print(f"\n5. Initial collision check: {collision_status}")

    # Run simulation for 10 seconds with 0.1 second time steps
    print("\n6. Running simulation for 10 seconds...")
    duration = 10.0
    delta_t = 0.1
    simulation.run(duration_s=duration, delta_t_s=delta_t)

    # Get final state
    final_vehicles = simulation.get_vehicles()
    print(f"\n7. Simulation completed at time: "
          f"{simulation.current_time_s:.2f}s")
    print(f"   Number of vehicles: {len(final_vehicles)}")

    # Display final vehicle states
    print("\n8. Final vehicle states:")
    for index, vehicle in enumerate(final_vehicles[:7]):
        vtype = "Car" if vehicle.type == lz.VehicleType.Car else "Truck"
        print(f"   Vehicle {index}: {vtype}, "
              f"position={vehicle.position_s_m:.1f}m, "
              f"velocity={vehicle.velocity_mps:.1f}m/s, "
              f"lane={vehicle.current_lane_id}")

    # Final collision check
    final_collision = simulation.check_collision()
    final_status = (
        'COLLISION DETECTED' if final_collision else 'No collisions'
    )
    print(f"\n9. Final collision check: {final_status}")

    print("\n" + "=" * 60)
    print("Simulation completed successfully!")
    print("=" * 60)


if __name__ == '__main__':
    main()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
