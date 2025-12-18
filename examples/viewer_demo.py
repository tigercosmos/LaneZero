#!/usr/bin/env python3.14
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
Quick demo showing viewer capabilities.
"""

import os
import sys
import LaneZero as lz


def main():
    if not lz.viewer.enable:
        print("ERROR: Viewer is not available. Build with Qt support.")
        sys.exit(1)

    print("=" * 60)
    print("LaneZero Viewer Quick Demo")
    print("=" * 60)

    map_path = os.path.join(
        os.path.dirname(__file__), "..", "data", "map", "two_road.json"
    )
    map_path = os.path.abspath(map_path)

    simulation_map = lz.Map()
    simulation_map.load_from_file(map_path)
    print(f"\nLoaded map: {map_path}")

    simulation = lz.Simulation()
    simulation.simulation_map = simulation_map

    vehicle1 = lz.Vehicle(1, lz.VehicleType.Car, 5.0, 10.0, 1, 4.5, 2.0)
    vehicle1.acceleration_mps2 = 2.0
    vehicle1.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)

    vehicle2 = lz.Vehicle(2, lz.VehicleType.Car, 20.0, 12.0, -1, 4.5, 2.0)
    vehicle2.acceleration_mps2 = 1.5
    vehicle2.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)

    vehicle3 = lz.Vehicle(3, lz.VehicleType.Truck, 35.0, 8.0, 1, 12.0, 2.5)
    vehicle3.acceleration_mps2 = 0.8
    vehicle3.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)

    simulation.add_vehicle_copy(vehicle1)
    simulation.add_vehicle_copy(vehicle2)
    simulation.add_vehicle_copy(vehicle3)

    print("Added 3 vehicles (2 cars, 1 truck)")
    print("\nStarting viewer...")
    print("Watch the vehicles move along the roads!")
    print("Close the window to exit.")

    viewer = lz.viewer.SimulationViewer(simulation, title="LaneZero Viewer Demo")
    viewer.run(duration_s=20.0, delta_t_s=0.1)

    print("\nDemo completed!")


if __name__ == "__main__":
    main()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
