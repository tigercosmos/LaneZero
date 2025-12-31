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
Example usage of LaneZero OpenScenario support

This example demonstrates loading and simulating OpenScenario-like scenarios.

Features demonstrated:
- Loading scenario from JSON file
- Parsing scenario entities, actions, events, and triggers
- Initializing vehicles based on scenario configuration
- Running scenario simulation with event triggering
- Evaluating conditions (time-based, distance-based)
- Executing actions (teleport, speed, lane change)

Status:
✅ Scenario loading works perfectly (--test-load)
✅ Simulation completes successfully with correct results
✅ Entity-vehicle mapping using vehicle IDs (no dangling pointers)
⚠️  There is a segfault during program cleanup that needs investigation

The segfault occurs after all simulation work completes successfully and
does not affect the correctness of the results. It appears to be related
to object destruction order or pybind11 lifetime management.

Usage:
  --test-load: Test scenario loading only (works without issues)
  --test-sim:  Test full simulation (completes but segfaults at end)
"""

import argparse
import LaneZero as lz


def test_scenario_loading():
    print("=" * 60)
    print("LaneZero OpenScenario Loading Test")
    print("=" * 60)

    # Create a scenario instance
    scenario = lz.Scenario()
    print("\n1. Created scenario instance")

    # Load scenario from file
    scenario_file = "data/scenario/npc_cut_in.json"
    try:
        scenario.load_from_file(scenario_file)
        print(f"2. Loaded scenario from: {scenario_file}")
    except Exception as error:
        print(f"Error loading scenario: {error}")
        return

    # Display scenario metadata
    meta = scenario.meta()
    print("\n3. Scenario metadata:")
    print(f"   Name: {meta.name}")
    print(f"   Version: {meta.version}")
    if meta.description:
        print(f"   Description: {meta.description}")

    # Display entities
    entities = scenario.entities()
    print(f"\n4. Scenario entities ({len(entities)} total):")
    for entity in entities:
        print(f"   - {entity.name} ({entity.type})")
        print(
            f"     Dimensions: {entity.dimensions.length}m x {entity.dimensions.width}m x {entity.dimensions.height}m"
        )

    # Display initialization actions
    init_actions = scenario.init().actions
    print(f"\n5. Initialization actions ({len(init_actions)} total):")
    for action in init_actions:
        print(f"   - {action.type} for entity: {action.entity}")

    # Display storyboard
    storyboard = scenario.storyboard()
    print("\n6. Storyboard:")
    print(f"   Acts: {len(storyboard.acts)}")
    for act in storyboard.acts:
        print(f"   - Act: {act.name}")
        print(f"     Maneuvers: {len(act.maneuvers)}")
        for maneuver in act.maneuvers:
            print(f"       - Maneuver: {maneuver.name}")
            print(f"         Events: {len(maneuver.events)}")


def test_scenario_simulation():
    print("\n" + "=" * 60)
    print("LaneZero OpenScenario Simulation Test")
    print("=" * 60)

    # Load map directly
    map_obj = lz.Map()
    map_file = "data/map/simple_highway.json"
    try:
        map_obj.load_from_file(map_file)
        print(f"\n1. Loaded map from: {map_file}")
    except Exception as error:
        print(f"Error loading map: {error}")
        return

    # Load scenario
    scenario = lz.Scenario()
    scenario_file = "data/scenario/npc_cut_in.json"
    try:
        scenario.load_from_file(scenario_file)
        print(f"2. Loaded scenario from: {scenario_file}")
    except Exception as error:
        print(f"Error loading scenario: {error}")
        return

    # Create world state
    world_state = lz.WorldState()
    world_state.world_map = map_obj
    world_state.current_time_s = 0.0

    # Create vehicles based on scenario entities
    entities = scenario.entities()
    vehicle_id = 1
    for entity in entities:
        if entity.name == "ego":
            vehicle = lz.Vehicle(
                id=vehicle_id,
                type=lz.VehicleType.Car,
                position=10.0,
                velocity=15.0,
                lane_id=-1,
                length=entity.dimensions.length,
                width=entity.dimensions.width,
            )
            world_state.ego_vehicle = vehicle
            print(f"3. Created ego vehicle (ID: {vehicle_id})")
        else:
            vehicle = lz.Vehicle(
                id=vehicle_id,
                type=lz.VehicleType.Car,
                position=25.0,
                velocity=16.0,
                lane_id=-2,
                length=entity.dimensions.length,
                width=entity.dimensions.width,
            )
            world_state.add_vehicle(vehicle)
            print(f"4. Created NPC vehicle (ID: {vehicle_id})")
        vehicle_id += 1

    # Map entities to vehicles using vehicle IDs
    print("\n5. Mapping entities to vehicles...")
    for entity in entities:
        if entity.name == "ego":
            if world_state.ego_vehicle:
                scenario.set_entity_vehicle_id(entity.name, world_state.ego_vehicle.id)
        else:
            for i in range(world_state.get_vehicle_count()):
                npc_vehicle = world_state.get_vehicle(i)
                if npc_vehicle:
                    scenario.set_entity_vehicle_id(entity.name, npc_vehicle.id)
                    break

    # Initialize scenario
    print("\n6. Initializing scenario...")
    scenario.initialize(world_state)

    # Print initial state
    print("\n7. Initial vehicle states:")
    if world_state.ego_vehicle:
        ego = world_state.ego_vehicle
        print(
            f"   Ego: position={ego.position_s_m:.1f}m, velocity={ego.velocity_mps:.1f}m/s, lane={ego.current_lane_id}"
        )
    for i in range(world_state.get_vehicle_count()):
        vehicle = world_state.get_vehicle(i)
        if vehicle:
            print(
                f"   NPC {vehicle.id}: position={vehicle.position_s_m:.1f}m, velocity={vehicle.velocity_mps:.1f}m/s, lane={vehicle.current_lane_id}"
            )

    # Simulate for a few steps
    print("\n8. Running simulation steps:")
    delta_t = 0.1
    max_time = 5.0

    while world_state.current_time_s < max_time and not scenario.is_complete(
        world_state
    ):
        scenario.update(world_state)
        world_state.current_time_s += delta_t

        if int(world_state.current_time_s * 10) % 10 == 0:
            print(f"   Time: {world_state.current_time_s:.1f}s")
            if world_state.ego_vehicle:
                ego = world_state.ego_vehicle
                print(
                    f"     Ego: position={ego.position_s_m:.1f}m, lane={ego.current_lane_id}"
                )

    print("\n9. Simulation completed")

    print("10. Cleaning up entity-vehicle mappings...")
    scenario.clear_entity_vehicles()
    print("11. Entity-vehicle mappings cleared")


def main(args):
    if args.test_load:
        test_scenario_loading()

    if args.test_sim:
        test_scenario_simulation()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LaneZero OpenScenario Example")
    parser.add_argument(
        "--test-load",
        action="store_true",
        help="Test scenario loading",
    )
    parser.add_argument(
        "--test-sim",
        action="store_true",
        help="Test scenario simulation",
    )

    args = parser.parse_args()

    if not args.test_load and not args.test_sim:
        args.test_load = True
        args.test_sim = True

    main(args)

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
