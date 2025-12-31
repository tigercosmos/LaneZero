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
Cut-In Scenario Planner Example

Demonstrates how the BehaviorPlanner and MotionPlanner respond to a cut-in
scenario where an NPC vehicle cuts in front of the ego vehicle.

The planner detects the cut-in and automatically:
- Detects vehicles cutting into the same lane
- Calculates safe distance thresholds
- Reduces speed to maintain safe following distance
- Plans smooth deceleration trajectory

Usage:
    python cut_in_planner_example.py              # Run without viewer
    python cut_in_planner_example.py --viewer     # Run with viewer
"""

import argparse
import sys
import LaneZero as lz


def run_cut_in_with_planner(use_viewer=False):
    print("=" * 70)
    print("Cut-In Scenario with Autonomous Planner Response")
    print("=" * 70)

    simulation = lz.Simulation()
    map_file = "data/map/simple_highway.json"
    try:
        simulation.simulation_map.load_from_file(map_file)
        print(f"\n1. Loaded map: {map_file}")
    except Exception as error:
        print(f"Error loading map: {error}")
        return

    scenario = lz.Scenario()
    scenario_file = "data/scenario/npc_cut_in.json"
    try:
        scenario.load_from_file(scenario_file)
        print(f"2. Loaded scenario: {scenario_file}")
    except Exception as error:
        print(f"Error loading scenario: {error}")
        return

    entities = scenario.entities()

    print("\n3. Creating vehicles based on scenario...")
    for entity in entities:
        if entity.name == "ego":
            ego_vehicle = lz.Vehicle(
                id=1,
                type=lz.VehicleType.Car,
                position=10.0,
                velocity=20.0,
                lane_id=-1,
                length=entity.dimensions.length,
                width=entity.dimensions.width,
            )
            ego_vehicle.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)
            simulation.set_ego_vehicle(ego_vehicle)
            scenario.set_entity_vehicle_id(entity.name, ego_vehicle.id)
            print(
                f"   Ego (ID {ego_vehicle.id}): position={ego_vehicle.position_s_m:.1f}m, "
                f"velocity={ego_vehicle.velocity_mps:.1f}m/s, lane={ego_vehicle.current_lane_id}"
            )
        else:
            npc_vehicle = lz.Vehicle(
                id=2,
                type=lz.VehicleType.Car,
                position=25.0,
                velocity=15.0,
                lane_id=-2,
                length=entity.dimensions.length,
                width=entity.dimensions.width,
            )
            npc_vehicle.acceleration_mps2 = 0.0
            npc_vehicle.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)
            simulation.add_vehicle_copy(npc_vehicle)
            scenario.set_entity_vehicle_id(entity.name, npc_vehicle.id)
            print(
                f"   NPC (ID {npc_vehicle.id}): position={npc_vehicle.position_s_m:.1f}m, "
                f"velocity={npc_vehicle.velocity_mps:.1f}m/s, lane={npc_vehicle.current_lane_id}"
            )

    print("   (NPC starts 15m ahead in lane -2, ego traveling faster will catch up)")

    print("\n4. Initializing scenario and planners...")
    world_state = simulation.get_world_state()
    scenario.initialize(world_state)

    behavior_planner = lz.BehaviorPlanner()
    motion_planner = lz.MotionPlanner()

    print("\n5. Running simulation with planner response:")
    print("-" * 70)
    delta_t = 0.1
    max_time = 15.0

    lane_change_triggered = False
    lane_change_start_time = 0.0
    lane_change_duration = 3.0
    lane_change_completed = False

    if use_viewer:
        if not lz.viewer.enable:
            print("ERROR: Viewer is not available. Build with Qt support.")
            sys.exit(1)

        print("Starting viewer...")
        print("Watch the NPC cut in and the ego vehicle respond!")
        print("Close the viewer window to stop the simulation.")
        print(
            "\nNote: Manual lane change will be triggered when NPC gets within 12m of ego"
        )

        vehicles_list = simulation.get_vehicles()
        npc_vehicle = None
        for v in vehicles_list:
            if v.id == 2:
                npc_vehicle = v
                break

        if not npc_vehicle:
            print("ERROR: NPC vehicle not found!")
            sys.exit(1)

        npc_lane_change_triggered = False
        npc_lane_change_start_time = 0.0
        npc_lane_change_completed = False
        npc_position = npc_vehicle.position_s_m
        npc_old_lane = -2
        planner_step_counter = 0
        lane_change_duration = 3.0

        def viewer_step_callback(sim, dt):
            nonlocal \
                npc_lane_change_triggered, \
                npc_lane_change_start_time, \
                npc_lane_change_completed
            nonlocal \
                npc_position, \
                npc_old_lane, \
                planner_step_counter, \
                lane_change_duration

            current_time = sim.current_time_s

            vehicles = sim.get_vehicles()
            ego_vehicle = None
            npc_vehicle_current = None
            for v in vehicles:
                if v.id == 1:
                    ego_vehicle = v
                elif v.id == 2:
                    npc_vehicle_current = v

            if not ego_vehicle or not npc_vehicle_current:
                sim.step(dt)
                return

            npc_position += npc_vehicle.velocity_mps * dt

            if npc_lane_change_completed:
                sim.set_vehicle_lane(2, -1)
                sim.set_vehicle_position(2, npc_position)
                sim.set_vehicle_lateral_offset(2, 0.0)
            elif npc_lane_change_triggered:
                elapsed = current_time - npc_lane_change_start_time
                progress = min(elapsed / lane_change_duration, 1.0)

                lane_width = 3.5
                lateral_offset = lane_width * progress

                sim.set_vehicle_position(2, npc_position)
                sim.set_vehicle_lateral_offset(2, lateral_offset)

                if progress >= 1.0:
                    sim.set_vehicle_lane(2, -1)
                    sim.set_vehicle_lateral_offset(2, 0.0)
                    npc_lane_change_completed = True
                    print(
                        f"\n>>> NPC completed smooth lane change to lane -1 at {current_time:.1f}s\n"
                    )
            else:
                sim.set_vehicle_position(2, npc_position)

            distance = abs(ego_vehicle.position_s_m - npc_position)

            if int(current_time * 10) % 5 == 0:
                print(
                    f"Time {current_time:.1f}s: Ego(lane={ego_vehicle.current_lane_id}, pos={ego_vehicle.position_s_m:.1f}m, vel={ego_vehicle.velocity_mps:.1f}m/s) NPC(lane={npc_vehicle_current.current_lane_id}, pos={npc_vehicle_current.position_s_m:.1f}m, lateral_offset={npc_vehicle_current.lateral_offset_m:.2f}m) dist={distance:.1f}m"
                )

            if (
                not npc_lane_change_triggered
                and npc_vehicle_current.position_s_m > ego_vehicle.position_s_m
            ):
                if distance <= 12.0:
                    npc_lane_change_triggered = True
                    npc_lane_change_start_time = current_time
                    print(
                        f"\n>>> LANE CHANGE TRIGGERED at {current_time:.1f}s (NPC ahead at {npc_vehicle_current.position_s_m:.1f}m, ego at {ego_vehicle.position_s_m:.1f}m, distance: {distance:.1f}m)\n"
                    )

            planner_step_counter += 1

            sim.step(dt)

            fresh_vehicles = sim.get_vehicles()
            for fv in fresh_vehicles:
                if fv.id == 2:
                    if int(current_time * 10) % 10 == 0:
                        print(
                            f">>> After sim.step: NPC lane={fv.current_lane_id}, pos={fv.position_s_m:.1f}m"
                        )
                    break

            if planner_step_counter >= 5:
                planner_step_counter = 0

                if npc_lane_change_triggered and not npc_lane_change_completed:
                    sim.set_vehicle_lane(2, -1)

                fresh_state = sim.get_world_state()
                goal = scenario.update_goal(fresh_state)
                behavior_decision = behavior_planner.plan(fresh_state, goal)
                trajectory = motion_planner.plan(behavior_decision, dt)

                if npc_lane_change_triggered and not npc_lane_change_completed:
                    sim.set_vehicle_lane(2, -2)

                if trajectory:
                    if len(trajectory.velocities_mps) > 0:
                        target_velocity = trajectory.velocities_mps[0]

                        state_str = "KeepLane"
                        if behavior_decision.cut_in_detected:
                            state_str = (
                                "Emergency"
                                if behavior_decision.safe_distance_m <= 5.0
                                else "CutInResponse"
                            )

                        fresh_ego = [v for v in sim.get_vehicles() if v.id == 1]
                        if fresh_ego:
                            current_velocity = fresh_ego[0].velocity_mps
                            print(
                                f">>> PLANNER: {state_str}! Target: {target_velocity:.1f}m/s, Current: {current_velocity:.1f}m/s"
                            )
                            sim.set_vehicle_velocity(1, target_velocity)

        viewer = lz.viewer.SimulationViewer(
            simulation, title="Cut-In Scenario - Autonomous Planner Response"
        )

        print("\n" + "=" * 70)
        print("VIEWER INSTRUCTIONS:")
        print("- Red vehicle = Ego (lane -1, left lane)")
        print("- Blue vehicle = NPC (starts in lane -2, right lane)")
        print("- Watch for NPC to move from right lane to left lane around 1.8s")
        print("=" * 70 + "\n")

        viewer.run(
            duration_s=max_time, delta_t_s=0.05, step_callback=viewer_step_callback
        )

        print("\n6. Simulation completed via viewer")
    else:
        print("Note: Manual lane change triggered when NPC gets within 10m of ego.")
        print("Running simulation with planner behavior...")

        vehicles_list = simulation.get_vehicles()
        npc_vehicle = None
        for v in vehicles_list:
            if v.id == 2:
                npc_vehicle = v
                break

        if npc_vehicle:
            print(f"Found NPC vehicle (ID {npc_vehicle.id})")
        else:
            print("WARNING: NPC vehicle not found!")

        num_steps = int(max_time / delta_t)
        for step in range(num_steps):
            current_time = step * delta_t

            if npc_vehicle:
                npc_vehicle.position_s_m += npc_vehicle.velocity_mps * delta_t

            world_state = simulation.get_world_state()

            if not lane_change_triggered and npc_vehicle:
                ego = world_state.ego_vehicle
                if ego:
                    distance = abs(npc_vehicle.position_s_m - ego.position_s_m)
                    if step == 0:
                        print(
                            f"Initial: Ego={ego.position_s_m:.1f}m NPC={npc_vehicle.position_s_m:.1f}m distance={distance:.1f}m"
                        )
                    if (
                        distance <= 12.0
                        and npc_vehicle.current_lane_id == -2
                        and npc_vehicle.position_s_m > ego.position_s_m
                    ):
                        lane_change_triggered = True
                        lane_change_start_time = current_time
                        print(
                            f"\n>>> Lane change triggered at {current_time:.1f}s (NPC ahead at {npc_vehicle.position_s_m:.1f}m, ego at {ego.position_s_m:.1f}m, distance: {distance:.1f}m)"
                        )

            if lane_change_triggered and not lane_change_completed and npc_vehicle:
                elapsed = current_time - lane_change_start_time
                if (
                    elapsed >= lane_change_duration * 0.3
                    and npc_vehicle.current_lane_id == -2
                ):
                    npc_vehicle.current_lane_id = -1
                    lane_change_completed = True
                    print(
                        f">>> NPC completed lane change to lane -1 at {current_time:.1f}s\n"
                    )

            simulation.step(delta_t)

            world_state = simulation.get_world_state()
            if npc_vehicle:
                for i in range(world_state.get_vehicle_count()):
                    v = world_state.get_vehicle(i)
                    if v and v.id == npc_vehicle.id:
                        v.current_lane_id = npc_vehicle.current_lane_id
                        v.position_s_m = npc_vehicle.position_s_m
                        break

            if step % 5 == 0:
                if world_state.ego_vehicle:
                    ego = world_state.ego_vehicle

                    if npc_vehicle and step % 10 == 0:
                        distance = abs(npc_vehicle.position_s_m - ego.position_s_m)
                        print(
                            f"[DEBUG] Step {step}: Ego={ego.position_s_m:.1f}m NPC={npc_vehicle.position_s_m:.1f}m dist={distance:.1f}m NPC_vel={npc_vehicle.velocity_mps:.1f}m/s NPC_lane={npc_vehicle.current_lane_id}"
                        )

                    goal = scenario.update_goal(world_state)
                    behavior_decision = behavior_planner.plan(world_state, goal)
                    _ = motion_planner.plan(behavior_decision, delta_t)

                    distance_to_npc = "N/A"
                    if npc_vehicle:
                        if npc_vehicle.current_lane_id == ego.current_lane_id:
                            distance_to_npc = f"{abs(npc_vehicle.position_s_m - ego.position_s_m):.1f}m"
                        else:
                            distance_to_npc = (
                                f"diff lane (NPC lane={npc_vehicle.current_lane_id})"
                            )

                    state_str = "KeepLane"
                    if behavior_decision.cut_in_detected:
                        state_str = (
                            "CutInResponse"
                            if behavior_decision.safe_distance_m > 5.0
                            else "Emergency"
                        )

                    print(
                        f"Time: {current_time:5.1f}s | "
                        f"Ego: pos={ego.position_s_m:6.1f}m vel={ego.velocity_mps:5.2f}m/s lane={ego.current_lane_id} | "
                        f"Distance: {distance_to_npc:>20} | "
                        f"State: {state_str:>15}"
                    )

        print("-" * 70)
        print("\n6. Simulation completed")

    print("7. Cleaning up...")
    scenario.clear_entity_vehicles()
    print("8. Done")


def main(args):
    run_cut_in_with_planner(use_viewer=args.viewer)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LaneZero Cut-In Planner Example")
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Enable visualization of the simulation",
    )
    args = parser.parse_args()
    main(args)

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
