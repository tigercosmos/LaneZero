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
Test cases for lane keeping simulation flow
"""

import LaneZero as lz


class TestLaneKeepingSimulation:
    """Test lane keeping simulation with planning flow"""

    def test_simulation_with_ego_vehicle(self):
        """Test simulation with ego vehicle using planning flow"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(num_lanes=3, lane_length=2000.0)

        ego_vehicle = lz.Vehicle(
            id=0,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=1,
            length=4.5,
            width=2.0,
        )

        simulation.set_ego_vehicle(ego_vehicle)

        initial_position = ego_vehicle.position_s_m

        simulation.step(delta_t_s=0.1)

        vehicles = simulation.get_vehicles()
        assert len(vehicles) == 1
        assert vehicles[0].position_s_m > initial_position
        assert simulation.current_time_s == 0.1

    def test_lane_keeping_multiple_steps(self):
        """Test that ego vehicle maintains lane through multiple steps"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(num_lanes=3, lane_length=2000.0)

        ego_vehicle = lz.Vehicle(
            id=0,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=1,
            length=4.5,
            width=2.0,
        )

        simulation.set_ego_vehicle(ego_vehicle)

        initial_lane = ego_vehicle.current_lane_id

        for step_index in range(10):
            simulation.step(delta_t_s=0.1)

        vehicles = simulation.get_vehicles()
        assert len(vehicles) == 1
        assert vehicles[0].current_lane_id == initial_lane
        assert abs(simulation.current_time_s - 1.0) < 1e-6

    def test_get_world_state(self):
        """Test getting world state from simulation"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(num_lanes=2, lane_length=1000.0)

        ego_vehicle = lz.Vehicle(
            id=0,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=30.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )

        simulation.set_ego_vehicle(ego_vehicle)

        world_state = simulation.get_world_state()

        assert world_state is not None
        assert world_state.current_time_s == simulation.current_time_s
        assert world_state.ego_vehicle is not None
        assert world_state.ego_vehicle.position_s_m == ego_vehicle.position_s_m

    def test_simulation_with_traffic(self):
        """Test simulation with ego vehicle and surrounding traffic"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(num_lanes=3, lane_length=2000.0)

        ego_vehicle = lz.Vehicle(
            id=0,
            type=lz.VehicleType.Car,
            position=500.0,
            velocity=30.0,
            lane_id=1,
            length=4.5,
            width=2.0,
        )

        simulation.set_ego_vehicle(ego_vehicle)

        simulation.spawn_traffic(num_vehicles=5)

        vehicles = simulation.get_vehicles()
        assert len(vehicles) == 6

        initial_position = ego_vehicle.position_s_m
        simulation.step(delta_t_s=0.1)

        vehicles = simulation.get_vehicles()
        ego_found = False
        for vehicle in vehicles:
            if vehicle.id == 0:
                ego_found = True
                assert vehicle.position_s_m > initial_position
        assert ego_found


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
