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
Test cases for Simulation class
"""

import LaneZero as lz


class TestSimulation:
    """Test Simulation class"""

    def test_simulation_creation(self):
        """Test creating a simulation instance"""
        simulation = lz.Simulation()
        assert simulation is not None

    def test_simulation_initial_state(self):
        """Test initial state of simulation"""
        simulation = lz.Simulation()
        assert simulation.current_time_s == 0.0

    def test_simulation_map_access(self):
        """Test accessing simulation map"""
        simulation = lz.Simulation()
        assert simulation.simulation_map is not None
        simulation.simulation_map.initialize_highway(
            num_lanes=3,
            lane_length=2000.0
        )
        assert len(simulation.simulation_map.lanes) == 3

    def test_spawn_traffic(self):
        """Test spawning traffic vehicles"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=3,
            lane_length=2000.0
        )
        simulation.spawn_traffic(num_vehicles=5)
        vehicles = simulation.get_vehicles()
        assert len(vehicles) == 5

    def test_spawned_vehicle_properties(self):
        """Test properties of spawned vehicles"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=3,
            lane_length=2000.0
        )
        simulation.spawn_traffic(num_vehicles=3)
        vehicles = simulation.get_vehicles()

        for vehicle in vehicles:
            assert vehicle.position_s_m >= 0.0
            assert vehicle.velocity_mps > 0.0
            assert vehicle.current_lane_id >= 0
            assert vehicle.current_lane_id < 3

    def test_check_collision_no_collision(self):
        """Test collision detection when no collision occurs"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=2,
            lane_length=2000.0
        )

        # Create two vehicles far apart
        vehicle1 = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0
        )
        vehicle2 = lz.Vehicle(
            id=2,
            type=lz.VehicleType.Car,
            position=200.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0
        )

        simulation.add_vehicle_copy(vehicle1)
        simulation.add_vehicle_copy(vehicle2)

        assert not simulation.check_collision()

    def test_check_collision_with_collision(self):
        """Test collision detection when collision occurs"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=2,
            lane_length=2000.0
        )

        # Create two vehicles very close together
        vehicle1 = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0
        )
        vehicle2 = lz.Vehicle(
            id=2,
            type=lz.VehicleType.Car,
            position=101.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0
        )

        simulation.add_vehicle_copy(vehicle1)
        simulation.add_vehicle_copy(vehicle2)

        assert simulation.check_collision()

    def test_check_collision_different_lanes(self):
        """Test that vehicles in different lanes don't collide"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=2,
            lane_length=2000.0
        )

        # Create two vehicles at same position but different lanes
        vehicle1 = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0
        )
        vehicle2 = lz.Vehicle(
            id=2,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=1,
            length=4.5,
            width=2.0
        )

        simulation.add_vehicle_copy(vehicle1)
        simulation.add_vehicle_copy(vehicle2)

        assert not simulation.check_collision()

    def test_run_simulation(self):
        """Test running a simulation"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=2,
            lane_length=5000.0
        )

        # Create a vehicle with constant velocity
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=0.0,
            velocity=10.0,
            lane_id=0,
            length=4.5,
            width=2.0
        )
        vehicle.acceleration_mps2 = 0.0

        simulation.add_vehicle_copy(vehicle)

        initial_position = vehicle.position_s_m
        simulation.run(duration_s=5.0, delta_t_s=1.0)

        # After 5 seconds at 10 m/s, should move 50 meters
        vehicles = simulation.get_vehicles()
        assert len(vehicles) == 1
        assert abs(vehicles[0].position_s_m - initial_position - 50.0) < 1e-3

    def test_run_simulation_time_update(self):
        """Test that simulation time is updated correctly"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=1,
            lane_length=5000.0
        )

        assert simulation.current_time_s == 0.0
        simulation.run(duration_s=10.0, delta_t_s=0.5)
        # The time should be approximately 10.0
        assert abs(simulation.current_time_s - 10.0) < 0.6

    def test_multiple_vehicles_simulation(self):
        """Test simulation with multiple vehicles"""
        simulation = lz.Simulation()
        simulation.simulation_map.initialize_highway(
            num_lanes=3,
            lane_length=5000.0
        )

        # Add multiple vehicles
        for index in range(5):
            vehicle = lz.Vehicle(
                id=index,
                type=lz.VehicleType.Car,
                position=index * 100.0,
                velocity=20.0,
                lane_id=index % 3,
                length=4.5,
                width=2.0
            )
            vehicle.acceleration_mps2 = 0.0
            simulation.add_vehicle_copy(vehicle)

        vehicles = simulation.get_vehicles()
        assert len(vehicles) == 5

        simulation.run(duration_s=2.0, delta_t_s=0.1)

        # All vehicles should have moved
        vehicles = simulation.get_vehicles()
        for index, vehicle in enumerate(vehicles):
            expected_position = index * 100.0 + 20.0 * 2.0
            assert abs(vehicle.position_s_m - expected_position) < 0.5


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
