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
Test cases for Vehicle class
"""

import LaneZero as lz


class TestVehicleType:
    """Test VehicleType enum"""

    def test_vehicle_type_enum(self):
        """Test that VehicleType enum values are accessible"""
        assert hasattr(lz.VehicleType, "Car")
        assert hasattr(lz.VehicleType, "Truck")

    def test_vehicle_type_values(self):
        """Test VehicleType enum values"""
        assert lz.VehicleType.Car != lz.VehicleType.Truck


class TestVehicle:
    """Test Vehicle class"""

    def test_vehicle_creation(self):
        """Test creating a vehicle instance"""
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )
        assert vehicle is not None

    def test_vehicle_attributes(self):
        """Test vehicle attribute access"""
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )
        assert vehicle.id == 1
        assert vehicle.type == lz.VehicleType.Car
        assert vehicle.position_s_m == 100.0
        assert vehicle.velocity_mps == 25.0
        assert vehicle.acceleration_mps2 == 0.0
        assert vehicle.current_lane_id == 0
        assert vehicle.length_m == 4.5
        assert vehicle.width_m == 2.0

    def test_vehicle_attribute_modification(self):
        """Test modifying vehicle attributes"""
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=100.0,
            velocity=25.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )
        vehicle.acceleration_mps2 = 2.0
        assert vehicle.acceleration_mps2 == 2.0

        vehicle.velocity_mps = 30.0
        assert vehicle.velocity_mps == 30.0

        vehicle.position_s_m = 150.0
        assert vehicle.position_s_m == 150.0

    def test_update_kinematics_constant_velocity(self):
        """Test kinematics update with zero acceleration"""
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=0.0,
            velocity=10.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )
        vehicle.acceleration_mps2 = 0.0
        vehicle.update_kinematics(delta_t=1.0)
        assert abs(vehicle.position_s_m - 10.0) < 1e-6
        assert abs(vehicle.velocity_mps - 10.0) < 1e-6

    def test_update_kinematics_with_acceleration(self):
        """Test kinematics update with constant acceleration"""
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=0.0,
            velocity=0.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )
        vehicle.acceleration_mps2 = 2.0
        vehicle.update_kinematics(delta_t=1.0)
        # s = v0*t + 0.5*a*t^2 = 0 + 0.5*2*1 = 1.0
        # v = v0 + a*t = 0 + 2*1 = 2.0
        assert abs(vehicle.position_s_m - 1.0) < 1e-6
        assert abs(vehicle.velocity_mps - 2.0) < 1e-6

    def test_update_kinematics_multiple_steps(self):
        """Test multiple kinematics updates"""
        vehicle = lz.Vehicle(
            id=1,
            type=lz.VehicleType.Car,
            position=0.0,
            velocity=10.0,
            lane_id=0,
            length=4.5,
            width=2.0,
        )
        vehicle.acceleration_mps2 = 1.0

        # First update
        vehicle.update_kinematics(delta_t=1.0)
        # s = 10*1 + 0.5*1*1 = 10.5
        # v = 10 + 1*1 = 11.0
        assert abs(vehicle.position_s_m - 10.5) < 1e-6
        assert abs(vehicle.velocity_mps - 11.0) < 1e-6

        # Second update
        vehicle.update_kinematics(delta_t=1.0)
        # s = 10.5 + 11*1 + 0.5*1*1 = 22.0
        # v = 11 + 1*1 = 12.0
        assert abs(vehicle.position_s_m - 22.0) < 1e-6
        assert abs(vehicle.velocity_mps - 12.0) < 1e-6

    def test_truck_vehicle(self):
        """Test creating a truck vehicle"""
        truck = lz.Vehicle(
            id=2,
            type=lz.VehicleType.Truck,
            position=200.0,
            velocity=20.0,
            lane_id=1,
            length=12.0,
            width=2.5,
        )
        assert truck.type == lz.VehicleType.Truck
        assert truck.length_m == 12.0
        assert truck.width_m == 2.5


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
