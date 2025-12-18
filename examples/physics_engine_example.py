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

import LaneZero as lz

print("=" * 60)
print("LaneZero Physics Engine Example")
print("=" * 60)
print()

print("1. Creating a vehicle with physics engine enabled")
vehicle = lz.Vehicle(
    id=0,
    type=lz.VehicleType.Car,
    position=0.0,
    velocity=20.0,
    lane_id=0,
    length=4.5,
    width=2.0,
)

print("2. Enable bicycle model physics engine")
vehicle.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)
print(f"   Physics engine type: {vehicle.get_physics_engine_type()}")

print("\n3. Set physics parameters")
params = lz.VehiclePhysicsParameters()
params.mass_kg = 1500.0
params.yaw_inertia_kgm2 = 3000.0
params.wheelbase_m = 2.7
params.front_axle_distance_m = 1.35
params.rear_axle_distance_m = 1.35
params.tire_cornering_stiffness_front_n_per_rad = 80000.0
params.tire_cornering_stiffness_rear_n_per_rad = 80000.0
params.max_steering_angle_rad = 0.6
params.max_steering_rate_radps = 0.5
params.max_lateral_acceleration_mps2 = 8.0
vehicle.set_physics_parameters(params)
print(f"   Mass: {params.mass_kg} kg")
print(f"   Wheelbase: {params.wheelbase_m} m")

print("\n4. Set initial physics state")
physics_state = lz.RigidBodyState()
physics_state.x_m = 0.0
physics_state.y_m = 0.0
physics_state.yaw_rad = 0.0
physics_state.velocity_x_mps = 20.0
physics_state.velocity_y_mps = 0.0
physics_state.yaw_rate_radps = 0.0
vehicle.set_physics_state(physics_state)
print(f"   Initial velocity: {physics_state.velocity_x_mps} m/s")
print(f"   Initial yaw: {physics_state.yaw_rad} rad")

print("\n5. Set control inputs for straight line driving")
control = lz.VehicleControl()
control.steering_angle_rad = 0.0
control.longitudinal_force_n = 0.0
vehicle.set_control(control)
print(f"   Steering angle: {control.steering_angle_rad} rad")
print(f"   Longitudinal force: {control.longitudinal_force_n} N")

print("\n6. Simulate 5 seconds with straight driving")
delta_t = 0.1
num_steps = 50

for step in range(num_steps):
    vehicle.update_kinematics(delta_t)

state = vehicle.get_physics_state()
print(f"   Final position: ({state.x_m:.2f}, {state.y_m:.2f}) m")
print(
    f"   Final velocity: ({state.velocity_x_mps:.2f}, {state.velocity_y_mps:.2f}) m/s"
)
print(f"   Final yaw: {state.yaw_rad:.4f} rad")

print("\n7. Apply steering for turning maneuver")
control.steering_angle_rad = 0.1
control.longitudinal_force_n = 0.0
vehicle.set_control(control)
print(f"   New steering angle: {control.steering_angle_rad} rad")

print("\n8. Simulate 3 seconds with turning")
num_steps = 30
for step in range(num_steps):
    vehicle.update_kinematics(delta_t)

state = vehicle.get_physics_state()
print(f"   Final position: ({state.x_m:.2f}, {state.y_m:.2f}) m")
print(
    f"   Final velocity: ({state.velocity_x_mps:.2f}, {state.velocity_y_mps:.2f}) m/s"
)
print(f"   Final yaw: {state.yaw_rad:.4f} rad")
print(f"   Final yaw rate: {state.yaw_rate_radps:.4f} rad/s")

print("\n" + "=" * 60)
print("Physics engine simulation completed successfully!")
print("=" * 60)

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
