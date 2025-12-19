/*
 * Copyright (c) 2025, LaneZero Contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <LaneZero/physics_engine/BicycleModelEngine.h>
#include <cmath>
#include <algorithm>

namespace LaneZero
{

double BicycleModelEngine::clamp(double value, double min_value, double max_value) const
{
    return std::max(min_value, std::min(value, max_value));
}

double BicycleModelEngine::compute_slip_angle_front(
    RigidBodyState const & state,
    double steering_angle_rad,
    VehiclePhysicsParameters const & parameters) const
{
    // Compute total velocity magnitude at center of mass
    // ||v|| = sqrt(vx^2 + vy^2)
    double velocity_center_of_mass = std::sqrt(state.velocity_x_mps * state.velocity_x_mps +
                                               state.velocity_y_mps * state.velocity_y_mps);

    // At very low speeds, slip angle is undefined and set to zero
    // This prevents numerical instability in atan2 calculation
    if (velocity_center_of_mass < 0.1)
    {
        return 0.0;
    }

    // Compute velocity at front axle using rigid body kinematics:
    // v_front_y = v_y + l_f * yaw_rate
    // where l_f is the distance from center of mass to front axle
    // The term l_f * yaw_rate accounts for the rotational contribution
    double velocity_front_y = state.velocity_y_mps +
                              parameters.front_axle_distance_m * state.yaw_rate_radps;
    double velocity_front_x = state.velocity_x_mps;

    // Slip angle α_f is the angle between the tire's heading and its velocity:
    // α_f = atan2(v_front_y, v_front_x) - δ
    // where δ is the steering angle
    // Positive slip angle means tire velocity points outward relative to heading
    double slip_angle = std::atan2(velocity_front_y, velocity_front_x) - steering_angle_rad;

    return slip_angle;
}

double BicycleModelEngine::compute_slip_angle_rear(
    RigidBodyState const & state,
    VehiclePhysicsParameters const & parameters) const
{
    // Compute total velocity magnitude at center of mass
    double velocity_center_of_mass = std::sqrt(state.velocity_x_mps * state.velocity_x_mps +
                                               state.velocity_y_mps * state.velocity_y_mps);

    // At very low speeds, slip angle is undefined and set to zero
    if (velocity_center_of_mass < 0.1)
    {
        return 0.0;
    }

    // Compute velocity at rear axle using rigid body kinematics:
    // v_rear_y = v_y - l_r * yaw_rate
    // where l_r is the distance from center of mass to rear axle
    // Note the minus sign: rear axle moves opposite to front during rotation
    double velocity_rear_y = state.velocity_y_mps -
                             parameters.rear_axle_distance_m * state.yaw_rate_radps;
    double velocity_rear_x = state.velocity_x_mps;

    // Rear slip angle α_r is the angle between tire heading and velocity:
    // α_r = atan2(v_rear_y, v_rear_x)
    // Note: no steering angle subtraction since rear wheels don't steer
    double slip_angle = std::atan2(velocity_rear_y, velocity_rear_x);

    return slip_angle;
}

void BicycleModelEngine::update(RigidBodyState & state,
                                VehicleControl const & control,
                                VehiclePhysicsParameters const & parameters,
                                double delta_t_s)
{
    // ========== Step 1: Clamp steering angle to physical limits ==========
    // Steering angle is limited by vehicle's mechanical constraints
    double steering_angle = clamp(control.steering_angle_rad,
                                  -parameters.max_steering_angle_rad,
                                  parameters.max_steering_angle_rad);

    // ========== Step 2: Compute slip angles at front and rear axles ==========
    // Slip angle is the difference between where tire is pointing and where it is moving
    double slip_angle_front = compute_slip_angle_front(state, steering_angle, parameters);
    double slip_angle_rear = compute_slip_angle_rear(state, parameters);

    // ========== Step 3: Compute lateral tire forces using linear tire model ==========
    // Linear tire model: F_y = -C_α * α
    // where C_α is cornering stiffness and α is slip angle
    // Negative sign because force opposes slip (restores alignment)
    // This is valid for small slip angles (< 5-10 degrees)
    double tire_force_front_y = -parameters.tire_cornering_stiffness_front_n_per_rad *
                                slip_angle_front;
    double tire_force_rear_y = -parameters.tire_cornering_stiffness_rear_n_per_rad *
                               slip_angle_rear;

    // ========== Step 4: Transform forces to body frame and sum total forces ==========
    // Longitudinal force (x-direction in body frame):
    // F_x = F_long - F_front_y * sin(δ)
    // The front tire lateral force has a component in x-direction when steered
    double total_force_x = control.longitudinal_force_n -
                           tire_force_front_y * std::sin(steering_angle);

    // Lateral force (y-direction in body frame):
    // F_y = F_front_y * cos(δ) + F_rear_y
    // Front lateral force is rotated by steering angle into body frame
    double total_force_y = tire_force_front_y * std::cos(steering_angle) + tire_force_rear_y;

    // ========== Step 5: Apply Newton's second law in rotating reference frame ==========
    // In body-fixed frame (rotating with vehicle), accelerations include centrifugal terms:
    // a_x_body = F_x/m + v_y * ω_z  (centrifugal acceleration from lateral velocity)
    // a_y_body = F_y/m - v_x * ω_z  (centrifugal acceleration from longitudinal velocity)
    // These extra terms account for the fact that the body frame is rotating
    double acceleration_x_body = total_force_x / parameters.mass_kg +
                                 state.velocity_y_mps * state.yaw_rate_radps;
    double acceleration_y_body = total_force_y / parameters.mass_kg -
                                 state.velocity_x_mps * state.yaw_rate_radps;

    // ========== Step 6: Apply lateral acceleration limit ==========
    // Physical limit to prevent unrealistic maneuvers (e.g., tire friction limit)
    // If total acceleration exceeds limit, scale both components proportionally
    double lateral_acceleration = std::sqrt(acceleration_y_body * acceleration_y_body +
                                            acceleration_x_body * acceleration_x_body);
    if (lateral_acceleration > parameters.max_lateral_acceleration_mps2)
    {
        double scale_factor = parameters.max_lateral_acceleration_mps2 / lateral_acceleration;
        acceleration_x_body *= scale_factor;
        acceleration_y_body *= scale_factor;
    }

    // ========== Step 7: Compute yaw moment and angular acceleration ==========
    // Yaw moment (torque about vertical axis):
    // M_z = l_f * F_front_y * cos(δ) - l_r * F_rear_y
    // Front force creates moment arm l_f, rear force creates moment arm l_r
    // Front force is rotated by steering angle (only y-component creates yaw moment)
    double yaw_moment = parameters.front_axle_distance_m * tire_force_front_y *
                            std::cos(steering_angle) -
                        parameters.rear_axle_distance_m * tire_force_rear_y;

    // Euler's rotation equation: α_z = M_z / I_z
    // where I_z is yaw moment of inertia
    double yaw_acceleration = yaw_moment / parameters.yaw_inertia_kgm2;

    // ========== Step 8: Integrate velocities using forward Euler method ==========
    // v(t + Δt) = v(t) + a(t) * Δt
    state.velocity_x_mps += acceleration_x_body * delta_t_s;
    state.velocity_y_mps += acceleration_y_body * delta_t_s;
    state.yaw_rate_radps += yaw_acceleration * delta_t_s;

    // ========== Step 9: Transform body-frame velocities to global frame ==========
    // Rotation matrix from body to global frame:
    // [v_x_global]   [cos(ψ)  -sin(ψ)] [v_x_body]
    // [v_y_global] = [sin(ψ)   cos(ψ)] [v_y_body]
    // where ψ is the yaw angle
    double velocity_x_global = state.velocity_x_mps * std::cos(state.yaw_rad) -
                               state.velocity_y_mps * std::sin(state.yaw_rad);
    double velocity_y_global = state.velocity_x_mps * std::sin(state.yaw_rad) +
                               state.velocity_y_mps * std::cos(state.yaw_rad);

    // ========== Step 10: Integrate positions using forward Euler method ==========
    // p(t + Δt) = p(t) + v(t) * Δt
    state.x_m += velocity_x_global * delta_t_s;
    state.y_m += velocity_y_global * delta_t_s;
    state.yaw_rad += state.yaw_rate_radps * delta_t_s;

    // ========== Step 11: Normalize yaw angle to [-π, π] ==========
    // Keep angle in standard range to prevent numerical drift
    while (state.yaw_rad > M_PI)
    {
        state.yaw_rad -= 2.0 * M_PI;
    }
    while (state.yaw_rad < -M_PI)
    {
        state.yaw_rad += 2.0 * M_PI;
    }
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
