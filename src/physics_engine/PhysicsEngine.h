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

#pragma once

#include <cstdint>

namespace LaneZero
{

/**
 * Enumeration of available physics engine types.
 * Allows selection of different vehicle dynamics models at runtime.
 */
enum class PhysicsEngineType
{
    BicycleModel // 2D bicycle model with front/rear tire dynamics
};

/**
 * 2D rigid body state representation for vehicle dynamics.
 * Uses a body-fixed coordinate frame where x-axis points forward,
 * y-axis points left, and z-axis points up (right-hand rule).
 */
struct RigidBodyState
{
    double x_m = 0.0; // Global X position in meters (world frame)
    double y_m = 0.0; // Global Y position in meters (world frame)
    double yaw_rad = 0.0; // Yaw angle in radians, rotation about z-axis (world frame)
    double velocity_x_mps = 0.0; // Forward velocity in m/s (body frame, along vehicle longitudinal axis)
    double velocity_y_mps = 0.0; // Lateral velocity in m/s (body frame, along vehicle lateral axis)
    double yaw_rate_radps = 0.0; // Yaw rate in rad/s (angular velocity about z-axis)
}; /* end struct RigidBodyState */

/**
 * Control inputs for vehicle dynamics.
 * These are the actuator commands that drive vehicle motion.
 */
struct VehicleControl
{
    double steering_angle_rad = 0.0; // Front wheel steering angle in radians (positive = left turn)
    double longitudinal_force_n = 0.0; // Longitudinal force in Newtons (positive = acceleration)
}; /* end struct VehicleControl */

/**
 * Physical parameters defining vehicle dynamics characteristics.
 * These parameters determine how the vehicle responds to control inputs.
 * Default values represent a typical passenger car.
 */
struct VehiclePhysicsParameters
{
    // Mass properties
    double mass_kg = 1500.0; // Vehicle mass in kilograms
    double yaw_inertia_kgm2 = 3000.0; // Moment of inertia about z-axis in kg⋅m²

    // Geometry
    double wheelbase_m = 2.7; // Distance between front and rear axles in meters
    double front_axle_distance_m = 1.35; // Distance from center of mass to front axle in meters
    double rear_axle_distance_m = 1.35; // Distance from center of mass to rear axle in meters

    // Tire properties (linear tire model)
    double tire_cornering_stiffness_front_n_per_rad = 80000.0; // Front tire cornering stiffness in N/rad
    double tire_cornering_stiffness_rear_n_per_rad = 80000.0; // Rear tire cornering stiffness in N/rad

    // Physical limits
    double max_steering_angle_rad = 0.6; // Maximum steering angle in radians (~34 degrees)
    double max_steering_rate_radps = 0.5; // Maximum steering rate in rad/s (not currently enforced)
    double max_lateral_acceleration_mps2 = 8.0; // Maximum lateral acceleration in m/s² (tire friction limit)
}; /* end struct VehiclePhysicsParameters */

/**
 * Abstract base class for vehicle physics engines.
 * Provides interface for different vehicle dynamics models.
 * Derived classes implement specific physics models (e.g., bicycle model, point mass).
 */
class PhysicsEngine
{
public:
    PhysicsEngine() = default;
    PhysicsEngine(PhysicsEngine const &) = default;
    PhysicsEngine(PhysicsEngine &&) = default;
    PhysicsEngine & operator=(PhysicsEngine const &) = default;
    PhysicsEngine & operator=(PhysicsEngine &&) = default;
    virtual ~PhysicsEngine() = default;

    /**
     * Update vehicle state based on control inputs and time step.
     *
     * @param state Current rigid body state (position, velocity, orientation) - modified in place
     * @param control Control inputs (steering angle, longitudinal force)
     * @param parameters Vehicle physical parameters (mass, inertia, geometry, tire properties)
     * @param delta_t_s Time step in seconds for numerical integration
     */
    virtual void update(RigidBodyState & state,
                        VehicleControl const & control,
                        VehiclePhysicsParameters const & parameters,
                        double delta_t_s) = 0;

    /**
     * Get the type of physics engine.
     *
     * @return PhysicsEngineType enum value identifying the engine implementation
     */
    virtual PhysicsEngineType type() const = 0;
}; /* end class PhysicsEngine */

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
