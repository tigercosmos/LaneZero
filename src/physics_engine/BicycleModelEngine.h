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
 * SUBSTITUTE APPS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <PhysicsEngine.h>

namespace LaneZero
{

/**
 * Bicycle model physics engine for 2D vehicle dynamics.
 *
 * Implements the classic bicycle (single-track) model where the vehicle is
 * represented by a rigid body with front and rear axles. This model captures:
 * - Slip angles at front and rear tires
 * - Linear tire force model (valid for small slip angles)
 * - 2D rigid body dynamics with yaw rotation
 * - Coupling between longitudinal and lateral dynamics
 *
 * The model is suitable for simulating passenger vehicle dynamics at moderate
 * speeds and accelerations, where tire slip angles remain small (<10 degrees).
 *
 * References:
 * - Rajamani, R. "Vehicle Dynamics and Control" (2011)
 * - Paden, B. et al. "A Survey of Motion Planning and Control Techniques
 *   for Self-Driving Urban Vehicles" (2016)
 */
class BicycleModelEngine : public PhysicsEngine
{
public:
    BicycleModelEngine() = default;
    BicycleModelEngine(BicycleModelEngine const &) = default;
    BicycleModelEngine(BicycleModelEngine &&) = default;
    BicycleModelEngine & operator=(BicycleModelEngine const &) = default;
    BicycleModelEngine & operator=(BicycleModelEngine &&) = default;
    ~BicycleModelEngine() override = default;

    /**
     * Update vehicle state using bicycle model dynamics.
     *
     * Implements the full bicycle model equations:
     * 1. Compute slip angles at front and rear axles
     * 2. Calculate tire lateral forces using linear tire model
     * 3. Apply Newton-Euler equations in body-fixed frame
     * 4. Integrate velocities and positions using forward Euler
     *
     * @param state Current rigid body state - updated in place
     * @param control Control inputs (steering, longitudinal force)
     * @param parameters Vehicle physical parameters
     * @param delta_t_s Time step for integration in seconds
     */
    void update(RigidBodyState & state,
                VehicleControl const & control,
                VehiclePhysicsParameters const & parameters,
                double delta_t_s) override;

    /**
     * Get the physics engine type.
     *
     * @return PhysicsEngineType::BicycleModel
     */
    PhysicsEngineType type() const override
    {
        return PhysicsEngineType::BicycleModel;
    }

private:
    /**
     * Clamp a value to the specified range.
     *
     * @param value Input value to clamp
     * @param min_value Minimum allowed value
     * @param max_value Maximum allowed value
     * @return Clamped value in [min_value, max_value]
     */
    double clamp(double value, double min_value, double max_value) const;

    /**
     * Compute slip angle at front axle.
     *
     * Slip angle is the difference between tire heading and velocity direction.
     * At front axle: α_f = atan2(v_y + l_f * ω, v_x) - δ
     *
     * @param state Current vehicle state
     * @param steering_angle_rad Front wheel steering angle in radians
     * @param parameters Vehicle parameters (geometry, mass, etc.)
     * @return Front slip angle in radians
     */
    double compute_slip_angle_front(RigidBodyState const & state,
                                    double steering_angle_rad,
                                    VehiclePhysicsParameters const & parameters) const;

    /**
     * Compute slip angle at rear axle.
     *
     * At rear axle: α_r = atan2(v_y - l_r * ω, v_x)
     * Note: no steering angle subtraction since rear wheels don't steer.
     *
     * @param state Current vehicle state
     * @param parameters Vehicle parameters (geometry, mass, etc.)
     * @return Rear slip angle in radians
     */
    double compute_slip_angle_rear(RigidBodyState const & state,
                                   VehiclePhysicsParameters const & parameters) const;
}; /* end class BicycleModelEngine */

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
