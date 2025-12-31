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
#include <cmath>

#include <LaneZero/map/Map.h>
#include <LaneZero/vehicle/Vehicle.h>
#include <LaneZero/physics_engine/BicycleModelEngine.h>

Vehicle::Vehicle(int32_t id,
                 VehicleType type,
                 double position,
                 double velocity,
                 int32_t lane_id,
                 double length,
                 double width)
    : id(id)
    , type(type)
    , position_s_m(position)
    , velocity_mps(velocity)
    , acceleration_mps2(0.0)
    , current_lane_id(lane_id)
    , lateral_offset_m(0.0)
    , length_m(length)
    , width_m(width)
    , m_physics_engine(nullptr)
{
    m_physics_state.velocity_x_mps = velocity;
}

Vehicle::Vehicle(Vehicle const & other)
    : id(other.id)
    , type(other.type)
    , position_s_m(other.position_s_m)
    , velocity_mps(other.velocity_mps)
    , acceleration_mps2(other.acceleration_mps2)
    , current_lane_id(other.current_lane_id)
    , lateral_offset_m(other.lateral_offset_m)
    , length_m(other.length_m)
    , width_m(other.width_m)
    , m_physics_state(other.m_physics_state)
    , m_control(other.m_control)
    , m_physics_parameters(other.m_physics_parameters)
{
    if (other.m_physics_engine)
    {
        LaneZero::PhysicsEngineType engine_type = other.m_physics_engine->type();
        switch (engine_type)
        {
        case LaneZero::PhysicsEngineType::BicycleModel:
            m_physics_engine = std::make_unique<LaneZero::BicycleModelEngine>();
            break;
        default:
            m_physics_engine = std::make_unique<LaneZero::BicycleModelEngine>();
            break;
        }
    }
}

Vehicle & Vehicle::operator=(Vehicle const & other)
{
    if (this != &other)
    {
        id = other.id;
        type = other.type;
        position_s_m = other.position_s_m;
        velocity_mps = other.velocity_mps;
        acceleration_mps2 = other.acceleration_mps2;
        current_lane_id = other.current_lane_id;
        lateral_offset_m = other.lateral_offset_m;
        length_m = other.length_m;
        width_m = other.width_m;
        m_physics_state = other.m_physics_state;
        m_control = other.m_control;
        m_physics_parameters = other.m_physics_parameters;

        if (other.m_physics_engine)
        {
            LaneZero::PhysicsEngineType engine_type = other.m_physics_engine->type();
            switch (engine_type)
            {
            case LaneZero::PhysicsEngineType::BicycleModel:
                m_physics_engine = std::make_unique<LaneZero::BicycleModelEngine>();
                break;
            default:
                m_physics_engine = std::make_unique<LaneZero::BicycleModelEngine>();
                break;
            }
        }
        else
        {
            m_physics_engine.reset();
        }
    }
    return *this;
}

void Vehicle::update_kinematics(double delta_t)
{
    position_s_m = position_s_m + velocity_mps * delta_t +
                   0.5 * acceleration_mps2 * delta_t * delta_t;
    velocity_mps = velocity_mps + acceleration_mps2 * delta_t;

    if (m_physics_engine)
    {
        m_control.longitudinal_force_n = acceleration_mps2 * m_physics_parameters.mass_kg;
        m_physics_state.velocity_x_mps = velocity_mps;

        m_physics_engine->update(m_physics_state, m_control, m_physics_parameters, delta_t);

        velocity_mps = std::sqrt(m_physics_state.velocity_x_mps * m_physics_state.velocity_x_mps +
                                 m_physics_state.velocity_y_mps * m_physics_state.velocity_y_mps);
    }
}

void Vehicle::calculate_control(
    LaneZero::Map const & map,
    std::vector<Vehicle *> const & surrounding_vehicles)
{
    // Placeholder for Phase 2 logic
}

void Vehicle::set_physics_engine_type(LaneZero::PhysicsEngineType engine_type)
{
    switch (engine_type)
    {
    case LaneZero::PhysicsEngineType::BicycleModel:
        m_physics_engine = std::make_unique<LaneZero::BicycleModelEngine>();
        break;
    default:
        m_physics_engine = std::make_unique<LaneZero::BicycleModelEngine>();
        break;
    }
}

LaneZero::PhysicsEngineType Vehicle::get_physics_engine_type() const
{
    if (m_physics_engine)
    {
        return m_physics_engine->type();
    }
    return LaneZero::PhysicsEngineType::BicycleModel;
}

void Vehicle::set_control(LaneZero::VehicleControl const & control)
{
    m_control = control;
}

LaneZero::VehicleControl Vehicle::get_control() const
{
    return m_control;
}

void Vehicle::set_physics_state(LaneZero::RigidBodyState const & state)
{
    m_physics_state = state;
}

LaneZero::RigidBodyState Vehicle::get_physics_state() const
{
    return m_physics_state;
}

void Vehicle::set_physics_parameters(LaneZero::VehiclePhysicsParameters const & parameters)
{
    m_physics_parameters = parameters;
}

LaneZero::VehiclePhysicsParameters Vehicle::get_physics_parameters() const
{
    return m_physics_parameters;
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
