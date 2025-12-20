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
#include <random>

#include <LaneZero/simulation/Simulation.h>

Simulation::Simulation()
    : current_time_s(0.0)
    , m_planner(std::make_unique<LaneZero::PlanningOrchestrator>())
    , m_scenario(std::make_unique<LaneZero::SimpleScenario>())
{
}

Simulation::~Simulation()
{
    for (auto * vehicle : vehicles)
    {
        delete vehicle;
    }
    vehicles.clear();
}

void Simulation::run(double duration_s, double delta_t_s)
{
    int num_steps = static_cast<int>(std::ceil(duration_s / delta_t_s));

    for (int step_index = 0; step_index < num_steps; ++step_index)
    {
        step(delta_t_s);
    }
}

void Simulation::step(double delta_t_s)
{
    current_time_s += delta_t_s;

    if (m_ego_vehicle && m_planner && m_scenario)
    {
        LaneZero::WorldState world = get_world_state();
        LaneZero::Goal goal = m_scenario->update_goal(world);
        LaneZero::PlanResult result = m_planner->plan(world, goal, delta_t_s);
        apply(result);
    }

    for (auto * vehicle : vehicles)
    {
        if (vehicle != m_ego_vehicle)
        {
            vehicle->calculate_control(simulation_map, vehicles);
        }
    }

    for (auto * vehicle : vehicles)
    {
        vehicle->update_kinematics(delta_t_s);
    }

    if (check_collision())
    {
    }
}

void Simulation::spawn_traffic(int32_t num_vehicles)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dist(0.0, 1000.0);
    std::uniform_real_distribution<> vel_dist(20.0, 33.0);

    int32_t max_lane_id = 0;
    for (auto const & road : simulation_map.roads())
    {
        for (auto const & lane_section : road.lane_sections)
        {
            for (auto const & lane : lane_section.lanes)
            {
                if (lane.lane_id > max_lane_id)
                {
                    max_lane_id = lane.lane_id;
                }
            }
        }
    }

    std::uniform_int_distribution<> lane_dist(0, max_lane_id > 0 ? max_lane_id : 0);

    for (int32_t i = 0; i < num_vehicles; ++i)
    {
        int32_t id = static_cast<int32_t>(vehicles.size());
        double position = pos_dist(gen);
        double velocity = vel_dist(gen);
        int32_t lane_id = lane_dist(gen);

        vehicles.push_back(
            new Vehicle(id, VehicleType::Car, position, velocity, lane_id, 4.5, 2.0));
    }
}

bool Simulation::check_collision()
{
    for (size_t i = 0; i < vehicles.size(); ++i)
    {
        for (size_t j = i + 1; j < vehicles.size(); ++j)
        {
            if (vehicles[i]->current_lane_id == vehicles[j]->current_lane_id)
            {
                double dist =
                    std::abs(vehicles[i]->position_s_m - vehicles[j]->position_s_m);
                double min_dist =
                    (vehicles[i]->length_m + vehicles[j]->length_m) / 2.0;
                if (dist < min_dist)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

LaneZero::WorldState Simulation::get_world_state() const
{
    LaneZero::WorldState world_state;
    world_state.world_map = simulation_map;
    world_state.current_time_s = current_time_s;

    if (m_ego_vehicle)
    {
        world_state.ego_vehicle = std::make_unique<Vehicle>(
            m_ego_vehicle->id,
            m_ego_vehicle->type,
            m_ego_vehicle->position_s_m,
            m_ego_vehicle->velocity_mps,
            m_ego_vehicle->current_lane_id,
            m_ego_vehicle->length_m,
            m_ego_vehicle->width_m);
        world_state.ego_vehicle->acceleration_mps2 = m_ego_vehicle->acceleration_mps2;
    }

    for (auto const * vehicle : vehicles)
    {
        if (vehicle != m_ego_vehicle)
        {
            auto vehicle_copy = std::make_unique<Vehicle>(
                vehicle->id,
                vehicle->type,
                vehicle->position_s_m,
                vehicle->velocity_mps,
                vehicle->current_lane_id,
                vehicle->length_m,
                vehicle->width_m);
            vehicle_copy->acceleration_mps2 = vehicle->acceleration_mps2;
            world_state.vehicles.push_back(std::move(vehicle_copy));
        }
    }

    return world_state;
}

void Simulation::apply(LaneZero::PlanResult const & result)
{
    if (m_ego_vehicle)
    {
        m_ego_vehicle->acceleration_mps2 = result.acceleration_mps2;

        LaneZero::VehicleControl control;
        control.steering_angle_rad = result.steering_angle_rad;
        control.longitudinal_force_n = result.acceleration_mps2 *
                                       m_ego_vehicle->get_physics_parameters().mass_kg;
        m_ego_vehicle->set_control(control);
    }
}

void Simulation::set_ego_vehicle(Vehicle * vehicle)
{
    m_ego_vehicle = vehicle;
}

int32_t Simulation::get_ego_vehicle_id() const
{
    if (m_ego_vehicle)
    {
        return m_ego_vehicle->id;
    }
    return -1;
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
