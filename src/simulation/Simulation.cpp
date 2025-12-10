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

#include "Simulation.h"
#include <cmath>
#include <random>

Simulation::Simulation()
    : current_time_s(0.0)
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

    for (int step = 0; step < num_steps; ++step)
    {
        current_time_s += delta_t_s;

        for (auto * vehicle : vehicles)
        {
            vehicle->calculate_control(simulation_map, vehicles);
        }

        for (auto * vehicle : vehicles)
        {
            vehicle->update_kinematics(delta_t_s);
        }

        if (check_collision())
        {
            // Handle collision
        }
    }
}

void Simulation::spawn_traffic(int num_vehicles)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dist(0.0, 1000.0);
    std::uniform_real_distribution<> vel_dist(20.0, 33.0);
    std::uniform_int_distribution<> lane_dist(
        0,
        static_cast<int>(simulation_map.lanes.size()) - 1);

    for (int i = 0; i < num_vehicles; ++i)
    {
        int id = static_cast<int>(vehicles.size());
        double position = pos_dist(gen);
        double velocity = vel_dist(gen);
        int lane_id = lane_dist(gen);

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
