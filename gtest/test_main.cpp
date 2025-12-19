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

#include <gtest/gtest.h>
#include <LaneZero/map/Map.h>
#include <LaneZero/vehicle/Vehicle.h>
#include <LaneZero/simulation/Simulation.h>

TEST(MapTest, Initialization)
{
    LaneZero::Map map;
    map.initialize_highway(3, 5000.0);

    EXPECT_FALSE(map.roads().empty());
    EXPECT_GT(map.roads()[0].lane_sections.size(), 0);
    EXPECT_GT(map.roads()[0].lane_sections[0].lanes.size(), 0);
}

TEST(VehicleTest, KinematicsUpdate)
{
    Vehicle car(1, VehicleType::Car, 0.0, 20.0, 0, 4.5, 2.0);
    car.acceleration_mps2 = 2.0;

    car.update_kinematics(1.0);

    EXPECT_DOUBLE_EQ(car.velocity_mps, 22.0);
    EXPECT_DOUBLE_EQ(car.position_s_m, 21.0);
}

TEST(SimulationTest, RunsWithoutCrash)
{
    Simulation sim;
    sim.simulation_map.initialize_highway(3, 5000.0);

    Vehicle * truck =
        new Vehicle(0, VehicleType::Truck, 0.0, 25.0, 1, 16.5, 2.5);
    sim.vehicles.push_back(truck);

    EXPECT_NO_THROW(sim.run(1.0, 0.1));
}
