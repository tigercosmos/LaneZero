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

#include <Vehicle.h>
#include <Map.h>

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
    , length_m(length)
    , width_m(width)
{
}

void Vehicle::update_kinematics(double delta_t)
{
    position_s_m = position_s_m + velocity_mps * delta_t +
                   0.5 * acceleration_mps2 * delta_t * delta_t;
    velocity_mps = velocity_mps + acceleration_mps2 * delta_t;
}

void Vehicle::calculate_control(
    LaneZero::Map const & map,
    std::vector<Vehicle *> const & surrounding_vehicles)
{
    // Placeholder for Phase 2 logic
}
