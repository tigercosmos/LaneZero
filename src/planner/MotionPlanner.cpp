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

#include <LaneZero/planner/MotionPlanner.h>
#include <LaneZero/planner/BehaviorPlanner.h>

#include <cmath>
#include <algorithm>

namespace LaneZero
{

MotionTrajectory MotionPlanner::plan(BehaviorDecision const & decision, double delta_t_s)
{
    MotionTrajectory trajectory;

    double const planning_horizon_s = 5.0;
    double const max_acceleration_mps2 = 3.0;
    double const max_deceleration_mps2 = -5.0;

    int32_t num_steps = static_cast<int32_t>(planning_horizon_s / delta_t_s);

    double current_position = 0.0;
    double current_velocity = decision.target_speed_mps;

    for (int32_t i = 0; i < num_steps; ++i)
    {
        double time = i * delta_t_s;

        double acceleration = 0.0;
        if (decision.state == BehaviorState::Emergency)
        {
            acceleration = max_deceleration_mps2;
        }
        else if (decision.state == BehaviorState::CutInResponse)
        {
            acceleration = std::max(max_deceleration_mps2,
                                    (decision.target_speed_mps - current_velocity) / planning_horizon_s);
        }

        current_velocity += acceleration * delta_t_s;
        current_velocity = std::max(0.0, std::min(current_velocity, decision.target_speed_mps * 1.2));

        current_position += current_velocity * delta_t_s;

        trajectory.positions_s_m.push_back(current_position);
        trajectory.velocities_mps.push_back(current_velocity);
        trajectory.timestamps_s.push_back(time);
    }

    return trajectory;
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
