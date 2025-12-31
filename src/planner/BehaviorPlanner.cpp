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

#include <LaneZero/planner/BehaviorPlanner.h>
#include <LaneZero/simulation/WorldState.h>
#include <LaneZero/scenario/Goal.h>

#include <cmath>
#include <algorithm>

namespace LaneZero
{

BehaviorDecision BehaviorPlanner::plan(WorldState const & world_state, Goal const & goal)
{
    BehaviorDecision decision;
    decision.target_lane_id = goal.target_lane_id;
    decision.target_speed_mps = goal.desired_speed_mps;
    decision.state = BehaviorState::KeepLane;
    decision.cut_in_detected = false;

    if (!world_state.ego_vehicle)
    {
        return decision;
    }

    auto const & ego = world_state.ego_vehicle;
    double const safe_distance_threshold = 15.0;
    double const emergency_distance_threshold = 10.0;
    double const safe_speed_reduction_factor = 0.7;
    double const emergency_speed_reduction_factor = 0.4;

    for (auto const & other_vehicle_ptr : world_state.vehicles)
    {
        if (!other_vehicle_ptr)
        {
            continue;
        }

        auto const & other = other_vehicle_ptr;

        if (other->current_lane_id == ego->current_lane_id)
        {
            double longitudinal_distance = other->position_s_m - ego->position_s_m;

            if (longitudinal_distance > 0 && longitudinal_distance < safe_distance_threshold)
            {
                decision.cut_in_detected = true;
                decision.safe_distance_m = longitudinal_distance;

                if (longitudinal_distance < emergency_distance_threshold)
                {
                    decision.state = BehaviorState::Emergency;
                    decision.target_speed_mps = other->velocity_mps;
                }
                else
                {
                    decision.state = BehaviorState::CutInResponse;
                    decision.target_speed_mps = other->velocity_mps;
                }

                break;
            }
        }
    }

    return decision;
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
