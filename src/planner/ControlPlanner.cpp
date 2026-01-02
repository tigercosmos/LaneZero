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

#include <LaneZero/planner/ControlPlanner.h>
#include <LaneZero/planner/MotionPlanner.h>

namespace LaneZero
{

ControlCommand ControlPlanner::plan(MotionTrajectory const & trajectory, double delta_t_s)
{
    ControlCommand command;
    command.acceleration_mps2 = 0.0;
    command.steering_angle_rad = 0.0;

    if (trajectory.velocities_mps.size() < 2 || trajectory.timestamps_s.size() < 2)
    {
        return command;
    }

    double velocity_current_mps = trajectory.velocities_mps[0];
    double velocity_target_mps = trajectory.velocities_mps[1];
    double time_delta_s = trajectory.timestamps_s[1] - trajectory.timestamps_s[0];

    if (time_delta_s <= 0.0)
    {
        return command;
    }

    double desired_acceleration_mps2 = (velocity_target_mps - velocity_current_mps) / time_delta_s;

    constexpr double maximum_acceleration_mps2 = 3.0;
    constexpr double maximum_deceleration_mps2 = -8.0;

    if (desired_acceleration_mps2 > maximum_acceleration_mps2)
    {
        command.acceleration_mps2 = maximum_acceleration_mps2;
    }
    else if (desired_acceleration_mps2 < maximum_deceleration_mps2)
    {
        command.acceleration_mps2 = maximum_deceleration_mps2;
    }
    else
    {
        command.acceleration_mps2 = desired_acceleration_mps2;
    }

    return command;
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
