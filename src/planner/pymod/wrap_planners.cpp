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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <LaneZero/planner/BehaviorPlanner.h>
#include <LaneZero/planner/MotionPlanner.h>
#include <LaneZero/simulation/WorldState.h>
#include <LaneZero/scenario/Goal.h>

namespace py = pybind11;

void wrap_planners(py::module & module)
{
    py::enum_<LaneZero::BehaviorState>(module, "BehaviorState")
        .value("KeepLane", LaneZero::BehaviorState::KeepLane)
        .value("CutInResponse", LaneZero::BehaviorState::CutInResponse)
        .value("Emergency", LaneZero::BehaviorState::Emergency);

    py::class_<LaneZero::BehaviorDecision>(module, "BehaviorDecision")
        .def(py::init<>())
        .def_readwrite("target_lane_id", &LaneZero::BehaviorDecision::target_lane_id)
        .def_readwrite("target_speed_mps", &LaneZero::BehaviorDecision::target_speed_mps)
        .def_readwrite("state", &LaneZero::BehaviorDecision::state)
        .def_readwrite("cut_in_detected", &LaneZero::BehaviorDecision::cut_in_detected)
        .def_readwrite("safe_distance_m", &LaneZero::BehaviorDecision::safe_distance_m);

    py::class_<LaneZero::BehaviorPlanner>(module, "BehaviorPlanner")
        .def(py::init<>())
        .def("plan", &LaneZero::BehaviorPlanner::plan, py::arg("world_state"), py::arg("goal"), "Plan behavior based on world state and goal");

    py::class_<LaneZero::MotionTrajectory>(module, "MotionTrajectory")
        .def(py::init<>())
        .def_readwrite("positions_s_m", &LaneZero::MotionTrajectory::positions_s_m)
        .def_readwrite("velocities_mps", &LaneZero::MotionTrajectory::velocities_mps)
        .def_readwrite("timestamps_s", &LaneZero::MotionTrajectory::timestamps_s);

    py::class_<LaneZero::MotionPlanner>(module, "MotionPlanner")
        .def(py::init<>())
        .def("plan", &LaneZero::MotionPlanner::plan, py::arg("decision"), py::arg("delta_t_s"), "Plan trajectory based on behavior decision");
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
