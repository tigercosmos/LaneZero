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
#include <LaneZero/physics_engine/PhysicsEngine.h>
#include <LaneZero/physics_engine/BicycleModelEngine.h>

namespace py = pybind11;

void wrap_physics_engine(py::module_ & module)
{
    py::enum_<LaneZero::PhysicsEngineType>(module, "PhysicsEngineType")
        .value("BicycleModel", LaneZero::PhysicsEngineType::BicycleModel)
        .export_values();

    py::class_<LaneZero::RigidBodyState>(module, "RigidBodyState")
        .def(py::init<>())
        .def_readwrite("x_m", &LaneZero::RigidBodyState::x_m)
        .def_readwrite("y_m", &LaneZero::RigidBodyState::y_m)
        .def_readwrite("yaw_rad", &LaneZero::RigidBodyState::yaw_rad)
        .def_readwrite("velocity_x_mps", &LaneZero::RigidBodyState::velocity_x_mps)
        .def_readwrite("velocity_y_mps", &LaneZero::RigidBodyState::velocity_y_mps)
        .def_readwrite("yaw_rate_radps", &LaneZero::RigidBodyState::yaw_rate_radps);

    py::class_<LaneZero::VehicleControl>(module, "VehicleControl")
        .def(py::init<>())
        .def_readwrite("steering_angle_rad", &LaneZero::VehicleControl::steering_angle_rad)
        .def_readwrite("longitudinal_force_n", &LaneZero::VehicleControl::longitudinal_force_n);

    py::class_<LaneZero::VehiclePhysicsParameters>(module, "VehiclePhysicsParameters")
        .def(py::init<>())
        .def_readwrite("mass_kg", &LaneZero::VehiclePhysicsParameters::mass_kg)
        .def_readwrite("yaw_inertia_kgm2", &LaneZero::VehiclePhysicsParameters::yaw_inertia_kgm2)
        .def_readwrite("wheelbase_m", &LaneZero::VehiclePhysicsParameters::wheelbase_m)
        .def_readwrite("front_axle_distance_m",
                       &LaneZero::VehiclePhysicsParameters::front_axle_distance_m)
        .def_readwrite("rear_axle_distance_m",
                       &LaneZero::VehiclePhysicsParameters::rear_axle_distance_m)
        .def_readwrite("tire_cornering_stiffness_front_n_per_rad",
                       &LaneZero::VehiclePhysicsParameters::tire_cornering_stiffness_front_n_per_rad)
        .def_readwrite("tire_cornering_stiffness_rear_n_per_rad",
                       &LaneZero::VehiclePhysicsParameters::tire_cornering_stiffness_rear_n_per_rad)
        .def_readwrite("max_steering_angle_rad",
                       &LaneZero::VehiclePhysicsParameters::max_steering_angle_rad)
        .def_readwrite("max_steering_rate_radps",
                       &LaneZero::VehiclePhysicsParameters::max_steering_rate_radps)
        .def_readwrite("max_lateral_acceleration_mps2",
                       &LaneZero::VehiclePhysicsParameters::max_lateral_acceleration_mps2);
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
