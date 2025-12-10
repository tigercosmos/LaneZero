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

#include "Vehicle.h"
#include "Map.h"

namespace py = pybind11;

void wrap_Vehicle(py::module & module)
{
    py::enum_<VehicleType>(module, "VehicleType")
        .value("Car", VehicleType::Car)
        .value("Truck", VehicleType::Truck)
        .export_values();

    py::class_<Vehicle>(module, "Vehicle")
        .def(py::init<int, VehicleType, double, double, int, double, double>(),
             py::arg("id"),
             py::arg("type"),
             py::arg("position"),
             py::arg("velocity"),
             py::arg("lane_id"),
             py::arg("length"),
             py::arg("width"))
        .def_readwrite("id", &Vehicle::id)
        .def_readwrite("type", &Vehicle::type)
        .def_readwrite("position_s_m", &Vehicle::position_s_m)
        .def_readwrite("velocity_mps", &Vehicle::velocity_mps)
        .def_readwrite("acceleration_mps2", &Vehicle::acceleration_mps2)
        .def_readwrite("current_lane_id", &Vehicle::current_lane_id)
        .def_readwrite("length_m", &Vehicle::length_m)
        .def_readwrite("width_m", &Vehicle::width_m)
        .def("update_kinematics",
             &Vehicle::update_kinematics,
             py::arg("delta_t"),
             "Update vehicle position and velocity based on acceleration")
        .def("calculate_control",
             &Vehicle::calculate_control,
             py::arg("map"),
             py::arg("surrounding_vehicles"),
             "Calculate control input for the vehicle");
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
