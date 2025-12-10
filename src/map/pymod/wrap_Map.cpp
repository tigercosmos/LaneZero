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

#include "Map.h"

namespace py = pybind11;

void wrap_Map(py::module & module)
{
    py::class_<Lane>(module, "Lane")
        .def(py::init<int, double, int, std::optional<int>>(),
             py::arg("id"),
             py::arg("length"),
             py::arg("speed_limit"),
             py::arg("adjacent") = std::nullopt)
        .def_readwrite("id", &Lane::id)
        .def_readwrite("length_meters", &Lane::length_meters)
        .def_readwrite("speed_limit_kph", &Lane::speed_limit_kph)
        .def_readwrite("adjacent_lane_id", &Lane::adjacent_lane_id);

    py::class_<Map>(module, "Map")
        .def(py::init<>())
        .def_readwrite("lanes", &Map::lanes)
        .def("initialize_highway", &Map::initialize_highway,
             py::arg("num_lanes"),
             py::arg("lane_length"),
             "Initialize a highway with specified number of lanes and lane length");
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
