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

#include <LaneZero/map/Map.h>

namespace py = pybind11;

void wrap_Map(py::module & module)
{
    py::class_<LaneZero::Point>(module, "Point")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &LaneZero::Point::x)
        .def_readwrite("y", &LaneZero::Point::y)
        .def_readwrite("z", &LaneZero::Point::z);

    py::class_<LaneZero::WidthCoefficients>(module, "WidthCoefficients")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double>(),
             py::arg("s_offset"),
             py::arg("a"),
             py::arg("b") = 0.0,
             py::arg("c") = 0.0,
             py::arg("d") = 0.0)
        .def_readwrite("s_offset", &LaneZero::WidthCoefficients::s_offset)
        .def_readwrite("a", &LaneZero::WidthCoefficients::a)
        .def_readwrite("b", &LaneZero::WidthCoefficients::b)
        .def_readwrite("c", &LaneZero::WidthCoefficients::c)
        .def_readwrite("d", &LaneZero::WidthCoefficients::d)
        .def_readwrite("valid_length", &LaneZero::WidthCoefficients::valid_length);

    py::class_<LaneZero::Lane>(module, "Lane")
        .def(py::init<>())
        .def(py::init<int32_t, std::string const &>(), py::arg("lane_id"), py::arg("type"))
        .def_readwrite("lane_id", &LaneZero::Lane::lane_id)
        .def_readwrite("type", &LaneZero::Lane::type)
        .def_readwrite("side", &LaneZero::Lane::side)
        .def_readwrite("level", &LaneZero::Lane::level)
        .def_readwrite("speed_limit", &LaneZero::Lane::speed_limit)
        .def_readwrite("width", &LaneZero::Lane::width)
        .def_readwrite("attributes", &LaneZero::Lane::attributes);

    py::class_<LaneZero::LaneSection>(module, "LaneSection")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("s_start"), py::arg("s_end"))
        .def_readwrite("s_start", &LaneZero::LaneSection::s_start)
        .def_readwrite("s_end", &LaneZero::LaneSection::s_end)
        .def_readwrite("lanes", &LaneZero::LaneSection::lanes);

    py::class_<LaneZero::RoadLink>(module, "RoadLink")
        .def(py::init<>())
        .def(py::init<std::string const &, std::string const &>(), py::arg("road_id"), py::arg("contact_point"))
        .def_readwrite("road_id", &LaneZero::RoadLink::road_id)
        .def_readwrite("contact_point", &LaneZero::RoadLink::contact_point);

    py::class_<LaneZero::Road>(module, "Road")
        .def(py::init<>())
        .def(py::init<std::string const &, double>(), py::arg("id"), py::arg("length"))
        .def_readwrite("id", &LaneZero::Road::id)
        .def_readwrite("length", &LaneZero::Road::length)
        .def_readwrite("reference_line", &LaneZero::Road::reference_line)
        .def_readwrite("lane_sections", &LaneZero::Road::lane_sections)
        .def_readwrite("successor", &LaneZero::Road::successor)
        .def_readwrite("predecessor", &LaneZero::Road::predecessor)
        .def_readwrite("speed_limit", &LaneZero::Road::speed_limit);

    py::class_<LaneZero::MapMeta>(module, "MapMeta")
        .def(py::init<>())
        .def(py::init<std::string const &, std::string const &>(), py::arg("name"), py::arg("version"))
        .def_readwrite("name", &LaneZero::MapMeta::name)
        .def_readwrite("version", &LaneZero::MapMeta::version)
        .def_readwrite("coordinate_system", &LaneZero::MapMeta::coordinate_system)
        .def_readwrite("units", &LaneZero::MapMeta::units)
        .def_readwrite("additional_properties", &LaneZero::MapMeta::additional_properties);

    py::class_<LaneZero::Map>(module, "Map")
        .def(py::init<>())
        .def("meta",
             py::overload_cast<>(&LaneZero::Map::meta),
             py::return_value_policy::reference_internal)
        .def("roads",
             py::overload_cast<>(&LaneZero::Map::roads),
             py::return_value_policy::reference_internal)
        .def("initialize_highway",
             &LaneZero::Map::initialize_highway,
             py::arg("num_lanes"),
             py::arg("lane_length"),
             "Initialize a highway with specified number of lanes and lane length")
        .def("to_json_string",
             &LaneZero::Map::to_json_string,
             "Serialize map to JSON string")
        .def("from_json_string",
             &LaneZero::Map::from_json_string,
             py::arg("json_string"),
             "Deserialize map from JSON string")
        .def("load_from_file",
             &LaneZero::Map::load_from_file,
             py::arg("file_path"),
             "Load map from JSON file")
        .def("save_to_file",
             &LaneZero::Map::save_to_file,
             py::arg("file_path"),
             "Save map to JSON file");
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
