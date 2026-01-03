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

#include <LaneZero/collision/CollisionDetection.h>

namespace py = pybind11;

void wrap_CollisionDetection(py::module & module)
{
    py::class_<LaneZero::AxisAlignedBoundingBox>(module, "AxisAlignedBoundingBox")
        .def(py::init<>())
        .def(py::init<double, double, double, double>(),
             py::arg("min_x"),
             py::arg("min_y"),
             py::arg("max_x"),
             py::arg("max_y"))
        .def_readwrite("min_x_m", &LaneZero::AxisAlignedBoundingBox::min_x_m)
        .def_readwrite("min_y_m", &LaneZero::AxisAlignedBoundingBox::min_y_m)
        .def_readwrite("max_x_m", &LaneZero::AxisAlignedBoundingBox::max_x_m)
        .def_readwrite("max_y_m", &LaneZero::AxisAlignedBoundingBox::max_y_m)
        .def_static("from_rotated_rectangle",
                    &LaneZero::AxisAlignedBoundingBox::from_rotated_rectangle,
                    py::arg("center_x_m"),
                    py::arg("center_y_m"),
                    py::arg("length_m"),
                    py::arg("width_m"),
                    py::arg("yaw_rad"),
                    "Create an AABB from a rotated rectangle")
        .def("intersects",
             &LaneZero::AxisAlignedBoundingBox::intersects,
             py::arg("other"),
             "Check if this AABB intersects with another AABB")
        .def("center",
             &LaneZero::AxisAlignedBoundingBox::center,
             "Get the center point of the AABB")
        .def("width",
             &LaneZero::AxisAlignedBoundingBox::width,
             "Get the width of the AABB")
        .def("height",
             &LaneZero::AxisAlignedBoundingBox::height,
             "Get the height of the AABB")
        .def("area",
             &LaneZero::AxisAlignedBoundingBox::area,
             "Get the area of the AABB")
        .def("expand",
             &LaneZero::AxisAlignedBoundingBox::expand,
             py::arg("margin_m"),
             "Expand the AABB by a margin")
        .def("contains_point",
             &LaneZero::AxisAlignedBoundingBox::contains_point,
             py::arg("x_m"),
             py::arg("y_m"),
             "Check if a point is inside the AABB")
        .def("__repr__",
             [](LaneZero::AxisAlignedBoundingBox const & aabb)
             {
                 return "AxisAlignedBoundingBox(min_x=" + std::to_string(aabb.min_x_m) +
                        ", min_y=" + std::to_string(aabb.min_y_m) +
                        ", max_x=" + std::to_string(aabb.max_x_m) +
                        ", max_y=" + std::to_string(aabb.max_y_m) + ")";
             });
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
