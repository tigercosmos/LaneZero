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

#include <viewer/wrap_viewer.hpp>
#include <viewer/viewer.hpp>

namespace LaneZero
{

namespace python
{

void wrap_RManager(pybind11::module & mod)
{
    pybind11::class_<RManager>(mod, "RManager")
        .def_static("get_instance", &RManager::instance, pybind11::return_value_policy::reference)
        .def("set_up", &RManager::setUp, pybind11::return_value_policy::reference)
        .def("exec", &RManager::exec)
        .def("show", &RManager::show)
        .def("quit", &RManager::quit)
        .def("resize", &RManager::resize, pybind11::arg("w"), pybind11::arg("h"))
        .def("set_window_title", &RManager::setWindowTitle, pybind11::arg("title"))
        .def("set_map", &RManager::set_map, pybind11::arg("map"))
        .def("set_vehicles", &RManager::set_vehicles, pybind11::arg("vehicles"))
        .def("set_ego_vehicle_id", &RManager::set_ego_vehicle_id, pybind11::arg("ego_id"))
        .def("update_view", &RManager::update_view)
        .def_property_readonly("main_window", &RManager::mainWindow, pybind11::return_value_policy::reference)
        .def_property_readonly("file_menu", &RManager::fileMenu, pybind11::return_value_policy::reference)
        .def_property_readonly("simulation_menu", &RManager::simulationMenu, pybind11::return_value_policy::reference)
        .def_property_readonly("view_menu", &RManager::viewMenu, pybind11::return_value_policy::reference)
        .def_property_readonly("window_menu", &RManager::windowMenu, pybind11::return_value_policy::reference);
}

void wrap_viewer(pybind11::module & mod)
{
    pybind11::module viewer_mod = mod.def_submodule("viewer", "Qt viewer module");
    viewer_mod.attr("enable") = true;

    wrap_RManager(viewer_mod);
}

} /* end namespace python */

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
