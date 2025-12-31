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

#include <LaneZero/simulation/WorldState.h>

namespace py = pybind11;

void wrap_WorldState(py::module & module)
{
    py::class_<LaneZero::WorldState>(module, "WorldState")
        .def(py::init<>())
        .def_readwrite("world_map", &LaneZero::WorldState::world_map)
        .def_readwrite("current_time_s", &LaneZero::WorldState::current_time_s)
        .def_property("ego_vehicle", [](LaneZero::WorldState & self) -> Vehicle *
                      { return self.ego_vehicle.get(); },
                      [](LaneZero::WorldState & self, Vehicle const & vehicle)
                      { self.ego_vehicle = std::make_unique<Vehicle>(vehicle); },
                      py::return_value_policy::reference)
        .def("get_other_vehicles", [](LaneZero::WorldState & self)
             {
                 std::vector<Vehicle> vehicle_copies;
                 for (auto const & vehicle_ptr : self.vehicles)
                 {
                     if (vehicle_ptr)
                     {
                         vehicle_copies.push_back(*vehicle_ptr);
                     }
                 }
                 return vehicle_copies; },
             "Get list of all other vehicles (returns copies)")
        .def("add_vehicle", [](LaneZero::WorldState & self, Vehicle const & vehicle)
             { self.vehicles.push_back(std::make_unique<Vehicle>(vehicle)); },
             "Add a vehicle to the world state")
        .def("get_vehicle_count", [](LaneZero::WorldState & self)
             { return self.vehicles.size(); },
             "Get the number of vehicles in the world state")
        .def("get_vehicle", [](LaneZero::WorldState & self, size_t index) -> Vehicle *
             {
                 if (index < self.vehicles.size())
                 {
                     return self.vehicles[index].get();
                 }
                 return nullptr; },
             "Get a vehicle by index",
             py::return_value_policy::reference);
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
