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

#include <LaneZero/simulation/Simulation.h>

namespace py = pybind11;

void wrap_Simulation(py::module & module)
{
    py::class_<Simulation>(module, "Simulation")
        .def(py::init<>())
        .def_readwrite("simulation_map", &Simulation::simulation_map)
        .def_readwrite("current_time_s", &Simulation::current_time_s)
        .def("run",
             &Simulation::run,
             py::arg("duration_s"),
             py::arg("delta_t_s"),
             "Run simulation for specified duration with given time step")
        .def("step",
             &Simulation::step,
             py::arg("delta_t_s"),
             "Advance simulation by one time step")
        .def("spawn_traffic",
             &Simulation::spawn_traffic,
             py::arg("num_vehicles"),
             "Spawn random traffic vehicles")
        .def("check_collision",
             &Simulation::check_collision,
             "Check if any vehicles are colliding")
        .def("get_vehicles", [](Simulation & self)
             {
                 std::vector<Vehicle> vehicle_copies;
                 for (auto * vehicle : self.vehicles)
                 {
                     if (vehicle != nullptr)
                     {
                         vehicle_copies.push_back(*vehicle);
                     }
                 }
                 return vehicle_copies; },
             "Get list of all vehicles in the simulation (returns copies)")
        .def("add_vehicle_copy", [](Simulation & self, Vehicle const & vehicle)
             { self.vehicles.push_back(new Vehicle(vehicle)); },
             py::arg("vehicle"),
             "Add a copy of a vehicle to the simulation")
        .def("get_world_state", &Simulation::get_world_state, "Get current world state from simulation")
        .def("set_ego_vehicle", [](Simulation & self, Vehicle const & vehicle)
             {
                 Vehicle * new_vehicle = new Vehicle(vehicle);
                 self.vehicles.push_back(new_vehicle);
                 self.set_ego_vehicle(new_vehicle); },
             py::arg("vehicle"),
             "Set ego vehicle for planning-based simulation")
        .def("get_ego_vehicle_id", &Simulation::get_ego_vehicle_id, "Get ego vehicle ID (-1 if not set)");
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
