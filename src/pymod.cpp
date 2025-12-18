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

namespace py = pybind11;

void wrap_Vehicle(py::module & module);
void wrap_Map(py::module & module);
void wrap_Simulation(py::module & module);
void wrap_WorldState(py::module & module);
void wrap_physics_engine(py::module_ & module);

namespace LaneZero
{
namespace python
{
void wrap_viewer(py::module & module);
} /* end namespace python */
} /* end namespace LaneZero */

PYBIND11_MODULE(_core, module)
{
    module.doc() = "LaneZero: A traffic simulation library";

    wrap_Vehicle(module);
    wrap_Map(module);
    wrap_WorldState(module);
    wrap_Simulation(module);
    wrap_physics_engine(module);

#ifdef LANEZERO_USE_QT
    LaneZero::python::wrap_viewer(module);
#endif
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
