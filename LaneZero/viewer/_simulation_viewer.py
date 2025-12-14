# -*- coding: UTF-8 -*-
#
# Copyright (c) 2025, LaneZero Contributors
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Simulation viewer wrapper for LaneZero.
"""

from ._viewer_core import enable

if enable:
    from PySide6.QtCore import QTimer

__all__ = [
    "SimulationViewer",
]


class SimulationViewer:
    def __init__(self, simulation, title="LaneZero Simulation Viewer"):
        if not enable:
            raise RuntimeError("Viewer is not available.Please build with Qt support.")

        from ._gui import controller
        from ._viewer_core import RManager

        self.simulation = simulation
        self.controller = controller
        self.rmgr = RManager.get_instance()
        self.timer = None
        self.delta_t = 0.1
        self.is_running = False
        self.is_paused = False

        self.rmgr.set_up()
        self.rmgr.set_window_title(title)
        self.rmgr.resize(w=1200, h=800)

        if simulation.simulation_map:
            self.rmgr.set_map(simulation.simulation_map)

        self.update_vehicles()

        self.rmgr.show()

    def update_vehicles(self):
        vehicles = self.simulation.get_vehicles()
        if vehicles:
            self.rmgr.set_vehicles(vehicles)
            ego_id = self.simulation.get_ego_vehicle_id()
            if ego_id >= 0:
                self.rmgr.set_ego_vehicle_id(ego_id)
            self.rmgr.update_view()

    def start_simulation(self, duration_s=None, delta_t_s=0.1):
        if not enable:
            return

        self.delta_t = delta_t_s
        self.is_running = True
        self.is_paused = False
        self.duration = duration_s
        self.start_time = self.simulation.current_time_s

        self.timer = QTimer()
        self.timer.timeout.connect(self._update_step)
        self.timer.start(int(delta_t_s * 1000))

    def _update_step(self):
        if self.is_paused or not self.is_running:
            return

        self.simulation.step(self.delta_t)
        self.update_vehicles()

        if self.duration is not None:
            elapsed = self.simulation.current_time_s - self.start_time
            if elapsed >= self.duration:
                self.stop_simulation()

    def pause_simulation(self):
        self.is_paused = True

    def resume_simulation(self):
        self.is_paused = False

    def stop_simulation(self):
        self.is_running = False
        if self.timer:
            self.timer.stop()

    def run(self, duration_s=None, delta_t_s=0.1):
        self.start_simulation(duration_s, delta_t_s)
        return self.rmgr.exec()

    def step_once(self, delta_t_s=0.1):
        self.simulation.step(delta_t_s)
        self.update_vehicles()


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
