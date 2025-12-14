#pragma once

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

#include <string>
#include <vector>

#include <QMainWindow>
#include <QApplication>
#include <QMenuBar>
#include <QMenu>
#include <QObject>

#include <map/Map.h>
#include <vehicle/Vehicle.h>

namespace LaneZero
{

class RenderWidget;

class RManager : public QObject
{
    Q_OBJECT

public:

    ~RManager() override;

    RManager & setUp();

    static RManager & instance();

    QCoreApplication * core() { return m_core; }

    QMainWindow * mainWindow() { return m_main_window; }

    QMenu * fileMenu() { return m_file_menu; }
    QMenu * simulationMenu() { return m_simulation_menu; }
    QMenu * viewMenu() { return m_view_menu; }
    QMenu * windowMenu() { return m_window_menu; }

    void quit() { m_core->quit(); }

    int exec() { return m_core->exec(); }

    void show() { m_main_window->show(); }

    void resize(int w, int h) { m_main_window->resize(w, h); }

    void setWindowTitle(std::string const & title) { m_main_window->setWindowTitle(QString::fromStdString(title)); }

    void set_map(Map const & map);
    void set_vehicles(std::vector<Vehicle> const & vehicles);
    void set_ego_vehicle_id(int32_t ego_id);
    void update_view();

    RenderWidget * render_widget() { return m_render_widget; }

private:

    RManager();

    void setUpWindow();
    void setUpMenu();

    bool m_already_setup = false;

    QCoreApplication * m_core = nullptr;
    QMainWindow * m_main_window = nullptr;

    QMenu * m_file_menu = nullptr;
    QMenu * m_simulation_menu = nullptr;
    QMenu * m_view_menu = nullptr;
    QMenu * m_window_menu = nullptr;

    RenderWidget * m_render_widget = nullptr;

    Map m_map_copy;
    std::vector<Vehicle> m_vehicles_copy;

}; /* end class RManager */

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
