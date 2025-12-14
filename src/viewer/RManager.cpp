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

#include <viewer/RManager.hpp>
#include <viewer/RenderWidget.hpp>

namespace LaneZero
{

RManager & RManager::instance()
{
    static RManager ret;
    return ret;
}

RManager::RManager()
    : QObject()
{
    m_core = QApplication::instance();
    static int argc = 1;
    static char exename[] = "LaneZeroView";
    static char * argv[] = {exename};
    if (nullptr == m_core)
    {
        m_core = new QApplication(argc, argv);
    }

    m_main_window = new QMainWindow;
    m_main_window->setWindowTitle("LaneZero Viewer");
}

RManager & RManager::setUp()
{
    if (!m_already_setup)
    {
        this->setUpWindow();
        this->setUpMenu();
        m_already_setup = true;
    }
    return *this;
}

RManager::~RManager()
{
}

void RManager::setUpWindow()
{
    m_main_window->resize(1200, 800);

    m_render_widget = new RenderWidget(m_main_window);
    m_main_window->setCentralWidget(m_render_widget);
}

void RManager::setUpMenu()
{
    QMenuBar * menu_bar = m_main_window->menuBar();

    m_file_menu = menu_bar->addMenu("&File");
    m_simulation_menu = menu_bar->addMenu("&Simulation");
    m_view_menu = menu_bar->addMenu("&View");
    m_window_menu = menu_bar->addMenu("&Window");
}

void RManager::set_map(Map const & map)
{
    m_map_copy = map;
    if (m_render_widget)
    {
        m_render_widget->set_map(&m_map_copy);
    }
}

void RManager::set_vehicles(std::vector<Vehicle> const & vehicles)
{
    m_vehicles_copy = vehicles;
    if (m_render_widget)
    {
        m_render_widget->set_vehicles(&m_vehicles_copy);
    }
}

void RManager::set_ego_vehicle_id(int32_t ego_id)
{
    if (m_render_widget)
    {
        m_render_widget->set_ego_vehicle_id(ego_id);
    }
}

void RManager::update_view()
{
    if (m_render_widget)
    {
        m_render_widget->update_view();
    }
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
