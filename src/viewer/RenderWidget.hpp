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

#include <memory>
#include <vector>

#include <QWidget>
#include <QPainter>

#include <map/Map.h>
#include <vehicle/Vehicle.h>

namespace LaneZero
{

class RenderWidget : public QWidget
{
    Q_OBJECT

public:

    explicit RenderWidget(QWidget * parent = nullptr);
    ~RenderWidget() override = default;

    void set_map(Map const * map_ptr);
    void set_vehicles(std::vector<Vehicle> const * vehicles_ptr);
    void update_view();

protected:

    void paintEvent(QPaintEvent * event) override;

private:

    void render_map(QPainter & painter);
    void render_vehicles(QPainter & painter);
    void render_road(QPainter & painter, Road const & road);
    void render_lane_boundaries(QPainter & painter, Road const & road);
    void render_vehicle_box(QPainter & painter, Vehicle const & vehicle, Road const & road);

    std::pair<double, double> calculate_lane_offset(Road const & road, int32_t lane_id, double position_s);
    std::pair<double, double> interpolate_position(std::vector<Point> const & reference_line, double position_s, double length);

    Map const * m_map = nullptr;
    std::vector<Vehicle> const * m_vehicles = nullptr;

    double m_scale = 10.0;
    double m_offset_x = 100.0;
    double m_offset_y = 400.0;

}; /* end class RenderWidget */

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
