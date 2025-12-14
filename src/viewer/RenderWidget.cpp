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

#include <viewer/RenderWidget.hpp>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QPen>
#include <QBrush>
#include <cmath>

namespace LaneZero
{

RenderWidget::RenderWidget(QWidget * parent)
    : QWidget(parent)
{
    setMinimumSize(800, 600);
    setStyleSheet("background-color: white;");
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(false);
}

void RenderWidget::set_map(Map const * map_ptr)
{
    m_map = map_ptr;
    update();
}

void RenderWidget::set_vehicles(std::vector<Vehicle> const * vehicles_ptr)
{
    m_vehicles = vehicles_ptr;
    update();
}

void RenderWidget::set_ego_vehicle_id(int32_t ego_id)
{
    m_ego_vehicle_id = ego_id;
    update();
}

void RenderWidget::update_view()
{
    update();
}

void RenderWidget::reset_camera()
{
    m_scale = 10.0;
    m_offset_x = 100.0;
    m_offset_y = 400.0;
    m_rotation = 0.0;
    update();
}

void RenderWidget::paintEvent(QPaintEvent * event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.fillRect(rect(), Qt::white);

    painter.save();

    QPointF center(width() / 2.0, height() / 2.0);
    painter.translate(center);
    painter.rotate(m_rotation);
    painter.translate(-center);

    if (m_map)
    {
        render_map(painter);
    }

    if (m_vehicles && m_map)
    {
        render_vehicles(painter);
    }

    painter.restore();
}

void RenderWidget::render_map(QPainter & painter)
{
    for (auto const & road : m_map->roads())
    {
        render_road(painter, road);
        render_lane_boundaries(painter, road);
    }
}

void RenderWidget::render_road(QPainter & painter, Road const & road)
{
    QPen pen(QColor(100, 100, 100));
    pen.setWidth(2);
    painter.setPen(pen);

    auto const & ref_line = road.reference_line;
    if (ref_line.size() < 2)
    {
        return;
    }

    for (size_t it = 0; it < ref_line.size() - 1; ++it)
    {
        double x1 = m_offset_x + ref_line[it].x * m_scale;
        double y1 = m_offset_y - ref_line[it].y * m_scale;
        double x2 = m_offset_x + ref_line[it + 1].x * m_scale;
        double y2 = m_offset_y - ref_line[it + 1].y * m_scale;

        painter.drawLine(
            static_cast<int>(x1),
            static_cast<int>(y1),
            static_cast<int>(x2),
            static_cast<int>(y2));
    }
}

void RenderWidget::render_lane_boundaries(QPainter & painter, Road const & road)
{
    if (road.lane_sections.empty())
    {
        return;
    }

    auto const & ref_line = road.reference_line;
    if (ref_line.size() < 2)
    {
        return;
    }

    for (auto const & lane_section : road.lane_sections)
    {
        for (auto const & lane : lane_section.lanes)
        {
            if (lane.lane_id == 0)
            {
                continue;
            }

            QPen pen;
            if (lane.type == "center" || lane.lane_id == 0)
            {
                pen.setColor(QColor(255, 200, 0));
                pen.setWidth(3);
                pen.setStyle(Qt::DashLine);
            }
            else
            {
                pen.setColor(QColor(200, 200, 200));
                pen.setWidth(1);
                pen.setStyle(Qt::DashLine);
            }
            painter.setPen(pen);

            double lane_width = lane.width.empty() ? 3.0 : lane.width[0].a;
            int32_t direction = (lane.lane_id > 0) ? 1 : -1;

            for (size_t it = 0; it < ref_line.size() - 1; ++it)
            {
                double dx = ref_line[it + 1].x - ref_line[it].x;
                double dy = ref_line[it + 1].y - ref_line[it].y;
                double length = std::sqrt(dx * dx + dy * dy);

                if (length < 0.001)
                {
                    continue;
                }

                double normal_x = -dy / length;
                double normal_y = dx / length;

                double offset = lane_width * direction;

                double x1 = m_offset_x + (ref_line[it].x + normal_x * offset) * m_scale;
                double y1 = m_offset_y - (ref_line[it].y + normal_y * offset) * m_scale;
                double x2 = m_offset_x + (ref_line[it + 1].x + normal_x * offset) * m_scale;
                double y2 = m_offset_y - (ref_line[it + 1].y + normal_y * offset) * m_scale;

                painter.drawLine(
                    static_cast<int>(x1),
                    static_cast<int>(y1),
                    static_cast<int>(x2),
                    static_cast<int>(y2));
            }
        }
    }
}

void RenderWidget::render_vehicles(QPainter & painter)
{
    for (auto const & vehicle : *m_vehicles)
    {
        for (auto const & road : m_map->roads())
        {
            if (vehicle.position_s_m >= 0 && vehicle.position_s_m <= road.length)
            {
                render_vehicle_box(painter, vehicle, road);
                break;
            }
        }
    }
}

void RenderWidget::render_vehicle_box(QPainter & painter, Vehicle const & vehicle, Road const & road)
{
    auto [pos_x, pos_y] = interpolate_position(road.reference_line, vehicle.position_s_m, road.length);
    auto [offset_x, offset_y] = calculate_lane_offset(road, vehicle.current_lane_id, vehicle.position_s_m);

    pos_x += offset_x;
    pos_y += offset_y;

    double screen_x = m_offset_x + pos_x * m_scale;
    double screen_y = m_offset_y - pos_y * m_scale;

    double box_length = vehicle.length_m * m_scale;
    double box_width = vehicle.width_m * m_scale;

    if (box_length < 3.0)
    {
        box_length = 3.0;
    }
    if (box_width < 3.0)
    {
        box_width = 3.0;
    }

    QColor color;
    if (vehicle.id == m_ego_vehicle_id)
    {
        color = QColor(255, 0, 0);
    }
    else if (vehicle.type == VehicleType::Car)
    {
        color = QColor(50, 150, 255);
    }
    else
    {
        color = QColor(255, 100, 50);
    }

    painter.setBrush(QBrush(color));
    painter.setPen(QPen(Qt::black, 2));

    painter.drawRect(
        static_cast<int>(screen_x - box_length / 2.0),
        static_cast<int>(screen_y - box_width / 2.0),
        static_cast<int>(box_length),
        static_cast<int>(box_width));
}

std::pair<double, double> RenderWidget::interpolate_position(
    std::vector<Point> const & reference_line,
    double position_s,
    double length)
{
    if (reference_line.size() < 2)
    {
        return {0.0, 0.0};
    }

    double accumulated_length = 0.0;
    for (size_t it = 0; it < reference_line.size() - 1; ++it)
    {
        double dx = reference_line[it + 1].x - reference_line[it].x;
        double dy = reference_line[it + 1].y - reference_line[it].y;
        double segment_length = std::sqrt(dx * dx + dy * dy);

        if (accumulated_length + segment_length >= position_s)
        {
            double ratio = (position_s - accumulated_length) / segment_length;
            double x = reference_line[it].x + dx * ratio;
            double y = reference_line[it].y + dy * ratio;
            return {x, y};
        }

        accumulated_length += segment_length;
    }

    return {reference_line.back().x, reference_line.back().y};
}

std::pair<double, double> RenderWidget::calculate_lane_offset(
    Road const & road,
    int32_t lane_id,
    double position_s)
{
    if (road.lane_sections.empty())
    {
        return {0.0, 0.0};
    }

    for (auto const & lane_section : road.lane_sections)
    {
        if (position_s >= lane_section.s_start && position_s <= lane_section.s_end)
        {
            for (auto const & lane : lane_section.lanes)
            {
                if (lane.lane_id == lane_id)
                {
                    double lane_width = lane.width.empty() ? 3.0 : lane.width[0].a;
                    double offset = lane_width * 0.5;

                    if (lane_id > 0)
                    {
                        return {0.0, offset};
                    }
                    else if (lane_id < 0)
                    {
                        return {0.0, -offset};
                    }
                    break;
                }
            }
        }
    }

    return {0.0, 0.0};
}

void RenderWidget::wheelEvent(QWheelEvent * event)
{
    double delta = event->angleDelta().y();

    if (delta > 0)
    {
        m_scale *= m_zoom_factor;
    }
    else if (delta < 0)
    {
        m_scale /= m_zoom_factor;
    }

    m_scale = std::max(m_min_scale, std::min(m_scale, m_max_scale));

    update();
    event->accept();
}

void RenderWidget::mousePressEvent(QMouseEvent * event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_is_panning = true;
        m_last_mouse_position = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
    }
    else if (event->button() == Qt::RightButton)
    {
        m_is_rotating = true;
        m_last_mouse_position = event->pos();
        setCursor(Qt::CrossCursor);
        event->accept();
    }
}

void RenderWidget::mouseMoveEvent(QMouseEvent * event)
{
    if (m_is_panning)
    {
        QPoint delta = event->pos() - m_last_mouse_position;
        m_offset_x += delta.x();
        m_offset_y += delta.y();
        m_last_mouse_position = event->pos();
        update();
        event->accept();
    }
    else if (m_is_rotating)
    {
        QPoint delta = event->pos() - m_last_mouse_position;
        m_rotation += delta.x() * 0.5;

        while (m_rotation >= 360.0)
        {
            m_rotation -= 360.0;
        }
        while (m_rotation < 0.0)
        {
            m_rotation += 360.0;
        }

        m_last_mouse_position = event->pos();
        update();
        event->accept();
    }
}

void RenderWidget::mouseReleaseEvent(QMouseEvent * event)
{
    if (event->button() == Qt::LeftButton && m_is_panning)
    {
        m_is_panning = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
    }
    else if (event->button() == Qt::RightButton && m_is_rotating)
    {
        m_is_rotating = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
    }
}

void RenderWidget::keyPressEvent(QKeyEvent * event)
{
    bool handled = true;
    double pan_step = 20.0;

    switch (event->key())
    {
    case Qt::Key_Left:
        m_offset_x -= pan_step;
        break;
    case Qt::Key_Right:
        m_offset_x += pan_step;
        break;
    case Qt::Key_Up:
        m_offset_y -= pan_step;
        break;
    case Qt::Key_Down:
        m_offset_y += pan_step;
        break;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
        m_scale *= m_zoom_factor;
        m_scale = std::min(m_scale, m_max_scale);
        break;
    case Qt::Key_Minus:
    case Qt::Key_Underscore:
        m_scale /= m_zoom_factor;
        m_scale = std::max(m_scale, m_min_scale);
        break;
    case Qt::Key_R:
        if (event->modifiers() & Qt::ShiftModifier)
        {
            reset_camera();
        }
        else
        {
            m_rotation += 15.0;
            if (m_rotation >= 360.0)
            {
                m_rotation -= 360.0;
            }
        }
        break;
    case Qt::Key_0:
        m_rotation = 0.0;
        break;
    default:
        handled = false;
        break;
    }

    if (handled)
    {
        update();
        event->accept();
    }
    else
    {
        QWidget::keyPressEvent(event);
    }
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
