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

#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace LaneZero
{

struct Point
{
    double x = 0.0;
    double y = 0.0;
    std::optional<double> z;

    Point() = default;
    Point(double x_value, double y_value);
    Point(double x_value, double y_value, double z_value);
};

struct WidthCoefficients
{
    double s_offset = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
    std::optional<double> valid_length;

    WidthCoefficients() = default;
    WidthCoefficients(double s_offset_value, double a_value, double b_value = 0.0, double c_value = 0.0, double d_value = 0.0);
};

struct Lane
{
    int32_t lane_id = 0;
    std::string type;
    std::optional<std::string> side;
    std::optional<bool> level;
    std::optional<double> speed_limit;
    std::vector<WidthCoefficients> width;
    std::map<std::string, std::string> attributes;

    Lane() = default;
    Lane(int32_t id, std::string const & lane_type);
};

struct LaneSection
{
    double s_start = 0.0;
    double s_end = 0.0;
    std::vector<Lane> lanes;

    LaneSection() = default;
    LaneSection(double start, double end);
};

struct RoadLink
{
    std::string road_id;
    std::string contact_point;

    RoadLink() = default;
    RoadLink(std::string const & id, std::string const & contact);
};

struct Road
{
    std::string id;
    double length = 0.0;
    std::vector<Point> reference_line;
    std::vector<LaneSection> lane_sections;
    std::optional<RoadLink> successor;
    std::optional<RoadLink> predecessor;
    std::optional<double> speed_limit;

    Road() = default;
    Road(std::string const & road_id, double road_length);
};

struct MapMeta
{
    std::string name;
    std::string version;
    std::optional<std::string> coordinate_system;
    std::map<std::string, std::string> units;
    std::map<std::string, std::string> additional_properties;

    MapMeta() = default;
    MapMeta(std::string const & map_name, std::string const & map_version);
};

class Map
{
public:
    Map() = default;
    Map(Map const &) = default;
    Map(Map &&) = default;
    Map & operator=(Map const &) = default;
    Map & operator=(Map &&) = default;
    ~Map() = default;

    MapMeta const & meta() const { return m_meta; }
    MapMeta & meta() { return m_meta; }

    std::vector<Road> const & roads() const { return m_roads; }
    std::vector<Road> & roads() { return m_roads; }

    void initialize_highway(int32_t num_lanes, double lane_length);

    std::string to_json_string() const;
    void from_json_string(std::string const & json_string);

    void load_from_file(std::string const & file_path);
    void save_to_file(std::string const & file_path) const;

private:
    MapMeta m_meta;
    std::vector<Road> m_roads;
};

} // namespace LaneZero

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
