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

#include <Map.h>

#include <fstream>
#include <stdexcept>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

namespace LaneZero
{

Point::Point(double x_value, double y_value)
    : x(x_value)
    , y(y_value)
{
}

Point::Point(double x_value, double y_value, double z_value)
    : x(x_value)
    , y(y_value)
    , z(z_value)
{
}

WidthCoefficients::WidthCoefficients(double s_offset_value, double a_value, double b_value, double c_value, double d_value)
    : s_offset(s_offset_value)
    , a(a_value)
    , b(b_value)
    , c(c_value)
    , d(d_value)
{
}

Lane::Lane(int32_t id, std::string const & lane_type)
    : lane_id(id)
    , type(lane_type)
{
}

LaneSection::LaneSection(double start, double end)
    : s_start(start)
    , s_end(end)
{
}

RoadLink::RoadLink(std::string const & id, std::string const & contact)
    : road_id(id)
    , contact_point(contact)
{
}

Road::Road(std::string const & road_id, double road_length)
    : id(road_id)
    , length(road_length)
{
}

MapMeta::MapMeta(std::string const & map_name, std::string const & map_version)
    : name(map_name)
    , version(map_version)
{
}

void Map::initialize_highway(int32_t num_lanes, double lane_length)
{
    m_meta = MapMeta("highway", "1.0");
    m_roads.clear();
    Road road("highway_road", lane_length);
    road.reference_line.emplace_back(0.0, 0.0);
    road.reference_line.emplace_back(lane_length, 0.0);
    LaneSection lane_section(0.0, lane_length);
    for (int32_t i = 0; i < num_lanes; ++i)
    {
        Lane lane(i, "driving");
        lane.side = "right";
        WidthCoefficients width_coefficients(0.0, 3.5);
        lane.width.push_back(width_coefficients);
        lane.speed_limit = 33.33;
        lane_section.lanes.push_back(lane);
    }
    road.lane_sections.push_back(lane_section);
    road.speed_limit = 33.33;
    m_roads.push_back(road);
}

std::string Map::to_json_string() const
{
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();

    rapidjson::Value meta_obj(rapidjson::kObjectType);
    {
        rapidjson::Value name_val;
        name_val.SetString(m_meta.name.c_str(), static_cast<rapidjson::SizeType>(m_meta.name.length()), allocator);
        meta_obj.AddMember("name", name_val, allocator);

        rapidjson::Value version_val;
        version_val.SetString(m_meta.version.c_str(), static_cast<rapidjson::SizeType>(m_meta.version.length()), allocator);
        meta_obj.AddMember("version", version_val, allocator);

        if (m_meta.coordinate_system.has_value())
        {
            rapidjson::Value cs_val;
            cs_val.SetString(m_meta.coordinate_system.value().c_str(), static_cast<rapidjson::SizeType>(m_meta.coordinate_system.value().length()), allocator);
            meta_obj.AddMember("coordinate_system", cs_val, allocator);
        }

        if (!m_meta.units.empty())
        {
            rapidjson::Value units_obj(rapidjson::kObjectType);
            for (auto const & [key, value] : m_meta.units)
            {
                rapidjson::Value key_val;
                key_val.SetString(key.c_str(), static_cast<rapidjson::SizeType>(key.length()), allocator);
                rapidjson::Value val_val;
                val_val.SetString(value.c_str(), static_cast<rapidjson::SizeType>(value.length()), allocator);
                units_obj.AddMember(key_val, val_val, allocator);
            }
            meta_obj.AddMember("units", units_obj, allocator);
        }

        for (auto const & [key, value] : m_meta.additional_properties)
        {
            rapidjson::Value key_val;
            key_val.SetString(key.c_str(), static_cast<rapidjson::SizeType>(key.length()), allocator);
            rapidjson::Value val_val;
            val_val.SetString(value.c_str(), static_cast<rapidjson::SizeType>(value.length()), allocator);
            meta_obj.AddMember(key_val, val_val, allocator);
        }
    }

    rapidjson::Value roads_array(rapidjson::kArrayType);
    for (auto const & road : m_roads)
    {
        rapidjson::Value road_obj(rapidjson::kObjectType);

        rapidjson::Value road_id_val;
        road_id_val.SetString(road.id.c_str(), static_cast<rapidjson::SizeType>(road.id.length()), allocator);
        road_obj.AddMember("id", road_id_val, allocator);
        road_obj.AddMember("length", road.length, allocator);

        rapidjson::Value ref_line_array(rapidjson::kArrayType);
        for (auto const & point : road.reference_line)
        {
            rapidjson::Value point_obj(rapidjson::kObjectType);
            point_obj.AddMember("x", point.x, allocator);
            point_obj.AddMember("y", point.y, allocator);
            if (point.z.has_value())
            {
                point_obj.AddMember("z", point.z.value(), allocator);
            }
            ref_line_array.PushBack(point_obj, allocator);
        }
        road_obj.AddMember("reference_line", ref_line_array, allocator);

        rapidjson::Value lane_sections_array(rapidjson::kArrayType);
        for (auto const & lane_section : road.lane_sections)
        {
            rapidjson::Value lane_section_obj(rapidjson::kObjectType);
            lane_section_obj.AddMember("s_start", lane_section.s_start, allocator);
            lane_section_obj.AddMember("s_end", lane_section.s_end, allocator);

            rapidjson::Value lanes_array(rapidjson::kArrayType);
            for (auto const & lane : lane_section.lanes)
            {
                rapidjson::Value lane_obj(rapidjson::kObjectType);
                lane_obj.AddMember("lane_id", lane.lane_id, allocator);

                rapidjson::Value lane_type_val;
                lane_type_val.SetString(lane.type.c_str(), static_cast<rapidjson::SizeType>(lane.type.length()), allocator);
                lane_obj.AddMember("type", lane_type_val, allocator);

                if (lane.side.has_value())
                {
                    rapidjson::Value side_val;
                    side_val.SetString(lane.side.value().c_str(), static_cast<rapidjson::SizeType>(lane.side.value().length()), allocator);
                    lane_obj.AddMember("side", side_val, allocator);
                }

                if (lane.level.has_value())
                {
                    lane_obj.AddMember("level", lane.level.value(), allocator);
                }

                if (lane.speed_limit.has_value())
                {
                    lane_obj.AddMember("speed_limit", lane.speed_limit.value(), allocator);
                }

                if (!lane.width.empty())
                {
                    rapidjson::Value width_array(rapidjson::kArrayType);
                    for (auto const & width_coeff : lane.width)
                    {
                        rapidjson::Value width_obj(rapidjson::kObjectType);
                        width_obj.AddMember("s_offset", width_coeff.s_offset, allocator);
                        width_obj.AddMember("a", width_coeff.a, allocator);
                        width_obj.AddMember("b", width_coeff.b, allocator);
                        width_obj.AddMember("c", width_coeff.c, allocator);
                        width_obj.AddMember("d", width_coeff.d, allocator);
                        if (width_coeff.valid_length.has_value())
                        {
                            width_obj.AddMember("valid_length", width_coeff.valid_length.value(), allocator);
                        }
                        width_array.PushBack(width_obj, allocator);
                    }
                    lane_obj.AddMember("width", width_array, allocator);
                }

                if (!lane.attributes.empty())
                {
                    rapidjson::Value attr_obj(rapidjson::kObjectType);
                    for (auto const & [key, value] : lane.attributes)
                    {
                        rapidjson::Value key_val;
                        key_val.SetString(key.c_str(), static_cast<rapidjson::SizeType>(key.length()), allocator);
                        rapidjson::Value val_val;
                        val_val.SetString(value.c_str(), static_cast<rapidjson::SizeType>(value.length()), allocator);
                        attr_obj.AddMember(key_val, val_val, allocator);
                    }
                    lane_obj.AddMember("attributes", attr_obj, allocator);
                }

                lanes_array.PushBack(lane_obj, allocator);
            }
            lane_section_obj.AddMember("lanes", lanes_array, allocator);
            lane_sections_array.PushBack(lane_section_obj, allocator);
        }
        road_obj.AddMember("lane_sections", lane_sections_array, allocator);

        if (road.successor.has_value())
        {
            rapidjson::Value successor_obj(rapidjson::kObjectType);
            rapidjson::Value road_id_val;
            road_id_val.SetString(road.successor.value().road_id.c_str(), static_cast<rapidjson::SizeType>(road.successor.value().road_id.length()), allocator);
            successor_obj.AddMember("road_id", road_id_val, allocator);

            rapidjson::Value contact_val;
            contact_val.SetString(road.successor.value().contact_point.c_str(), static_cast<rapidjson::SizeType>(road.successor.value().contact_point.length()), allocator);
            successor_obj.AddMember("contact_point", contact_val, allocator);
            road_obj.AddMember("successor", successor_obj, allocator);
        }

        if (road.predecessor.has_value())
        {
            rapidjson::Value predecessor_obj(rapidjson::kObjectType);
            rapidjson::Value road_id_val;
            road_id_val.SetString(road.predecessor.value().road_id.c_str(), static_cast<rapidjson::SizeType>(road.predecessor.value().road_id.length()), allocator);
            predecessor_obj.AddMember("road_id", road_id_val, allocator);

            rapidjson::Value contact_val;
            contact_val.SetString(road.predecessor.value().contact_point.c_str(), static_cast<rapidjson::SizeType>(road.predecessor.value().contact_point.length()), allocator);
            predecessor_obj.AddMember("contact_point", contact_val, allocator);
            road_obj.AddMember("predecessor", predecessor_obj, allocator);
        }

        if (road.speed_limit.has_value())
        {
            road_obj.AddMember("speed_limit", road.speed_limit.value(), allocator);
        }

        roads_array.PushBack(road_obj, allocator);
    }

    rapidjson::Value map_obj(rapidjson::kObjectType);
    map_obj.AddMember("meta", meta_obj, allocator);
    map_obj.AddMember("roads", roads_array, allocator);

    rapidjson::Value root_obj(rapidjson::kObjectType);
    root_obj.AddMember("map", map_obj, allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    root_obj.Accept(writer);

    return buffer.GetString();
}

void Map::from_json_string(std::string const & json_string)
{
    rapidjson::Document document;
    document.Parse(json_string.c_str());

    if (document.HasParseError())
    {
        throw std::runtime_error("Failed to parse JSON string");
    }

    if (!document.IsObject() || !document.HasMember("map"))
    {
        throw std::runtime_error("JSON must have 'map' root object");
    }

    auto const & map_obj = document["map"];

    if (!map_obj.HasMember("meta") || !map_obj["meta"].IsObject())
    {
        throw std::runtime_error("Map must have 'meta' object");
    }

    auto const & meta_obj = map_obj["meta"];
    m_meta = MapMeta();
    m_meta.name = meta_obj["name"].GetString();
    m_meta.version = meta_obj["version"].GetString();

    if (meta_obj.HasMember("coordinate_system") && meta_obj["coordinate_system"].IsString())
    {
        m_meta.coordinate_system = meta_obj["coordinate_system"].GetString();
    }

    if (meta_obj.HasMember("units") && meta_obj["units"].IsObject())
    {
        for (auto it = meta_obj["units"].MemberBegin(); it != meta_obj["units"].MemberEnd(); ++it)
        {
            if (it->value.IsString())
            {
                m_meta.units[it->name.GetString()] = it->value.GetString();
            }
        }
    }

    m_roads.clear();
    auto const & roads_array = map_obj["roads"];
    for (rapidjson::SizeType i = 0; i < roads_array.Size(); ++i)
    {
        auto const & road_obj = roads_array[i];
        Road road;
        road.id = road_obj["id"].GetString();
        road.length = road_obj["length"].GetDouble();

        auto const & ref_line_array = road_obj["reference_line"];
        for (rapidjson::SizeType j = 0; j < ref_line_array.Size(); ++j)
        {
            auto const & point_obj = ref_line_array[j];
            Point point;
            point.x = point_obj["x"].GetDouble();
            point.y = point_obj["y"].GetDouble();
            if (point_obj.HasMember("z") && point_obj["z"].IsNumber())
            {
                point.z = point_obj["z"].GetDouble();
            }
            road.reference_line.push_back(point);
        }

        auto const & lane_sections_array = road_obj["lane_sections"];
        for (rapidjson::SizeType j = 0; j < lane_sections_array.Size(); ++j)
        {
            auto const & lane_section_obj = lane_sections_array[j];
            LaneSection lane_section;
            lane_section.s_start = lane_section_obj["s_start"].GetDouble();
            lane_section.s_end = lane_section_obj["s_end"].GetDouble();

            auto const & lanes_array = lane_section_obj["lanes"];
            for (rapidjson::SizeType k = 0; k < lanes_array.Size(); ++k)
            {
                auto const & lane_obj = lanes_array[k];
                Lane lane;
                lane.lane_id = lane_obj["lane_id"].GetInt();
                lane.type = lane_obj["type"].GetString();

                if (lane_obj.HasMember("side") && lane_obj["side"].IsString())
                {
                    lane.side = lane_obj["side"].GetString();
                }

                if (lane_obj.HasMember("level") && lane_obj["level"].IsBool())
                {
                    lane.level = lane_obj["level"].GetBool();
                }

                if (lane_obj.HasMember("speed_limit") && lane_obj["speed_limit"].IsNumber())
                {
                    lane.speed_limit = lane_obj["speed_limit"].GetDouble();
                }

                if (lane_obj.HasMember("width") && lane_obj["width"].IsArray())
                {
                    auto const & width_array = lane_obj["width"];
                    for (rapidjson::SizeType m = 0; m < width_array.Size(); ++m)
                    {
                        auto const & width_obj = width_array[m];
                        WidthCoefficients width_coeff;
                        width_coeff.s_offset = width_obj["s_offset"].GetDouble();
                        width_coeff.a = width_obj["a"].GetDouble();
                        width_coeff.b = width_obj.HasMember("b") ? width_obj["b"].GetDouble() : 0.0;
                        width_coeff.c = width_obj.HasMember("c") ? width_obj["c"].GetDouble() : 0.0;
                        width_coeff.d = width_obj.HasMember("d") ? width_obj["d"].GetDouble() : 0.0;
                        if (width_obj.HasMember("valid_length") && width_obj["valid_length"].IsNumber())
                        {
                            width_coeff.valid_length = width_obj["valid_length"].GetDouble();
                        }
                        lane.width.push_back(width_coeff);
                    }
                }

                if (lane_obj.HasMember("attributes") && lane_obj["attributes"].IsObject())
                {
                    for (auto it = lane_obj["attributes"].MemberBegin(); it != lane_obj["attributes"].MemberEnd(); ++it)
                    {
                        if (it->value.IsString())
                        {
                            lane.attributes[it->name.GetString()] = it->value.GetString();
                        }
                    }
                }

                lane_section.lanes.push_back(lane);
            }
            road.lane_sections.push_back(lane_section);
        }

        if (road_obj.HasMember("successor") && road_obj["successor"].IsObject())
        {
            auto const & successor_obj = road_obj["successor"];
            RoadLink successor;
            successor.road_id = successor_obj["road_id"].GetString();
            successor.contact_point = successor_obj["contact_point"].GetString();
            road.successor = successor;
        }

        if (road_obj.HasMember("predecessor") && road_obj["predecessor"].IsObject())
        {
            auto const & predecessor_obj = road_obj["predecessor"];
            RoadLink predecessor;
            predecessor.road_id = predecessor_obj["road_id"].GetString();
            predecessor.contact_point = predecessor_obj["contact_point"].GetString();
            road.predecessor = predecessor;
        }

        if (road_obj.HasMember("speed_limit") && road_obj["speed_limit"].IsNumber())
        {
            road.speed_limit = road_obj["speed_limit"].GetDouble();
        }

        m_roads.push_back(road);
    }
}

void Map::load_from_file(std::string const & file_path)
{
    std::ifstream input_file(file_path);
    if (!input_file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + file_path);
    }

    rapidjson::IStreamWrapper input_stream_wrapper(input_file);
    rapidjson::Document document;
    document.ParseStream(input_stream_wrapper);

    if (document.HasParseError())
    {
        throw std::runtime_error("Failed to parse JSON file: " + file_path);
    }

    std::string json_string;
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);

    from_json_string(buffer.GetString());
}

void Map::save_to_file(std::string const & file_path) const
{
    std::ofstream output_file(file_path);
    if (!output_file.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + file_path);
    }

    std::string json_string = to_json_string();
    output_file << json_string;
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
