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

#include <LaneZero/scenario/Scenario.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

namespace LaneZero
{

WorldPosition::WorldPosition(double x_value, double y_value)
    : x(x_value)
    , y(y_value)
{
}

WorldPosition::WorldPosition(double x_value, double y_value, double z_value, double heading_value)
    : x(x_value)
    , y(y_value)
    , z(z_value)
    , heading(heading_value)
{
}

LanePosition::LanePosition(std::string const & road, int32_t lane, double s_value)
    : road_id(road)
    , lane_id(lane)
    , s(s_value)
{
}

RoadPosition::RoadPosition(std::string const & road, double s_value)
    : road_id(road)
    , s(s_value)
{
}

EntityDimensions::EntityDimensions(double len, double wid, double hei)
    : length(len)
    , width(wid)
    , height(hei)
{
}

ScenarioEntity::ScenarioEntity(std::string const & entity_name, EntityType entity_type)
    : name(entity_name)
    , type(entity_type)
{
}

Condition::Condition(ConditionType cond_type)
    : type(cond_type)
{
}

bool Condition::evaluate(WorldState const & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids) const
{
    switch (type)
    {
        case ConditionType::Time:
        {
            if (!value.has_value() || !comparison.has_value())
            {
                return false;
            }

            double current_time = world_state.current_time_s;
            double target_value = value.value();

            switch (comparison.value())
            {
                case ComparisonOperator::LessThan:
                    return current_time < target_value;
                case ComparisonOperator::LessEqual:
                    return current_time <= target_value;
                case ComparisonOperator::GreaterThan:
                    return current_time > target_value;
                case ComparisonOperator::GreaterEqual:
                    return current_time >= target_value;
                case ComparisonOperator::Equal:
                    return std::abs(current_time - target_value) < 0.01;
                case ComparisonOperator::NotEqual:
                    return std::abs(current_time - target_value) >= 0.01;
                default:
                    return false;
            }
        }
        case ConditionType::DistanceToEntity:
        {
            if (!entity.has_value() || !target_entity.has_value() || !distance.has_value() || !comparison.has_value())
            {
                return false;
            }

            Vehicle * entity_vehicle = nullptr;
            Vehicle * target_vehicle = nullptr;

            auto it1 = entity_vehicle_ids.find(entity.value());
            if (it1 != entity_vehicle_ids.end())
            {
                int32_t vehicle_id = it1->second;
                if (world_state.ego_vehicle && world_state.ego_vehicle->id == vehicle_id)
                {
                    entity_vehicle = world_state.ego_vehicle.get();
                }
                else
                {
                    for (auto const & v : world_state.vehicles)
                    {
                        if (v && v->id == vehicle_id)
                        {
                            entity_vehicle = v.get();
                            break;
                        }
                    }
                }
            }

            auto it2 = entity_vehicle_ids.find(target_entity.value());
            if (it2 != entity_vehicle_ids.end())
            {
                int32_t vehicle_id = it2->second;
                if (world_state.ego_vehicle && world_state.ego_vehicle->id == vehicle_id)
                {
                    target_vehicle = world_state.ego_vehicle.get();
                }
                else
                {
                    for (auto const & v : world_state.vehicles)
                    {
                        if (v && v->id == vehicle_id)
                        {
                            target_vehicle = v.get();
                            break;
                        }
                    }
                }
            }

            if (!entity_vehicle || !target_vehicle)
            {
                return false;
            }

            double distance_value = std::abs(entity_vehicle->position_s_m - target_vehicle->position_s_m);

            switch (comparison.value())
            {
                case ComparisonOperator::LessThan:
                    return distance_value < distance.value();
                case ComparisonOperator::LessEqual:
                    return distance_value <= distance.value();
                case ComparisonOperator::GreaterThan:
                    return distance_value > distance.value();
                case ComparisonOperator::GreaterEqual:
                    return distance_value >= distance.value();
                case ComparisonOperator::Equal:
                    return std::abs(distance_value - distance.value()) < 0.1;
                default:
                    return false;
            }
        }
        default:
            return false;
    }
}

bool Trigger::is_triggered(WorldState const & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids) const
{
    if (conditions.empty())
    {
        return false;
    }

    if (rule == TriggerRule::All)
    {
        return std::all_of(conditions.begin(), conditions.end(), [&world_state, &entity_vehicle_ids](Condition const & cond) {
            return cond.evaluate(world_state, entity_vehicle_ids);
        });
    }
    else
    {
        return std::any_of(conditions.begin(), conditions.end(), [&world_state, &entity_vehicle_ids](Condition const & cond) {
            return cond.evaluate(world_state, entity_vehicle_ids);
        });
    }
}

SpeedDynamics::SpeedDynamics(std::string const & dyn_shape, double val, std::string const & dim)
    : shape(dyn_shape)
    , value(val)
    , dimension(dim)
{
}

ScenarioAction::ScenarioAction(ActionType action_type, std::string const & entity_name)
    : type(action_type)
    , entity(entity_name)
{
}

void ScenarioAction::execute(WorldState & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids) const
{
    Vehicle * target_vehicle = nullptr;

    auto it = entity_vehicle_ids.find(entity);
    if (it != entity_vehicle_ids.end())
    {
        int32_t vehicle_id = it->second;
        if (world_state.ego_vehicle && world_state.ego_vehicle->id == vehicle_id)
        {
            target_vehicle = world_state.ego_vehicle.get();
        }
        else
        {
            for (auto const & v : world_state.vehicles)
            {
                if (v && v->id == vehicle_id)
                {
                    target_vehicle = v.get();
                    break;
                }
            }
        }
    }

    if (!target_vehicle)
    {
        return;
    }

    switch (type)
    {
        case ActionType::Teleport:
        {
            if (position.has_value())
            {
                if (std::holds_alternative<LanePosition>(position.value()))
                {
                    LanePosition const & lane_pos = std::get<LanePosition>(position.value());
                    target_vehicle->position_s_m = lane_pos.s;
                    target_vehicle->current_lane_id = lane_pos.lane_id;
                }
            }
            break;
        }
        case ActionType::Speed:
        {
            if (target_speed.has_value())
            {
                target_vehicle->velocity_mps = target_speed.value();
            }
            break;
        }
        case ActionType::LaneChange:
        {
            if (target_lane_id.has_value())
            {
                target_vehicle->current_lane_id = target_lane_id.value();
            }
            break;
        }
        default:
            break;
    }
}

Event::Event(std::string const & event_name)
    : name(event_name)
{
}

void Event::update(WorldState & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids)
{
    switch (state)
    {
        case EventState::Standby:
        {
            if (start_trigger.is_triggered(world_state, entity_vehicle_ids))
            {
                state = EventState::Running;
                start_time = world_state.current_time_s;
                for (auto & action : actions)
                {
                    action.execute(world_state, entity_vehicle_ids);
                }
            }
            break;
        }
        case EventState::Running:
        {
            state = EventState::Complete;
            break;
        }
        case EventState::Complete:
        default:
            break;
    }
}

Maneuver::Maneuver(std::string const & maneuver_name)
    : name(maneuver_name)
{
}

Act::Act(std::string const & act_name)
    : name(act_name)
{
}

ScenarioMeta::ScenarioMeta(std::string const & scenario_name, std::string const & scenario_version)
    : name(scenario_name)
    , version(scenario_version)
{
}

Goal Scenario::update_goal(WorldState const & world_state)
{
    return Goal();
}

void Scenario::update(WorldState & world_state)
{
    if (!m_initialized)
    {
        initialize(world_state);
    }

    for (auto & act : m_storyboard.acts)
    {
        if (!act.is_active)
        {
            if (!act.start_trigger.has_value() || act.start_trigger.value().is_triggered(world_state, m_entity_vehicle_ids))
            {
                act.is_active = true;
            }
        }

        if (act.is_active)
        {
            for (auto & maneuver : act.maneuvers)
            {
                for (auto & event : maneuver.events)
                {
                    event.update(world_state, m_entity_vehicle_ids);
                }
            }

            if (act.stop_trigger.has_value() && act.stop_trigger.value().is_triggered(world_state, m_entity_vehicle_ids))
            {
                act.is_active = false;
            }
        }
    }
}

void Scenario::initialize(WorldState & world_state)
{
    for (auto & action : m_init.actions)
    {
        action.execute(world_state, m_entity_vehicle_ids);
    }
    m_initialized = true;
}

bool Scenario::is_complete(WorldState const & world_state) const
{
    if (m_storyboard.stop_trigger.has_value())
    {
        return m_storyboard.stop_trigger.value().is_triggered(world_state, m_entity_vehicle_ids);
    }
    return false;
}

std::string Scenario::to_json_string() const
{
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);

    writer.StartObject();
    writer.Key("scenario");
    writer.StartObject();

    writer.Key("meta");
    writer.StartObject();
    writer.Key("name");
    writer.String(m_meta.name.c_str());
    writer.Key("version");
    writer.String(m_meta.version.c_str());
    if (m_meta.description.has_value())
    {
        writer.Key("description");
        writer.String(m_meta.description.value().c_str());
    }
    writer.EndObject();

    writer.EndObject();
    writer.EndObject();

    return buffer.GetString();
}

void Scenario::from_json_string(std::string const & json_string)
{
    rapidjson::Document document;
    document.Parse(json_string.c_str());

    if (document.HasParseError())
    {
        throw std::runtime_error("Failed to parse scenario JSON");
    }

    if (!document.HasMember("scenario") || !document["scenario"].IsObject())
    {
        throw std::runtime_error("Invalid scenario JSON format");
    }

    rapidjson::Value const & scenario_obj = document["scenario"];

    if (scenario_obj.HasMember("meta") && scenario_obj["meta"].IsObject())
    {
        rapidjson::Value const & meta_obj = scenario_obj["meta"];
        if (meta_obj.HasMember("name") && meta_obj["name"].IsString())
        {
            m_meta.name = meta_obj["name"].GetString();
        }
        if (meta_obj.HasMember("version") && meta_obj["version"].IsString())
        {
            m_meta.version = meta_obj["version"].GetString();
        }
        if (meta_obj.HasMember("description") && meta_obj["description"].IsString())
        {
            m_meta.description = meta_obj["description"].GetString();
        }
        if (meta_obj.HasMember("map_ref") && meta_obj["map_ref"].IsObject())
        {
            rapidjson::Value const & map_ref_obj = meta_obj["map_ref"];
            for (auto it = map_ref_obj.MemberBegin(); it != map_ref_obj.MemberEnd(); ++it)
            {
                if (it->value.IsString())
                {
                    m_meta.map_ref[it->name.GetString()] = it->value.GetString();
                }
            }
        }
        if (meta_obj.HasMember("units") && meta_obj["units"].IsObject())
        {
            rapidjson::Value const & units_obj = meta_obj["units"];
            for (auto it = units_obj.MemberBegin(); it != units_obj.MemberEnd(); ++it)
            {
                if (it->value.IsString())
                {
                    m_meta.units[it->name.GetString()] = it->value.GetString();
                }
            }
        }
    }

    if (scenario_obj.HasMember("entities") && scenario_obj["entities"].IsArray())
    {
        rapidjson::Value const & entities_array = scenario_obj["entities"];
        for (rapidjson::SizeType i = 0; i < entities_array.Size(); ++i)
        {
            rapidjson::Value const & entity_obj = entities_array[i];
            ScenarioEntity entity;

            if (entity_obj.HasMember("name") && entity_obj["name"].IsString())
            {
                entity.name = entity_obj["name"].GetString();
            }

            if (entity_obj.HasMember("type") && entity_obj["type"].IsString())
            {
                std::string type_str = entity_obj["type"].GetString();
                if (type_str == "vehicle")
                {
                    entity.type = EntityType::Vehicle;
                }
                else if (type_str == "pedestrian")
                {
                    entity.type = EntityType::Pedestrian;
                }
                else
                {
                    entity.type = EntityType::MiscObject;
                }
            }

            if (entity_obj.HasMember("category") && entity_obj["category"].IsString())
            {
                entity.category = entity_obj["category"].GetString();
            }

            if (entity_obj.HasMember("dimensions") && entity_obj["dimensions"].IsObject())
            {
                rapidjson::Value const & dim_obj = entity_obj["dimensions"];
                if (dim_obj.HasMember("length") && dim_obj["length"].IsNumber())
                {
                    entity.dimensions.length = dim_obj["length"].GetDouble();
                }
                if (dim_obj.HasMember("width") && dim_obj["width"].IsNumber())
                {
                    entity.dimensions.width = dim_obj["width"].GetDouble();
                }
                if (dim_obj.HasMember("height") && dim_obj["height"].IsNumber())
                {
                    entity.dimensions.height = dim_obj["height"].GetDouble();
                }
            }

            m_entities.push_back(entity);
        }
    }

    if (scenario_obj.HasMember("init") && scenario_obj["init"].IsObject())
    {
        rapidjson::Value const & init_obj = scenario_obj["init"];
        if (init_obj.HasMember("actions") && init_obj["actions"].IsArray())
        {
            rapidjson::Value const & actions_array = init_obj["actions"];
            for (rapidjson::SizeType i = 0; i < actions_array.Size(); ++i)
            {
                rapidjson::Value const & action_obj = actions_array[i];
                ScenarioAction action;

                if (action_obj.HasMember("type") && action_obj["type"].IsString())
                {
                    std::string type_str = action_obj["type"].GetString();
                    if (type_str == "teleport")
                    {
                        action.type = ActionType::Teleport;
                    }
                    else if (type_str == "speed")
                    {
                        action.type = ActionType::Speed;
                    }
                    else if (type_str == "lane_change")
                    {
                        action.type = ActionType::LaneChange;
                    }
                }

                if (action_obj.HasMember("entity") && action_obj["entity"].IsString())
                {
                    action.entity = action_obj["entity"].GetString();
                }

                if (action_obj.HasMember("position") && action_obj["position"].IsObject())
                {
                    rapidjson::Value const & pos_obj = action_obj["position"];
                    if (pos_obj.HasMember("type") && pos_obj["type"].IsString())
                    {
                        std::string pos_type = pos_obj["type"].GetString();
                        if (pos_type == "lane")
                        {
                            LanePosition lane_pos;
                            if (pos_obj.HasMember("road_id") && pos_obj["road_id"].IsString())
                            {
                                lane_pos.road_id = pos_obj["road_id"].GetString();
                            }
                            if (pos_obj.HasMember("lane_id") && pos_obj["lane_id"].IsInt())
                            {
                                lane_pos.lane_id = pos_obj["lane_id"].GetInt();
                            }
                            if (pos_obj.HasMember("s") && pos_obj["s"].IsNumber())
                            {
                                lane_pos.s = pos_obj["s"].GetDouble();
                            }
                            action.position = lane_pos;
                        }
                    }
                }

                if (action_obj.HasMember("target_speed") && action_obj["target_speed"].IsNumber())
                {
                    action.target_speed = action_obj["target_speed"].GetDouble();
                }

                if (action_obj.HasMember("target_lane_id") && action_obj["target_lane_id"].IsInt())
                {
                    action.target_lane_id = action_obj["target_lane_id"].GetInt();
                }

                m_init.actions.push_back(action);
            }
        }
    }

    if (scenario_obj.HasMember("storyboard") && scenario_obj["storyboard"].IsObject())
    {
        rapidjson::Value const & storyboard_obj = scenario_obj["storyboard"];

        if (storyboard_obj.HasMember("stop_trigger") && storyboard_obj["stop_trigger"].IsObject())
        {
            rapidjson::Value const & trigger_obj = storyboard_obj["stop_trigger"];
            Trigger stop_trigger;

            if (trigger_obj.HasMember("conditions") && trigger_obj["conditions"].IsArray())
            {
                rapidjson::Value const & conditions_array = trigger_obj["conditions"];
                for (rapidjson::SizeType i = 0; i < conditions_array.Size(); ++i)
                {
                    rapidjson::Value const & cond_obj = conditions_array[i];
                    Condition condition;

                    if (cond_obj.HasMember("type") && cond_obj["type"].IsString())
                    {
                        std::string type_str = cond_obj["type"].GetString();
                        if (type_str == "time")
                        {
                            condition.type = ConditionType::Time;
                        }
                        else if (type_str == "distance_to_entity")
                        {
                            condition.type = ConditionType::DistanceToEntity;
                        }
                    }

                    if (cond_obj.HasMember("value") && cond_obj["value"].IsNumber())
                    {
                        condition.value = cond_obj["value"].GetDouble();
                    }

                    if (cond_obj.HasMember("entity") && cond_obj["entity"].IsString())
                    {
                        condition.entity = cond_obj["entity"].GetString();
                    }

                    if (cond_obj.HasMember("target_entity") && cond_obj["target_entity"].IsString())
                    {
                        condition.target_entity = cond_obj["target_entity"].GetString();
                    }

                    if (cond_obj.HasMember("distance") && cond_obj["distance"].IsNumber())
                    {
                        condition.distance = cond_obj["distance"].GetDouble();
                    }

                    if (cond_obj.HasMember("comparison") && cond_obj["comparison"].IsString())
                    {
                        std::string comp_str = cond_obj["comparison"].GetString();
                        if (comp_str == "<")
                        {
                            condition.comparison = ComparisonOperator::LessThan;
                        }
                        else if (comp_str == "<=")
                        {
                            condition.comparison = ComparisonOperator::LessEqual;
                        }
                        else if (comp_str == ">")
                        {
                            condition.comparison = ComparisonOperator::GreaterThan;
                        }
                        else if (comp_str == ">=")
                        {
                            condition.comparison = ComparisonOperator::GreaterEqual;
                        }
                        else if (comp_str == "==")
                        {
                            condition.comparison = ComparisonOperator::Equal;
                        }
                    }

                    stop_trigger.conditions.push_back(condition);
                }
            }

            m_storyboard.stop_trigger = stop_trigger;
        }

        if (storyboard_obj.HasMember("acts") && storyboard_obj["acts"].IsArray())
        {
            rapidjson::Value const & acts_array = storyboard_obj["acts"];
            for (rapidjson::SizeType i = 0; i < acts_array.Size(); ++i)
            {
                rapidjson::Value const & act_obj = acts_array[i];
                Act act;

                if (act_obj.HasMember("name") && act_obj["name"].IsString())
                {
                    act.name = act_obj["name"].GetString();
                }

                if (act_obj.HasMember("maneuvers") && act_obj["maneuvers"].IsArray())
                {
                    rapidjson::Value const & maneuvers_array = act_obj["maneuvers"];
                    for (rapidjson::SizeType j = 0; j < maneuvers_array.Size(); ++j)
                    {
                        rapidjson::Value const & maneuver_obj = maneuvers_array[j];
                        Maneuver maneuver;

                        if (maneuver_obj.HasMember("name") && maneuver_obj["name"].IsString())
                        {
                            maneuver.name = maneuver_obj["name"].GetString();
                        }

                        if (maneuver_obj.HasMember("actors") && maneuver_obj["actors"].IsArray())
                        {
                            rapidjson::Value const & actors_array = maneuver_obj["actors"];
                            for (rapidjson::SizeType k = 0; k < actors_array.Size(); ++k)
                            {
                                if (actors_array[k].IsString())
                                {
                                    maneuver.actors.push_back(actors_array[k].GetString());
                                }
                            }
                        }

                        if (maneuver_obj.HasMember("events") && maneuver_obj["events"].IsArray())
                        {
                            rapidjson::Value const & events_array = maneuver_obj["events"];
                            for (rapidjson::SizeType k = 0; k < events_array.Size(); ++k)
                            {
                                rapidjson::Value const & event_obj = events_array[k];
                                Event event;

                                if (event_obj.HasMember("name") && event_obj["name"].IsString())
                                {
                                    event.name = event_obj["name"].GetString();
                                }

                                if (event_obj.HasMember("start_trigger") && event_obj["start_trigger"].IsObject())
                                {
                                    rapidjson::Value const & trigger_obj = event_obj["start_trigger"];
                                    Trigger start_trigger;

                                    if (trigger_obj.HasMember("conditions") && trigger_obj["conditions"].IsArray())
                                    {
                                        rapidjson::Value const & conditions_array = trigger_obj["conditions"];
                                        for (rapidjson::SizeType m = 0; m < conditions_array.Size(); ++m)
                                        {
                                            rapidjson::Value const & cond_obj = conditions_array[m];
                                            Condition condition;

                                            if (cond_obj.HasMember("type") && cond_obj["type"].IsString())
                                            {
                                                std::string type_str = cond_obj["type"].GetString();
                                                if (type_str == "time")
                                                {
                                                    condition.type = ConditionType::Time;
                                                }
                                                else if (type_str == "distance_to_entity")
                                                {
                                                    condition.type = ConditionType::DistanceToEntity;
                                                }
                                            }

                                            if (cond_obj.HasMember("value") && cond_obj["value"].IsNumber())
                                            {
                                                condition.value = cond_obj["value"].GetDouble();
                                            }

                                            if (cond_obj.HasMember("entity") && cond_obj["entity"].IsString())
                                            {
                                                condition.entity = cond_obj["entity"].GetString();
                                            }

                                            if (cond_obj.HasMember("target_entity") && cond_obj["target_entity"].IsString())
                                            {
                                                condition.target_entity = cond_obj["target_entity"].GetString();
                                            }

                                            if (cond_obj.HasMember("distance") && cond_obj["distance"].IsNumber())
                                            {
                                                condition.distance = cond_obj["distance"].GetDouble();
                                            }

                                            if (cond_obj.HasMember("comparison") && cond_obj["comparison"].IsString())
                                            {
                                                std::string comp_str = cond_obj["comparison"].GetString();
                                                if (comp_str == "<")
                                                {
                                                    condition.comparison = ComparisonOperator::LessThan;
                                                }
                                                else if (comp_str == "<=")
                                                {
                                                    condition.comparison = ComparisonOperator::LessEqual;
                                                }
                                                else if (comp_str == ">")
                                                {
                                                    condition.comparison = ComparisonOperator::GreaterThan;
                                                }
                                                else if (comp_str == ">=")
                                                {
                                                    condition.comparison = ComparisonOperator::GreaterEqual;
                                                }
                                                else if (comp_str == "==")
                                                {
                                                    condition.comparison = ComparisonOperator::Equal;
                                                }
                                            }

                                            start_trigger.conditions.push_back(condition);
                                        }
                                    }

                                    event.start_trigger = start_trigger;
                                }

                                if (event_obj.HasMember("actions") && event_obj["actions"].IsArray())
                                {
                                    rapidjson::Value const & actions_array = event_obj["actions"];
                                    for (rapidjson::SizeType m = 0; m < actions_array.Size(); ++m)
                                    {
                                        rapidjson::Value const & action_obj = actions_array[m];
                                        ScenarioAction action;

                                        if (action_obj.HasMember("type") && action_obj["type"].IsString())
                                        {
                                            std::string type_str = action_obj["type"].GetString();
                                            if (type_str == "lane_change")
                                            {
                                                action.type = ActionType::LaneChange;
                                            }
                                        }

                                        if (action_obj.HasMember("entity") && action_obj["entity"].IsString())
                                        {
                                            action.entity = action_obj["entity"].GetString();
                                        }

                                        if (action_obj.HasMember("target_lane_id") && action_obj["target_lane_id"].IsInt())
                                        {
                                            action.target_lane_id = action_obj["target_lane_id"].GetInt();
                                        }

                                        if (action_obj.HasMember("duration") && action_obj["duration"].IsNumber())
                                        {
                                            action.duration = action_obj["duration"].GetDouble();
                                        }

                                        event.actions.push_back(action);
                                    }
                                }

                                maneuver.events.push_back(event);
                            }
                        }

                        act.maneuvers.push_back(maneuver);
                    }
                }

                m_storyboard.acts.push_back(act);
            }
        }
    }
}

void Scenario::load_from_file(std::string const & file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open scenario file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    from_json_string(buffer.str());
}

void Scenario::save_to_file(std::string const & file_path) const
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot create scenario file: " + file_path);
    }

    file << to_json_string();
}

Goal SimpleScenario::update_goal(WorldState const & world_state)
{
    if (world_state.ego_vehicle)
    {
        int32_t current_lane = world_state.ego_vehicle->current_lane_id;
        double speed_limit = 30.0;

        if (!world_state.world_map.roads().empty())
        {
            Road const & road = world_state.world_map.roads()[0];
            if (road.speed_limit.has_value())
            {
                speed_limit = road.speed_limit.value();
            }
        }

        return Goal(current_lane, speed_limit);
    }

    return Goal(0, 30.0);
}

void Scenario::set_entity_vehicle_id(std::string const & entity_name, int32_t vehicle_id)
{
    m_entity_vehicle_ids[entity_name] = vehicle_id;
}

int32_t Scenario::get_entity_vehicle_id(std::string const & entity_name) const
{
    auto it = m_entity_vehicle_ids.find(entity_name);
    if (it != m_entity_vehicle_ids.end())
    {
        return it->second;
    }
    return -1;
}

void Scenario::clear_entity_vehicles()
{
    m_entity_vehicle_ids.clear();
}

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
