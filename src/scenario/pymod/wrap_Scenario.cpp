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

#include <LaneZero/scenario/Scenario.h>
#include <LaneZero/scenario/Goal.h>

namespace py = pybind11;

void wrap_Scenario(py::module & module)
{
    py::enum_<LaneZero::PositionType>(module, "PositionType")
        .value("World", LaneZero::PositionType::World)
        .value("Lane", LaneZero::PositionType::Lane)
        .value("Road", LaneZero::PositionType::Road);

    py::class_<LaneZero::WorldPosition>(module, "WorldPosition")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def(py::init<double, double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"), py::arg("heading"))
        .def_readwrite("x", &LaneZero::WorldPosition::x)
        .def_readwrite("y", &LaneZero::WorldPosition::y)
        .def_readwrite("z", &LaneZero::WorldPosition::z)
        .def_readwrite("heading", &LaneZero::WorldPosition::heading);

    py::class_<LaneZero::LanePosition>(module, "LanePosition")
        .def(py::init<>())
        .def(py::init<std::string const &, int32_t, double>(), py::arg("road_id"), py::arg("lane_id"), py::arg("s"))
        .def_readwrite("road_id", &LaneZero::LanePosition::road_id)
        .def_readwrite("lane_id", &LaneZero::LanePosition::lane_id)
        .def_readwrite("s", &LaneZero::LanePosition::s)
        .def_readwrite("offset", &LaneZero::LanePosition::offset)
        .def_readwrite("heading", &LaneZero::LanePosition::heading);

    py::class_<LaneZero::RoadPosition>(module, "RoadPosition")
        .def(py::init<>())
        .def(py::init<std::string const &, double>(), py::arg("road_id"), py::arg("s"))
        .def_readwrite("road_id", &LaneZero::RoadPosition::road_id)
        .def_readwrite("s", &LaneZero::RoadPosition::s)
        .def_readwrite("t", &LaneZero::RoadPosition::t)
        .def_readwrite("heading", &LaneZero::RoadPosition::heading);

    py::class_<LaneZero::EntityDimensions>(module, "EntityDimensions")
        .def(py::init<>())
        .def(py::init<double, double, double>(), py::arg("length"), py::arg("width"), py::arg("height"))
        .def_readwrite("length", &LaneZero::EntityDimensions::length)
        .def_readwrite("width", &LaneZero::EntityDimensions::width)
        .def_readwrite("height", &LaneZero::EntityDimensions::height);

    py::class_<LaneZero::EntityPerformance>(module, "EntityPerformance")
        .def(py::init<>())
        .def_readwrite("max_speed", &LaneZero::EntityPerformance::max_speed)
        .def_readwrite("max_accel", &LaneZero::EntityPerformance::max_accel)
        .def_readwrite("max_decel", &LaneZero::EntityPerformance::max_decel);

    py::enum_<LaneZero::EntityType>(module, "EntityType")
        .value("Vehicle", LaneZero::EntityType::Vehicle)
        .value("Pedestrian", LaneZero::EntityType::Pedestrian)
        .value("MiscObject", LaneZero::EntityType::MiscObject);

    py::class_<LaneZero::ScenarioEntity>(module, "ScenarioEntity")
        .def(py::init<>())
        .def(py::init<std::string const &, LaneZero::EntityType>(), py::arg("name"), py::arg("type"))
        .def_readwrite("name", &LaneZero::ScenarioEntity::name)
        .def_readwrite("type", &LaneZero::ScenarioEntity::type)
        .def_readwrite("category", &LaneZero::ScenarioEntity::category)
        .def_readwrite("model", &LaneZero::ScenarioEntity::model)
        .def_readwrite("dimensions", &LaneZero::ScenarioEntity::dimensions)
        .def_readwrite("performance", &LaneZero::ScenarioEntity::performance)
        .def_readwrite("properties", &LaneZero::ScenarioEntity::properties);

    py::enum_<LaneZero::ConditionType>(module, "ConditionType")
        .value("Time", LaneZero::ConditionType::Time)
        .value("DistanceToEntity", LaneZero::ConditionType::DistanceToEntity)
        .value("ReachPosition", LaneZero::ConditionType::ReachPosition)
        .value("SpeedThreshold", LaneZero::ConditionType::SpeedThreshold)
        .value("Parameter", LaneZero::ConditionType::Parameter);

    py::enum_<LaneZero::ComparisonOperator>(module, "ComparisonOperator")
        .value("LessThan", LaneZero::ComparisonOperator::LessThan)
        .value("LessEqual", LaneZero::ComparisonOperator::LessEqual)
        .value("GreaterThan", LaneZero::ComparisonOperator::GreaterThan)
        .value("GreaterEqual", LaneZero::ComparisonOperator::GreaterEqual)
        .value("Equal", LaneZero::ComparisonOperator::Equal)
        .value("NotEqual", LaneZero::ComparisonOperator::NotEqual);

    py::class_<LaneZero::Condition>(module, "Condition")
        .def(py::init<>())
        .def(py::init<LaneZero::ConditionType>(), py::arg("type"))
        .def_readwrite("type", &LaneZero::Condition::type)
        .def_readwrite("value", &LaneZero::Condition::value)
        .def_readwrite("entity", &LaneZero::Condition::entity)
        .def_readwrite("target_entity", &LaneZero::Condition::target_entity)
        .def_readwrite("distance", &LaneZero::Condition::distance)
        .def_readwrite("comparison", &LaneZero::Condition::comparison)
        .def_readwrite("tolerance", &LaneZero::Condition::tolerance)
        .def_readwrite("speed", &LaneZero::Condition::speed)
        .def_readwrite("parameter", &LaneZero::Condition::parameter);

    py::enum_<LaneZero::TriggerRule>(module, "TriggerRule")
        .value("Any", LaneZero::TriggerRule::Any)
        .value("All", LaneZero::TriggerRule::All);

    py::class_<LaneZero::Trigger>(module, "Trigger")
        .def(py::init<>())
        .def_readwrite("conditions", &LaneZero::Trigger::conditions)
        .def_readwrite("rule", &LaneZero::Trigger::rule);

    py::enum_<LaneZero::ActionType>(module, "ActionType")
        .value("Teleport", LaneZero::ActionType::Teleport)
        .value("Speed", LaneZero::ActionType::Speed)
        .value("LaneChange", LaneZero::ActionType::LaneChange)
        .value("FollowTrajectory", LaneZero::ActionType::FollowTrajectory)
        .value("AssignRoute", LaneZero::ActionType::AssignRoute)
        .value("Wait", LaneZero::ActionType::Wait);

    py::class_<LaneZero::SpeedDynamics>(module, "SpeedDynamics")
        .def(py::init<>())
        .def(py::init<std::string const &, double, std::string const &>(), py::arg("shape"), py::arg("value"), py::arg("dimension"))
        .def_readwrite("shape", &LaneZero::SpeedDynamics::shape)
        .def_readwrite("value", &LaneZero::SpeedDynamics::value)
        .def_readwrite("dimension", &LaneZero::SpeedDynamics::dimension);

    py::class_<LaneZero::TrajectoryPoint>(module, "TrajectoryPoint")
        .def(py::init<>())
        .def_readwrite("time", &LaneZero::TrajectoryPoint::time)
        .def_readwrite("speed", &LaneZero::TrajectoryPoint::speed);

    py::class_<LaneZero::Trajectory>(module, "Trajectory")
        .def(py::init<>())
        .def_readwrite("points", &LaneZero::Trajectory::points)
        .def_readwrite("closed", &LaneZero::Trajectory::closed);

    py::class_<LaneZero::Route>(module, "Route")
        .def(py::init<>())
        .def_readwrite("waypoints", &LaneZero::Route::waypoints);

    py::class_<LaneZero::ScenarioAction>(module, "ScenarioAction")
        .def(py::init<>())
        .def(py::init<LaneZero::ActionType, std::string const &>(), py::arg("type"), py::arg("entity"))
        .def_readwrite("type", &LaneZero::ScenarioAction::type)
        .def_readwrite("entity", &LaneZero::ScenarioAction::entity)
        .def_readwrite("position", &LaneZero::ScenarioAction::position)
        .def_readwrite("target_speed", &LaneZero::ScenarioAction::target_speed)
        .def_readwrite("dynamics", &LaneZero::ScenarioAction::dynamics)
        .def_readwrite("target_lane_id", &LaneZero::ScenarioAction::target_lane_id)
        .def_readwrite("duration", &LaneZero::ScenarioAction::duration)
        .def_readwrite("trajectory", &LaneZero::ScenarioAction::trajectory)
        .def_readwrite("time_reference", &LaneZero::ScenarioAction::time_reference)
        .def_readwrite("route", &LaneZero::ScenarioAction::route)
        .def_readwrite("wait_time", &LaneZero::ScenarioAction::wait_time);

    py::enum_<LaneZero::EventPriority>(module, "EventPriority")
        .value("Overwrite", LaneZero::EventPriority::Overwrite)
        .value("Parallel", LaneZero::EventPriority::Parallel)
        .value("Skip", LaneZero::EventPriority::Skip);

    py::enum_<LaneZero::EventState>(module, "EventState")
        .value("Standby", LaneZero::EventState::Standby)
        .value("Running", LaneZero::EventState::Running)
        .value("Complete", LaneZero::EventState::Complete);

    py::class_<LaneZero::Event>(module, "Event")
        .def(py::init<>())
        .def(py::init<std::string const &>(), py::arg("name"))
        .def_readwrite("name", &LaneZero::Event::name)
        .def_readwrite("priority", &LaneZero::Event::priority)
        .def_readwrite("start_trigger", &LaneZero::Event::start_trigger)
        .def_readwrite("actions", &LaneZero::Event::actions)
        .def_readwrite("state", &LaneZero::Event::state)
        .def_readwrite("start_time", &LaneZero::Event::start_time);

    py::class_<LaneZero::Maneuver>(module, "Maneuver")
        .def(py::init<>())
        .def(py::init<std::string const &>(), py::arg("name"))
        .def_readwrite("name", &LaneZero::Maneuver::name)
        .def_readwrite("actors", &LaneZero::Maneuver::actors)
        .def_readwrite("events", &LaneZero::Maneuver::events);

    py::class_<LaneZero::Act>(module, "Act")
        .def(py::init<>())
        .def(py::init<std::string const &>(), py::arg("name"))
        .def_readwrite("name", &LaneZero::Act::name)
        .def_readwrite("start_trigger", &LaneZero::Act::start_trigger)
        .def_readwrite("stop_trigger", &LaneZero::Act::stop_trigger)
        .def_readwrite("maneuvers", &LaneZero::Act::maneuvers)
        .def_readwrite("is_active", &LaneZero::Act::is_active);

    py::class_<LaneZero::Storyboard>(module, "Storyboard")
        .def(py::init<>())
        .def_readwrite("stop_trigger", &LaneZero::Storyboard::stop_trigger)
        .def_readwrite("acts", &LaneZero::Storyboard::acts);

    py::class_<LaneZero::ScenarioMeta>(module, "ScenarioMeta")
        .def(py::init<>())
        .def(py::init<std::string const &, std::string const &>(), py::arg("name"), py::arg("version"))
        .def_readwrite("name", &LaneZero::ScenarioMeta::name)
        .def_readwrite("version", &LaneZero::ScenarioMeta::version)
        .def_readwrite("description", &LaneZero::ScenarioMeta::description)
        .def_readwrite("author", &LaneZero::ScenarioMeta::author)
        .def_readwrite("date", &LaneZero::ScenarioMeta::date)
        .def_readwrite("map_ref", &LaneZero::ScenarioMeta::map_ref)
        .def_readwrite("units", &LaneZero::ScenarioMeta::units);

    py::class_<LaneZero::ScenarioInit>(module, "ScenarioInit")
        .def(py::init<>())
        .def_readwrite("actions", &LaneZero::ScenarioInit::actions);

    py::class_<LaneZero::Goal>(module, "Goal")
        .def(py::init<>())
        .def(py::init<int32_t, double>(), py::arg("target_lane_id"), py::arg("desired_speed_mps"))
        .def_readwrite("target_lane_id", &LaneZero::Goal::target_lane_id)
        .def_readwrite("desired_speed_mps", &LaneZero::Goal::desired_speed_mps);

    py::class_<LaneZero::Scenario>(module, "Scenario")
        .def(py::init<>())
        .def("update_goal", &LaneZero::Scenario::update_goal, py::arg("world_state"))
        .def("update", &LaneZero::Scenario::update, py::arg("world_state"))
        .def("initialize", &LaneZero::Scenario::initialize, py::arg("world_state"))
        .def("is_complete", &LaneZero::Scenario::is_complete, py::arg("world_state"))
        .def("meta", py::overload_cast<>(&LaneZero::Scenario::meta), py::return_value_policy::reference_internal)
        .def("entities", py::overload_cast<>(&LaneZero::Scenario::entities), py::return_value_policy::reference_internal)
        .def("init", py::overload_cast<>(&LaneZero::Scenario::init), py::return_value_policy::reference_internal)
        .def("storyboard", py::overload_cast<>(&LaneZero::Scenario::storyboard), py::return_value_policy::reference_internal)
        .def("parameters", py::overload_cast<>(&LaneZero::Scenario::parameters), py::return_value_policy::reference_internal)
        .def("to_json_string", &LaneZero::Scenario::to_json_string)
        .def("from_json_string", &LaneZero::Scenario::from_json_string, py::arg("json_string"))
        .def("load_from_file", &LaneZero::Scenario::load_from_file, py::arg("file_path"))
        .def("save_to_file", &LaneZero::Scenario::save_to_file, py::arg("file_path"))
        .def("set_entity_vehicle_id", &LaneZero::Scenario::set_entity_vehicle_id, py::arg("entity_name"), py::arg("vehicle_id"))
        .def("get_entity_vehicle_id", &LaneZero::Scenario::get_entity_vehicle_id, py::arg("entity_name"))
        .def("clear_entity_vehicles", &LaneZero::Scenario::clear_entity_vehicles);

    py::class_<LaneZero::SimpleScenario, LaneZero::Scenario>(module, "SimpleScenario")
        .def(py::init<>())
        .def("update_goal", &LaneZero::SimpleScenario::update_goal, py::arg("world_state"));
}

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
