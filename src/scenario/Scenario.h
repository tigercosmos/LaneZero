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

#include <LaneZero/scenario/Goal.h>
#include <LaneZero/simulation/WorldState.h>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace LaneZero
{

enum class PositionType
{
    World,
    Lane,
    Road
}; /* end enum class PositionType */

struct WorldPosition
{
    double x = 0.0;
    double y = 0.0;
    std::optional<double> z;
    std::optional<double> heading;

    WorldPosition() = default;
    WorldPosition(double x_value, double y_value);
    WorldPosition(double x_value, double y_value, double z_value, double heading_value);
}; /* end struct WorldPosition */

struct LanePosition
{
    std::string road_id;
    int32_t lane_id = 0;
    double s = 0.0;
    std::optional<double> offset;
    std::optional<double> heading;

    LanePosition() = default;
    LanePosition(std::string const & road, int32_t lane, double s_value);
}; /* end struct LanePosition */

struct RoadPosition
{
    std::string road_id;
    double s = 0.0;
    std::optional<double> t;
    std::optional<double> heading;

    RoadPosition() = default;
    RoadPosition(std::string const & road, double s_value);
}; /* end struct RoadPosition */

using Position = std::variant<WorldPosition, LanePosition, RoadPosition>;

struct EntityDimensions
{
    double length = 4.5;
    double width = 1.8;
    double height = 1.5;

    EntityDimensions() = default;
    EntityDimensions(double len, double wid, double hei);
}; /* end struct EntityDimensions */

struct EntityPerformance
{
    std::optional<double> max_speed;
    std::optional<double> max_accel;
    std::optional<double> max_decel;

    EntityPerformance() = default;
}; /* end struct EntityPerformance */

enum class EntityType
{
    Vehicle,
    Pedestrian,
    MiscObject
}; /* end enum class EntityType */

struct ScenarioEntity
{
    std::string name;
    EntityType type = EntityType::Vehicle;
    std::string category;
    std::optional<std::string> model;
    EntityDimensions dimensions;
    EntityPerformance performance;
    std::map<std::string, std::string> properties;

    ScenarioEntity() = default;
    ScenarioEntity(std::string const & entity_name, EntityType entity_type);
}; /* end struct ScenarioEntity */

enum class ConditionType
{
    Time,
    DistanceToEntity,
    ReachPosition,
    SpeedThreshold,
    Parameter
}; /* end enum class ConditionType */

enum class ComparisonOperator
{
    LessThan,
    LessEqual,
    GreaterThan,
    GreaterEqual,
    Equal,
    NotEqual
}; /* end enum class ComparisonOperator */

struct Condition
{
    ConditionType type = ConditionType::Time;
    std::optional<double> value;
    std::optional<std::string> entity;
    std::optional<std::string> target_entity;
    std::optional<double> distance;
    std::optional<ComparisonOperator> comparison;
    std::optional<Position> position;
    std::optional<double> tolerance;
    std::optional<double> speed;
    std::optional<std::string> parameter;

    Condition() = default;
    Condition(ConditionType cond_type);

    bool evaluate(WorldState const & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids) const;
}; /* end struct Condition */

enum class TriggerRule
{
    Any,
    All
}; /* end enum class TriggerRule */

struct Trigger
{
    std::vector<Condition> conditions;
    TriggerRule rule = TriggerRule::Any;

    Trigger() = default;

    bool is_triggered(WorldState const & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids) const;
}; /* end struct Trigger */

enum class ActionType
{
    Teleport,
    Speed,
    LaneChange,
    FollowTrajectory,
    AssignRoute,
    Wait
}; /* end enum class ActionType */

struct SpeedDynamics
{
    std::string shape = "linear";
    double value = 0.0;
    std::string dimension = "time";

    SpeedDynamics() = default;
    SpeedDynamics(std::string const & dyn_shape, double val, std::string const & dim);
}; /* end struct SpeedDynamics */

struct TrajectoryPoint
{
    std::optional<double> time;
    Position position;
    std::optional<double> speed;

    TrajectoryPoint() = default;
}; /* end struct TrajectoryPoint */

struct Trajectory
{
    std::vector<TrajectoryPoint> points;
    bool closed = false;

    Trajectory() = default;
}; /* end struct Trajectory */

struct Route
{
    std::vector<Position> waypoints;

    Route() = default;
}; /* end struct Route */

struct ScenarioAction
{
    ActionType type = ActionType::Teleport;
    std::string entity;
    std::optional<Position> position;
    std::optional<double> target_speed;
    std::optional<SpeedDynamics> dynamics;
    std::optional<int32_t> target_lane_id;
    std::optional<double> duration;
    std::optional<Trajectory> trajectory;
    std::optional<std::string> time_reference;
    std::optional<Route> route;
    std::optional<double> wait_time;

    ScenarioAction() = default;
    ScenarioAction(ActionType action_type, std::string const & entity_name);

    void execute(WorldState & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids) const;
}; /* end struct ScenarioAction */

enum class EventPriority
{
    Overwrite,
    Parallel,
    Skip
}; /* end enum class EventPriority */

enum class EventState
{
    Standby,
    Running,
    Complete
}; /* end enum class EventState */

struct Event
{
    std::string name;
    EventPriority priority = EventPriority::Overwrite;
    Trigger start_trigger;
    std::vector<ScenarioAction> actions;
    EventState state = EventState::Standby;
    double start_time = 0.0;

    Event() = default;
    Event(std::string const & event_name);

    void update(WorldState & world_state, std::map<std::string, int32_t> const & entity_vehicle_ids);
}; /* end struct Event */

struct Maneuver
{
    std::string name;
    std::vector<std::string> actors;
    std::vector<Event> events;

    Maneuver() = default;
    Maneuver(std::string const & maneuver_name);
}; /* end struct Maneuver */

struct Act
{
    std::string name;
    std::optional<Trigger> start_trigger;
    std::optional<Trigger> stop_trigger;
    std::vector<Maneuver> maneuvers;
    bool is_active = false;

    Act() = default;
    Act(std::string const & act_name);
}; /* end struct Act */

struct Storyboard
{
    std::optional<Trigger> stop_trigger;
    std::vector<Act> acts;

    Storyboard() = default;
}; /* end struct Storyboard */

struct ScenarioMeta
{
    std::string name;
    std::string version;
    std::optional<std::string> description;
    std::optional<std::string> author;
    std::optional<std::string> date;
    std::map<std::string, std::string> map_ref;
    std::map<std::string, std::string> units;

    ScenarioMeta() = default;
    ScenarioMeta(std::string const & scenario_name, std::string const & scenario_version);
}; /* end struct ScenarioMeta */

struct ScenarioInit
{
    std::vector<ScenarioAction> actions;

    ScenarioInit() = default;
}; /* end struct ScenarioInit */

class Scenario
{
public:
    Scenario() = default;
    Scenario(Scenario const &) = default;
    Scenario(Scenario &&) = default;
    Scenario & operator=(Scenario const &) = default;
    Scenario & operator=(Scenario &&) = default;
    virtual ~Scenario() = default;

    virtual Goal update_goal(WorldState const & world_state);
    virtual void update(WorldState & world_state);
    virtual void initialize(WorldState & world_state);
    virtual bool is_complete(WorldState const & world_state) const;

    ScenarioMeta const & meta() const { return m_meta; }
    ScenarioMeta & meta() { return m_meta; }

    std::vector<ScenarioEntity> const & entities() const { return m_entities; }
    std::vector<ScenarioEntity> & entities() { return m_entities; }

    ScenarioInit const & init() const { return m_init; }
    ScenarioInit & init() { return m_init; }

    Storyboard const & storyboard() const { return m_storyboard; }
    Storyboard & storyboard() { return m_storyboard; }

    std::map<std::string, double> const & parameters() const { return m_parameters; }
    std::map<std::string, double> & parameters() { return m_parameters; }

    std::string to_json_string() const;
    void from_json_string(std::string const & json_string);

    void load_from_file(std::string const & file_path);
    void save_to_file(std::string const & file_path) const;

    void set_entity_vehicle_id(std::string const & entity_name, int32_t vehicle_id);
    int32_t get_entity_vehicle_id(std::string const & entity_name) const;
    void clear_entity_vehicles();

private:
    ScenarioMeta m_meta;
    std::vector<ScenarioEntity> m_entities;
    std::map<std::string, double> m_parameters;
    ScenarioInit m_init;
    Storyboard m_storyboard;
    bool m_initialized = false;
    std::map<std::string, int32_t> m_entity_vehicle_ids;
}; /* end class Scenario */

class SimpleScenario : public Scenario
{
public:
    SimpleScenario() = default;
    SimpleScenario(SimpleScenario const &) = default;
    SimpleScenario(SimpleScenario &&) = default;
    SimpleScenario & operator=(SimpleScenario const &) = default;
    SimpleScenario & operator=(SimpleScenario &&) = default;
    ~SimpleScenario() override = default;

    Goal update_goal(WorldState const & world_state) override;
}; /* end class SimpleScenario */

} /* end namespace LaneZero */

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
