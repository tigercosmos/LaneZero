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
LaneZero: A traffic simulation library

This package provides Python bindings for the LaneZero C++ library.
"""

from . import _core
from . import viewer

# Export core classes
Vehicle = _core.Vehicle
VehicleType = _core.VehicleType
Point = _core.Point
WidthCoefficients = _core.WidthCoefficients
Lane = _core.Lane
LaneSection = _core.LaneSection
RoadLink = _core.RoadLink
Road = _core.Road
MapMeta = _core.MapMeta
Map = _core.Map
Simulation = _core.Simulation

# Export physics engine classes
PhysicsEngineType = _core.PhysicsEngineType
RigidBodyState = _core.RigidBodyState
VehicleControl = _core.VehicleControl
VehiclePhysicsParameters = _core.VehiclePhysicsParameters

# Export scenario classes
WorldPosition = _core.WorldPosition
LanePosition = _core.LanePosition
RoadPosition = _core.RoadPosition
EntityDimensions = _core.EntityDimensions
EntityPerformance = _core.EntityPerformance
EntityType = _core.EntityType
ScenarioEntity = _core.ScenarioEntity
ConditionType = _core.ConditionType
ComparisonOperator = _core.ComparisonOperator
Condition = _core.Condition
TriggerRule = _core.TriggerRule
Trigger = _core.Trigger
ActionType = _core.ActionType
SpeedDynamics = _core.SpeedDynamics
TrajectoryPoint = _core.TrajectoryPoint
Trajectory = _core.Trajectory
Route = _core.Route
ScenarioAction = _core.ScenarioAction
EventPriority = _core.EventPriority
EventState = _core.EventState
Event = _core.Event
Maneuver = _core.Maneuver
Act = _core.Act
Storyboard = _core.Storyboard
ScenarioMeta = _core.ScenarioMeta
ScenarioInit = _core.ScenarioInit
Goal = _core.Goal
Scenario = _core.Scenario
SimpleScenario = _core.SimpleScenario
WorldState = _core.WorldState

# Export planner classes
BehaviorState = _core.BehaviorState
BehaviorDecision = _core.BehaviorDecision
BehaviorPlanner = _core.BehaviorPlanner
MotionTrajectory = _core.MotionTrajectory
MotionPlanner = _core.MotionPlanner

__all__ = [
    "Vehicle",
    "VehicleType",
    "Point",
    "WidthCoefficients",
    "Lane",
    "LaneSection",
    "RoadLink",
    "Road",
    "MapMeta",
    "Map",
    "Simulation",
    "PhysicsEngineType",
    "RigidBodyState",
    "VehicleControl",
    "VehiclePhysicsParameters",
    "WorldPosition",
    "LanePosition",
    "RoadPosition",
    "EntityDimensions",
    "EntityPerformance",
    "EntityType",
    "ScenarioEntity",
    "ConditionType",
    "ComparisonOperator",
    "Condition",
    "TriggerRule",
    "Trigger",
    "ActionType",
    "SpeedDynamics",
    "TrajectoryPoint",
    "Trajectory",
    "Route",
    "ScenarioAction",
    "EventPriority",
    "EventState",
    "Event",
    "Maneuver",
    "Act",
    "Storyboard",
    "ScenarioMeta",
    "ScenarioInit",
    "Goal",
    "Scenario",
    "SimpleScenario",
    "WorldState",
    "BehaviorState",
    "BehaviorDecision",
    "BehaviorPlanner",
    "MotionTrajectory",
    "MotionPlanner",
    "viewer",
]

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
