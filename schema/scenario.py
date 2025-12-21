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

SIMPLE_OPENSCENARIO_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "SimpleOpenScenario Format",
    "type": "object",
    "required": ["scenario"],
    "properties": {
        "scenario": {
            "type": "object",
            "required": ["meta", "entities", "init", "storyboard"],
            "properties": {
                "meta": {
                    "type": "object",
                    "required": ["name", "version"],
                    "properties": {
                        "name": {"type": "string"},
                        "version": {"type": "string"},
                        "description": {"type": "string"},
                        "author": {"type": "string"},
                        "date": {"type": "string"},
                        "map_ref": {
                            "type": "object",
                            "properties": {
                                "name": {"type": "string"},
                                "version": {"type": "string"},
                            },
                            "additionalProperties": True,
                        },
                        "coordinate_system": {"type": "string"},
                        "units": {
                            "type": "object",
                            "properties": {
                                "distance": {"type": "string"},
                                "speed": {"type": "string"},
                                "angle": {"type": "string"},
                                "time": {"type": "string"},
                            },
                            "additionalProperties": True,
                        },
                    },
                    "additionalProperties": True,
                },
                "parameters": {
                    "type": "object",
                    "description": "Scenario parameters. Values can be number/string/bool.",
                    "additionalProperties": {
                        "anyOf": [
                            {"type": "number"},
                            {"type": "string"},
                            {"type": "boolean"},
                            {"type": "null"},
                        ]
                    },
                },
                "entities": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                        "type": "object",
                        "required": ["name", "type"],
                        "properties": {
                            "name": {"type": "string"},
                            "type": {
                                "type": "string",
                                "enum": ["vehicle", "pedestrian", "misc_object"],
                            },
                            "category": {"type": "string"},
                            "model": {"type": "string"},
                            "dimensions": {
                                "type": "object",
                                "properties": {
                                    "length": {"type": "number", "minimum": 0},
                                    "width": {"type": "number", "minimum": 0},
                                    "height": {"type": "number", "minimum": 0},
                                },
                                "additionalProperties": False,
                            },
                            "performance": {
                                "type": "object",
                                "properties": {
                                    "max_speed": {"type": "number", "minimum": 0},
                                    "max_accel": {"type": "number", "minimum": 0},
                                    "max_decel": {"type": "number", "minimum": 0},
                                },
                                "additionalProperties": False,
                            },
                            "properties": {
                                "type": "object",
                                "description": "Free-form metadata for simulators.",
                                "additionalProperties": True,
                            },
                        },
                        "additionalProperties": False,
                    },
                },
                "init": {
                    "type": "object",
                    "required": ["actions"],
                    "properties": {
                        "actions": {
                            "type": "array",
                            "minItems": 1,
                            "items": {"$ref": "#/definitions/action"},
                        }
                    },
                    "additionalProperties": False,
                },
                "storyboard": {
                    "type": "object",
                    "required": ["acts"],
                    "properties": {
                        "stop_trigger": {"$ref": "#/definitions/trigger"},
                        "acts": {
                            "type": "array",
                            "minItems": 1,
                            "items": {
                                "type": "object",
                                "required": ["name", "maneuvers"],
                                "properties": {
                                    "name": {"type": "string"},
                                    "start_trigger": {"$ref": "#/definitions/trigger"},
                                    "stop_trigger": {"$ref": "#/definitions/trigger"},
                                    "maneuvers": {
                                        "type": "array",
                                        "minItems": 1,
                                        "items": {
                                            "type": "object",
                                            "required": ["name", "events"],
                                            "properties": {
                                                "name": {"type": "string"},
                                                "actors": {
                                                    "type": "array",
                                                    "minItems": 1,
                                                    "items": {"type": "string"},
                                                    "description": "Entity names participating in this maneuver.",
                                                },
                                                "events": {
                                                    "type": "array",
                                                    "minItems": 1,
                                                    "items": {
                                                        "type": "object",
                                                        "required": [
                                                            "name",
                                                            "start_trigger",
                                                            "actions",
                                                        ],
                                                        "properties": {
                                                            "name": {"type": "string"},
                                                            "priority": {
                                                                "type": "string",
                                                                "enum": [
                                                                    "overwrite",
                                                                    "parallel",
                                                                    "skip",
                                                                ],
                                                            },
                                                            "start_trigger": {
                                                                "$ref": "#/definitions/trigger"
                                                            },
                                                            "actions": {
                                                                "type": "array",
                                                                "minItems": 1,
                                                                "items": {
                                                                    "$ref": "#/definitions/action"
                                                                },
                                                            },
                                                        },
                                                        "additionalProperties": False,
                                                    },
                                                },
                                            },
                                            "additionalProperties": False,
                                        },
                                    },
                                },
                                "additionalProperties": False,
                            },
                        },
                    },
                    "additionalProperties": False,
                },
            },
            "additionalProperties": False,
        }
    },
    "definitions": {
        "position": {
            "type": "object",
            "description": "One-of: world, lane, or road position.",
            "oneOf": [
                {"$ref": "#/definitions/world_position"},
                {"$ref": "#/definitions/lane_position"},
                {"$ref": "#/definitions/road_position"},
            ],
        },
        "world_position": {
            "type": "object",
            "required": ["type", "x", "y"],
            "properties": {
                "type": {"type": "string", "const": "world"},
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
                "heading": {
                    "type": "number",
                    "description": "Radians or degrees; interpret via meta.units.angle.",
                },
            },
            "additionalProperties": False,
        },
        "lane_position": {
            "type": "object",
            "required": ["type", "road_id", "lane_id", "s"],
            "properties": {
                "type": {"type": "string", "const": "lane"},
                "road_id": {"type": "string"},
                "lane_id": {"type": "integer"},
                "s": {"type": "number", "minimum": 0},
                "offset": {
                    "type": "number",
                    "description": "Lateral offset from lane center (optional).",
                },
                "heading": {"type": "number"},
            },
            "additionalProperties": False,
        },
        "road_position": {
            "type": "object",
            "required": ["type", "road_id", "s"],
            "properties": {
                "type": {"type": "string", "const": "road"},
                "road_id": {"type": "string"},
                "s": {"type": "number", "minimum": 0},
                "t": {
                    "type": "number",
                    "description": "Lateral offset from reference line.",
                },
                "heading": {"type": "number"},
            },
            "additionalProperties": False,
        },
        "trajectory": {
            "type": "object",
            "required": ["points"],
            "properties": {
                "points": {
                    "type": "array",
                    "minItems": 2,
                    "items": {
                        "type": "object",
                        "required": ["position"],
                        "properties": {
                            "time": {"type": "number", "minimum": 0},
                            "position": {"$ref": "#/definitions/position"},
                            "speed": {"type": "number", "minimum": 0},
                        },
                        "additionalProperties": False,
                    },
                },
                "closed": {"type": "boolean"},
            },
            "additionalProperties": False,
        },
        "trigger": {
            "type": "object",
            "description": "A trigger is an OR of conditions; each condition is a small typed rule.",
            "required": ["conditions"],
            "properties": {
                "conditions": {
                    "type": "array",
                    "minItems": 1,
                    "items": {"$ref": "#/definitions/condition"},
                },
                "rule": {
                    "type": "string",
                    "enum": ["any", "all"],
                    "default": "any",
                    "description": "If 'any', triggers when any condition is true; if 'all', requires all.",
                },
            },
            "additionalProperties": False,
        },
        "condition": {
            "type": "object",
            "required": ["type"],
            "properties": {
                "type": {
                    "type": "string",
                    "enum": [
                        "time",
                        "distance_to_entity",
                        "reach_position",
                        "speed_threshold",
                        "parameter",
                    ],
                },
                # time condition
                "value": {"type": "number"},
                # distance_to_entity
                "entity": {"type": "string"},
                "target_entity": {"type": "string"},
                "distance": {"type": "number", "minimum": 0},
                "comparison": {"type": "string", "enum": ["<", "<=", ">", ">=", "=="]},
                # reach_position
                "position": {"$ref": "#/definitions/position"},
                "tolerance": {"type": "number", "minimum": 0},
                # speed_threshold
                "speed": {"type": "number", "minimum": 0},
                # parameter condition
                "parameter": {"type": "string"},
            },
            "additionalProperties": False,
            "allOf": [
                {
                    "if": {"properties": {"type": {"const": "time"}}},
                    "then": {
                        "required": ["value", "comparison"],
                        "properties": {
                            "comparison": {
                                "type": "string",
                                "enum": ["<", "<=", ">", ">=", "=="],
                            },
                        },
                    },
                },
                {
                    "if": {"properties": {"type": {"const": "distance_to_entity"}}},
                    "then": {
                        "required": [
                            "entity",
                            "target_entity",
                            "distance",
                            "comparison",
                        ],
                    },
                },
                {
                    "if": {"properties": {"type": {"const": "reach_position"}}},
                    "then": {
                        "required": ["entity", "position"],
                    },
                },
                {
                    "if": {"properties": {"type": {"const": "speed_threshold"}}},
                    "then": {
                        "required": ["entity", "speed", "comparison"],
                    },
                },
                {
                    "if": {"properties": {"type": {"const": "parameter"}}},
                    "then": {
                        "required": ["parameter", "value", "comparison"],
                        "properties": {
                            "comparison": {
                                "type": "string",
                                "enum": ["<", "<=", ">", ">=", "==", "!="],
                            },
                        },
                    },
                },
            ],
        },
        "action": {
            "type": "object",
            "required": ["type", "entity"],
            "properties": {
                "type": {
                    "type": "string",
                    "enum": [
                        "teleport",
                        "speed",
                        "lane_change",
                        "follow_trajectory",
                        "assign_route",
                        "wait",
                    ],
                },
                "entity": {"type": "string"},
                # teleport
                "position": {"$ref": "#/definitions/position"},
                # speed action
                "target_speed": {"type": "number", "minimum": 0},
                "dynamics": {
                    "type": "object",
                    "properties": {
                        "shape": {"type": "string", "enum": ["step", "linear"]},
                        "value": {
                            "type": "number",
                            "minimum": 0,
                            "description": "Accel (m/s^2) or time (s) depending on use.",
                        },
                        "dimension": {"type": "string", "enum": ["time", "rate"]},
                    },
                    "additionalProperties": False,
                },
                # lane_change
                "target_lane_id": {"type": "integer"},
                "duration": {"type": "number", "minimum": 0},
                # follow_trajectory
                "trajectory": {"$ref": "#/definitions/trajectory"},
                "time_reference": {
                    "type": "string",
                    "enum": ["absolute", "relative"],
                    "default": "relative",
                },
                # assign_route
                "route": {
                    "type": "object",
                    "required": ["waypoints"],
                    "properties": {
                        "waypoints": {
                            "type": "array",
                            "minItems": 2,
                            "items": {"$ref": "#/definitions/position"},
                        }
                    },
                    "additionalProperties": False,
                },
                # wait
                "wait_time": {"type": "number", "minimum": 0},
            },
            "additionalProperties": False,
            "allOf": [
                {
                    "if": {"properties": {"type": {"const": "teleport"}}},
                    "then": {"required": ["position"]},
                },
                {
                    "if": {"properties": {"type": {"const": "speed"}}},
                    "then": {"required": ["target_speed"]},
                },
                {
                    "if": {"properties": {"type": {"const": "lane_change"}}},
                    "then": {"required": ["target_lane_id"]},
                },
                {
                    "if": {"properties": {"type": {"const": "follow_trajectory"}}},
                    "then": {"required": ["trajectory"]},
                },
                {
                    "if": {"properties": {"type": {"const": "assign_route"}}},
                    "then": {"required": ["route"]},
                },
                {
                    "if": {"properties": {"type": {"const": "wait"}}},
                    "then": {"required": ["wait_time"]},
                },
            ],
        },
    },
}
