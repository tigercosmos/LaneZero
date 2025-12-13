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

# SimpleOpen Map Format JSON Schema (Draft-07) in Python dict form

SIMPLE_OPENDRIVE_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "SimpleOpen Map Format",
    "type": "object",
    "required": ["map"],
    "properties": {
        "map": {
            "type": "object",
            "required": ["meta", "roads"],
            "properties": {
                "meta": {
                    "type": "object",
                    "required": ["name", "version"],
                    "properties": {
                        "name": {"type": "string"},
                        "version": {"type": "string"},
                        "coordinate_system": {"type": "string"},
                        "units": {
                            "type": "object",
                            "properties": {
                                "distance": {"type": "string"},
                                "angle": {"type": "string"},
                            },
                        },
                    },
                    "additionalProperties": True,
                },
                "roads": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                        "type": "object",
                        "required": [
                            "id",
                            "length",
                            "reference_line",
                            "lane_sections",
                        ],
                        "properties": {
                            "id": {"type": "string"},
                            "length": {"type": "number", "exclusiveMinimum": 0},
                            "reference_line": {
                                "type": "array",
                                "minItems": 2,
                                "items": {
                                    "type": "object",
                                    "required": ["x", "y"],
                                    "properties": {
                                        "x": {"type": "number"},
                                        "y": {"type": "number"},
                                        "z": {"type": "number"},
                                    },
                                    "additionalProperties": False,
                                },
                            },
                            "successor": {
                                "type": "object",
                                "required": ["road_id", "contact_point"],
                                "properties": {
                                    "road_id": {"type": "string"},
                                    "contact_point": {
                                        "type": "string",
                                        "enum": ["start", "end"],
                                    },
                                },
                                "additionalProperties": False,
                            },
                            "predecessor": {
                                "type": "object",
                                "required": ["road_id", "contact_point"],
                                "properties": {
                                    "road_id": {"type": "string"},
                                    "contact_point": {
                                        "type": "string",
                                        "enum": ["start", "end"],
                                    },
                                },
                                "additionalProperties": False,
                            },
                            "speed_limit": {"type": "number", "minimum": 0},
                            "lane_sections": {
                                "type": "array",
                                "minItems": 1,
                                "items": {
                                    "type": "object",
                                    "required": ["s_start", "s_end", "lanes"],
                                    "properties": {
                                        "s_start": {
                                            "type": "number",
                                            "minimum": 0,
                                        },
                                        "s_end": {
                                            "type": "number",
                                            "exclusiveMinimum": 0,
                                        },
                                        "lanes": {
                                            "type": "array",
                                            "minItems": 1,
                                            "items": {
                                                "type": "object",
                                                "required": ["lane_id", "type"],
                                                "properties": {
                                                    "lane_id": {"type": "integer"},
                                                    "type": {
                                                        "type": "string",
                                                        "enum": [
                                                            "center",
                                                            "driving",
                                                            "shoulder",
                                                            "sidewalk",
                                                            "bikelane",
                                                            "parking",
                                                            "divider",
                                                            "reserved",
                                                        ],
                                                    },
                                                    "side": {
                                                        "type": "string",
                                                        "enum": [
                                                            "left",
                                                            "right",
                                                            "center",
                                                        ],
                                                    },
                                                    "level": {"type": "boolean"},
                                                    "speed_limit": {
                                                        "type": "number",
                                                        "minimum": 0,
                                                    },
                                                    "width": {
                                                        "type": "array",
                                                        "minItems": 1,
                                                        "items": {
                                                            "type": "object",
                                                            "required": [
                                                                "s_offset",
                                                                "a",
                                                            ],
                                                            "properties": {
                                                                "s_offset": {
                                                                    "type": "number",
                                                                    "minimum": 0,
                                                                },
                                                                "a": {"type": "number"},
                                                                "b": {"type": "number"},
                                                                "c": {"type": "number"},
                                                                "d": {"type": "number"},
                                                                "valid_length": {
                                                                    "type": "number",
                                                                    "minimum": 0,
                                                                },
                                                            },
                                                            "additionalProperties": False,
                                                        },
                                                    },
                                                    "attributes": {"type": "object"},
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
            "additionalProperties": True,
        }
    },
}
