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
                                "angle": {"type": "string"}
                            }
                        }
                    },
                    "additionalProperties": True
                },
                "roads": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                        "type": "object",
                        "required": ["id", "length", "reference_line", "lane_sections"],
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
                                        "z": {"type": "number"}
                                    },
                                    "additionalProperties": False
                                }
                            },
                            "successor": {
                                "type": "object",
                                "required": ["road_id", "contact_point"],
                                "properties": {
                                    "road_id": {"type": "string"},
                                    "contact_point": {
                                        "type": "string",
                                        "enum": ["start", "end"]
                                    }
                                },
                                "additionalProperties": False
                            },
                            "predecessor": {
                                "type": "object",
                                "required": ["road_id", "contact_point"],
                                "properties": {
                                    "road_id": {"type": "string"},
                                    "contact_point": {
                                        "type": "string",
                                        "enum": ["start", "end"]
                                    }
                                },
                                "additionalProperties": False
                            },
                            "speed_limit": {"type": "number", "minimum": 0},
                            "lane_sections": {
                                "type": "array",
                                "minItems": 1,
                                "items": {
                                    "type": "object",
                                    "required": ["s_start", "s_end", "lanes"],
                                    "properties": {
                                        "s_start": {"type": "number", "minimum": 0},
                                        "s_end": {"type": "number", "exclusiveMinimum": 0},
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
                                                            "center", "driving", "shoulder",
                                                            "sidewalk", "bikelane", "parking",
                                                            "divider", "reserved"
                                                        ]
                                                    },
                                                    "side": {
                                                        "type": "string",
                                                        "enum": ["left", "right", "center"]
                                                    },
                                                    "level": {"type": "boolean"},
                                                    "speed_limit": {"type": "number", "minimum": 0},
                                                    "width": {
                                                        "type": "array",
                                                        "minItems": 1,
                                                        "items": {
                                                            "type": "object",
                                                            "required": ["s_offset", "a"],
                                                            "properties": {
                                                                "s_offset": {"type": "number", "minimum": 0},
                                                                "a": {"type": "number"},
                                                                "b": {"type": "number"},
                                                                "c": {"type": "number"},
                                                                "d": {"type": "number"},
                                                                "valid_length": {"type": "number", "minimum": 0}
                                                            },
                                                            "additionalProperties": False
                                                        }
                                                    },
                                                    "attributes": {"type": "object"}
                                                },
                                                "additionalProperties": False
                                            }
                                        }
                                    },
                                    "additionalProperties": False
                                }
                            }
                        },
                        "additionalProperties": False
                    }
                }
            },
            "additionalProperties": True
        }
    }
}
