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
Test cases for Scenario class
"""

import json
import tempfile
import os
import LaneZero as lz


class TestScenario:
    """Test Scenario class"""

    def test_scenario_creation(self):
        """Test creating a scenario instance"""
        scenario = lz.Scenario()
        assert scenario is not None

    def test_scenario_load_from_file(self):
        """Test loading scenario from JSON file"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        assert scenario.meta().name == "Minimal Cut-In Scenario"
        assert scenario.meta().version == "1.0"

    def test_scenario_entities(self):
        """Test retrieving entities from scenario"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        entities = scenario.entities()
        assert len(entities) == 2

        entity_names = [entity.name for entity in entities]
        assert "ego" in entity_names
        assert "npc" in entity_names

        for entity in entities:
            assert entity.type == lz.EntityType.Vehicle
            assert entity.category == "car"
            assert entity.dimensions.length == 4.5
            assert entity.dimensions.width == 1.8
            assert entity.dimensions.height == 1.5

    def test_scenario_init_actions(self):
        """Test retrieving initialization actions from scenario"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        init_actions = scenario.init().actions
        assert len(init_actions) == 4

        action_types = [action.type for action in init_actions]
        assert action_types.count(lz.ActionType.Teleport) == 2
        assert action_types.count(lz.ActionType.Speed) == 2

    def test_scenario_teleport_action_parsing(self):
        """Test parsing teleport actions from scenario"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        entity_positions = {}
        for action in scenario.init().actions:
            if action.type == lz.ActionType.Teleport and action.position is not None:
                pos = action.position
                if isinstance(pos, lz.LanePosition):
                    entity_positions[action.entity] = {
                        "s": pos.s,
                        "lane_id": pos.lane_id,
                        "road_id": pos.road_id,
                    }

        assert "ego" in entity_positions
        assert entity_positions["ego"]["s"] == 10.0
        assert entity_positions["ego"]["lane_id"] == -1
        assert entity_positions["ego"]["road_id"] == "road_1"

        assert "npc" in entity_positions
        assert entity_positions["npc"]["s"] == 25.0
        assert entity_positions["npc"]["lane_id"] == -2
        assert entity_positions["npc"]["road_id"] == "road_1"

    def test_scenario_speed_action_parsing(self):
        """Test parsing speed actions from scenario"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        entity_speeds = {}
        for action in scenario.init().actions:
            if action.type == lz.ActionType.Speed and action.target_speed is not None:
                entity_speeds[action.entity] = action.target_speed

        assert "ego" in entity_speeds
        assert entity_speeds["ego"] == 20.0

        assert "npc" in entity_speeds
        assert entity_speeds["npc"] == 15.0

    def test_scenario_complete_entity_state_extraction(self):
        """Test extracting complete entity states from scenario"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        entity_states = {}
        for action in scenario.init().actions:
            entity_name = action.entity
            if entity_name not in entity_states:
                entity_states[entity_name] = {}

            if action.type == lz.ActionType.Teleport and action.position is not None:
                pos = action.position
                if isinstance(pos, lz.LanePosition):
                    entity_states[entity_name]["position"] = pos.s
                    entity_states[entity_name]["lane_id"] = pos.lane_id
            elif action.type == lz.ActionType.Speed and action.target_speed is not None:
                entity_states[entity_name]["velocity"] = action.target_speed

        assert "ego" in entity_states
        assert entity_states["ego"]["position"] == 10.0
        assert entity_states["ego"]["velocity"] == 20.0
        assert entity_states["ego"]["lane_id"] == -1

        assert "npc" in entity_states
        assert entity_states["npc"]["position"] == 25.0
        assert entity_states["npc"]["velocity"] == 15.0
        assert entity_states["npc"]["lane_id"] == -2

    def test_scenario_storyboard(self):
        """Test retrieving storyboard from scenario"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        storyboard = scenario.storyboard()
        assert storyboard is not None
        assert len(storyboard.acts) == 1

        act = storyboard.acts[0]
        assert act.name == "CutInAct"
        assert len(act.maneuvers) == 1

        maneuver = act.maneuvers[0]
        assert maneuver.name == "NpcCutIn"
        assert "npc" in maneuver.actors

    def test_scenario_to_json_string(self):
        """Test converting scenario to JSON string"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        json_string = scenario.to_json_string()
        assert json_string is not None
        assert len(json_string) > 0

        json_data = json.loads(json_string)
        assert "scenario" in json_data
        assert "meta" in json_data["scenario"]
        assert json_data["scenario"]["meta"]["name"] == "Minimal Cut-In Scenario"

    def test_scenario_from_json_string(self):
        """Test creating scenario from JSON string"""
        scenario_json = {
            "scenario": {
                "meta": {
                    "name": "Test Scenario",
                    "version": "1.0",
                    "description": "Test scenario for unit testing",
                },
                "entities": [
                    {
                        "name": "test_vehicle",
                        "type": "vehicle",
                        "category": "car",
                        "dimensions": {"length": 5.0, "width": 2.0, "height": 1.6},
                    }
                ],
                "init": {
                    "actions": [
                        {
                            "type": "teleport",
                            "entity": "test_vehicle",
                            "position": {
                                "type": "lane",
                                "road_id": "road_1",
                                "lane_id": -1,
                                "s": 100,
                            },
                        },
                        {"type": "speed", "entity": "test_vehicle", "target_speed": 30},
                    ]
                },
                "storyboard": {"stop_trigger": {"conditions": []}, "acts": []},
            }
        }

        scenario = lz.Scenario()
        scenario.from_json_string(json.dumps(scenario_json))

        assert scenario.meta().name == "Test Scenario"
        assert len(scenario.entities()) == 1
        assert scenario.entities()[0].name == "test_vehicle"
        assert len(scenario.init().actions) == 2

    def test_scenario_save_and_load(self):
        """Test saving and loading scenario to/from file"""
        scenario_json = {
            "scenario": {
                "meta": {"name": "Temp Test Scenario", "version": "2.0"},
                "entities": [],
                "init": {"actions": []},
                "storyboard": {"stop_trigger": {"conditions": []}, "acts": []},
            }
        }

        scenario1 = lz.Scenario()
        scenario1.from_json_string(json.dumps(scenario_json))

        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as temp_file:
            temp_path = temp_file.name

        try:
            scenario1.save_to_file(temp_path)

            scenario2 = lz.Scenario()
            scenario2.load_from_file(temp_path)

            assert scenario2.meta().name == "Temp Test Scenario"
            assert scenario2.meta().version == "2.0"
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)

    def test_scenario_entity_vehicle_mapping(self):
        """Test mapping entities to vehicle IDs"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        scenario.set_entity_vehicle_id("ego", 1)
        scenario.set_entity_vehicle_id("npc", 2)

        assert scenario.get_entity_vehicle_id("ego") == 1
        assert scenario.get_entity_vehicle_id("npc") == 2

        scenario.clear_entity_vehicles()
        assert scenario.get_entity_vehicle_id("ego") == -1
        assert scenario.get_entity_vehicle_id("npc") == -1

    def test_scenario_position_types(self):
        """Test different position types in actions"""
        scenario_json = {
            "scenario": {
                "meta": {"name": "Position Types Test", "version": "1.0"},
                "entities": [
                    {
                        "name": "vehicle1",
                        "type": "vehicle",
                        "category": "car",
                        "dimensions": {"length": 4.5, "width": 1.8, "height": 1.5},
                    }
                ],
                "init": {
                    "actions": [
                        {
                            "type": "teleport",
                            "entity": "vehicle1",
                            "position": {
                                "type": "lane",
                                "road_id": "road_1",
                                "lane_id": -1,
                                "s": 50,
                            },
                        }
                    ]
                },
                "storyboard": {"stop_trigger": {"conditions": []}, "acts": []},
            }
        }

        scenario = lz.Scenario()
        scenario.from_json_string(json.dumps(scenario_json))

        action = scenario.init().actions[0]
        assert action.type == lz.ActionType.Teleport
        assert action.position is not None

        pos = action.position
        assert isinstance(pos, lz.LanePosition)
        assert pos.lane_id == -1
        assert pos.s == 50.0
        assert pos.road_id == "road_1"

    def test_scenario_multiple_entities(self):
        """Test scenario with multiple entities"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        entities = scenario.entities()
        assert len(entities) == 2

        entity_dict = {entity.name: entity for entity in entities}

        assert "ego" in entity_dict
        assert "npc" in entity_dict

        for entity in entities:
            assert hasattr(entity, "dimensions")
            assert entity.dimensions.length > 0
            assert entity.dimensions.width > 0

    def test_scenario_action_entity_consistency(self):
        """Test that all action entities exist in entities list"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        entity_names = {entity.name for entity in scenario.entities()}

        for action in scenario.init().actions:
            assert action.entity in entity_names, (
                f"Action entity '{action.entity}' not found in entities list"
            )

    def test_scenario_init_actions_order(self):
        """Test that init actions are read in correct order"""
        scenario = lz.Scenario()
        scenario.load_from_file("data/scenario/npc_cut_in.json")

        actions = scenario.init().actions
        assert len(actions) == 4

        assert actions[0].type == lz.ActionType.Teleport
        assert actions[0].entity == "ego"

        assert actions[1].type == lz.ActionType.Speed
        assert actions[1].entity == "ego"

        assert actions[2].type == lz.ActionType.Teleport
        assert actions[2].entity == "npc"

        assert actions[3].type == lz.ActionType.Speed
        assert actions[3].entity == "npc"


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
