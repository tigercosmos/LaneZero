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
Test cases for Map and Lane classes
"""

import json
import os
import tempfile

import LaneZero as lz


class TestPoint:
    """Test Point class"""

    def test_point_creation_2d(self):
        """Test creating a 2D point"""
        point = lz.Point(10.0, 20.0)
        assert point.x == 10.0
        assert point.y == 20.0
        assert point.z is None

    def test_point_creation_3d(self):
        """Test creating a 3D point"""
        point = lz.Point(10.0, 20.0, 30.0)
        assert point.x == 10.0
        assert point.y == 20.0
        assert point.z == 30.0


class TestWidthCoefficients:
    """Test WidthCoefficients class"""

    def test_width_coefficients_creation(self):
        """Test creating width coefficients"""
        wc = lz.WidthCoefficients(0.0, 3.5)
        assert wc.s_offset == 0.0
        assert wc.a == 3.5
        assert wc.b == 0.0
        assert wc.c == 0.0
        assert wc.d == 0.0
        assert wc.valid_length is None

    def test_width_coefficients_full(self):
        """Test creating width coefficients with all parameters"""
        wc = lz.WidthCoefficients(10.0, 3.5, 0.1, 0.01, 0.001)
        assert wc.s_offset == 10.0
        assert wc.a == 3.5
        assert wc.b == 0.1
        assert wc.c == 0.01
        assert wc.d == 0.001


class TestLane:
    """Test Lane class"""

    def test_lane_creation(self):
        """Test creating a lane instance"""
        lane = lz.Lane(1, "driving")
        assert lane is not None
        assert lane.lane_id == 1
        assert lane.type == "driving"

    def test_lane_attributes(self):
        """Test lane attribute access and modification"""
        lane = lz.Lane(1, "driving")
        lane.side = "left"
        lane.speed_limit = 33.33

        assert lane.side == "left"
        assert lane.speed_limit == 33.33

    def test_lane_with_width(self):
        """Test creating a lane with width coefficients"""
        lane = lz.Lane(1, "driving")
        assert lane.width is not None
        assert isinstance(lane.width, list)


class TestLaneSection:
    """Test LaneSection class"""

    def test_lane_section_creation(self):
        """Test creating a lane section"""
        section = lz.LaneSection(0.0, 100.0)
        assert section.s_start == 0.0
        assert section.s_end == 100.0
        assert len(section.lanes) == 0

    def test_lane_section_with_lanes(self):
        """Test lane section with lanes attribute"""
        section = lz.LaneSection(0.0, 100.0)
        assert section.lanes is not None
        assert isinstance(section.lanes, list)


class TestRoad:
    """Test Road class"""

    def test_road_creation(self):
        """Test creating a road"""
        road = lz.Road("R1", 100.0)
        assert road.id == "R1"
        assert road.length == 100.0

    def test_road_with_reference_line(self):
        """Test road with reference line attribute"""
        road = lz.Road("R1", 100.0)
        assert road.reference_line is not None
        assert isinstance(road.reference_line, list)


class TestMapMeta:
    """Test MapMeta class"""

    def test_map_meta_creation(self):
        """Test creating map metadata"""
        meta = lz.MapMeta("test_map", "1.0")
        assert meta.name == "test_map"
        assert meta.version == "1.0"


class TestMap:
    """Test Map class"""

    def test_map_creation(self):
        """Test creating a map instance"""
        highway_map = lz.Map()
        assert highway_map is not None

    def test_map_empty_roads(self):
        """Test that new map has empty roads"""
        highway_map = lz.Map()
        assert len(highway_map.roads()) == 0

    def test_initialize_highway(self):
        """Test initializing a highway map"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)
        assert len(highway_map.roads()) == 1
        assert len(highway_map.roads()[0].lane_sections) == 1
        assert len(highway_map.roads()[0].lane_sections[0].lanes) == 3

    def test_highway_lane_properties(self):
        """Test properties of initialized highway lanes"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)

        lanes = highway_map.roads()[0].lane_sections[0].lanes
        for index, lane in enumerate(lanes):
            assert lane.lane_id == index
            assert lane.type == "driving"
            assert lane.side == "right"
            assert lane.speed_limit == 33.33

    def test_map_meta_access(self):
        """Test accessing map metadata"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=2, lane_length=1500.0)

        assert highway_map.meta().name == "highway"
        assert highway_map.meta().version == "1.0"

    def test_reinitialize_highway(self):
        """Test reinitializing highway clears previous roads"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)
        assert len(highway_map.roads()) == 1

        highway_map.initialize_highway(num_lanes=5, lane_length=3000.0)
        assert len(highway_map.roads()) == 1
        assert highway_map.roads()[0].length == 3000.0
        assert len(highway_map.roads()[0].lane_sections[0].lanes) == 5

    def test_json_serialization(self):
        """Test JSON serialization and deserialization"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=2, lane_length=1000.0)

        json_str = highway_map.to_json_string()
        assert json_str is not None
        assert len(json_str) > 0

        # Verify it's valid JSON
        json_data = json.loads(json_str)
        assert "map" in json_data
        assert "meta" in json_data["map"]
        assert "roads" in json_data["map"]

    def test_json_round_trip(self):
        """Test JSON serialization round trip"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=1500.0)

        json_str = highway_map.to_json_string()

        new_map = lz.Map()
        new_map.from_json_string(json_str)

        assert len(new_map.roads()) == 1
        assert new_map.roads()[0].length == 1500.0
        assert len(new_map.roads()[0].lane_sections[0].lanes) == 3

    def test_save_and_load_file(self):
        """Test saving and loading map from file"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=2, lane_length=2000.0)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            temp_file = f.name

        try:
            highway_map.save_to_file(temp_file)

            loaded_map = lz.Map()
            loaded_map.load_from_file(temp_file)

            assert len(loaded_map.roads()) == 1
            assert loaded_map.roads()[0].length == 2000.0
            assert len(loaded_map.roads()[0].lane_sections[0].lanes) == 2
        finally:
            if os.path.exists(temp_file):
                os.unlink(temp_file)

    def test_load_existing_map(self):
        """Test loading the existing two_road.json file"""
        map_file = "data/map/two_road.json"
        if not os.path.exists(map_file):
            return

        highway_map = lz.Map()
        highway_map.load_from_file(map_file)

        assert len(highway_map.roads()) == 2
        assert highway_map.meta().name == "mini"
        assert highway_map.roads()[0].id == "R1"
        assert highway_map.roads()[1].id == "R2"


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
