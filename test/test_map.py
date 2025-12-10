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

import LaneZero as lz


class TestLane:
    """Test Lane class"""

    def test_lane_creation(self):
        """Test creating a lane instance"""
        lane = lz.Lane(
            id=0,
            length=1000.0,
            speed_limit=120
        )
        assert lane is not None

    def test_lane_attributes(self):
        """Test lane attribute access"""
        lane = lz.Lane(
            id=0,
            length=1000.0,
            speed_limit=120
        )
        assert lane.id == 0
        assert lane.length_meters == 1000.0
        assert lane.speed_limit_kph == 120
        assert lane.adjacent_lane_id is None

    def test_lane_with_adjacent(self):
        """Test creating a lane with adjacent lane"""
        lane = lz.Lane(
            id=0,
            length=1000.0,
            speed_limit=120,
            adjacent=1
        )
        assert lane.adjacent_lane_id == 1

    def test_lane_attribute_modification(self):
        """Test modifying lane attributes"""
        lane = lz.Lane(
            id=0,
            length=1000.0,
            speed_limit=120
        )
        lane.speed_limit_kph = 100
        assert lane.speed_limit_kph == 100

        lane.adjacent_lane_id = 2
        assert lane.adjacent_lane_id == 2


class TestMap:
    """Test Map class"""

    def test_map_creation(self):
        """Test creating a map instance"""
        highway_map = lz.Map()
        assert highway_map is not None

    def test_map_empty_lanes(self):
        """Test that new map has empty lanes"""
        highway_map = lz.Map()
        assert len(highway_map.lanes) == 0

    def test_initialize_highway(self):
        """Test initializing a highway map"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)
        assert len(highway_map.lanes) == 3

    def test_highway_lane_properties(self):
        """Test properties of initialized highway lanes"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)

        for index, lane in enumerate(highway_map.lanes):
            assert lane.id == index
            assert lane.length_meters == 2000.0
            assert lane.speed_limit_kph == 120

    def test_highway_lane_adjacency(self):
        """Test adjacent lane relationships"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)

        # First lane should have adjacent lane 1
        assert highway_map.lanes[0].adjacent_lane_id == 1

        # Second lane should have adjacent lane 2
        assert highway_map.lanes[1].adjacent_lane_id == 2

        # Last lane should have no adjacent lane
        assert highway_map.lanes[2].adjacent_lane_id is None

    def test_map_lanes_access(self):
        """Test accessing lanes from map"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=2, lane_length=1500.0)

        lane0 = highway_map.lanes[0]
        assert lane0.id == 0
        assert lane0.length_meters == 1500.0

        lane1 = highway_map.lanes[1]
        assert lane1.id == 1
        assert lane1.adjacent_lane_id is None

    def test_reinitialize_highway(self):
        """Test reinitializing highway clears previous lanes"""
        highway_map = lz.Map()
        highway_map.initialize_highway(num_lanes=3, lane_length=2000.0)
        assert len(highway_map.lanes) == 3

        highway_map.initialize_highway(num_lanes=5, lane_length=3000.0)
        assert len(highway_map.lanes) == 5

        for lane in highway_map.lanes:
            assert lane.length_meters == 3000.0


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
