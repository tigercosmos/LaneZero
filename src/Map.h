#pragma once

#include <vector>
#include <optional>

class Lane {
public:
    int id;
    double length_meters;
    int speed_limit_kph;
    std::optional<int> adjacent_lane_id;

    Lane(int id, double length, int speed_limit, std::optional<int> adjacent = std::nullopt);
};

class Map {
public:
    std::vector<Lane> lanes;

    void initialize_highway(int num_lanes, double lane_length);
};
