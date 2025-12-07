#include "Map.h"

Lane::Lane(int id, double length, int speed_limit, std::optional<int> adjacent)
    : id(id), length_meters(length), speed_limit_kph(speed_limit), adjacent_lane_id(adjacent) {}

void Map::initialize_highway(int num_lanes, double lane_length) {
    lanes.clear();
    for (int i = 0; i < num_lanes; ++i) {
        std::optional<int> adjacent = (i < num_lanes - 1) ? std::optional<int>(i + 1) : std::nullopt;
        lanes.emplace_back(i, lane_length, 120, adjacent);
    }
}
