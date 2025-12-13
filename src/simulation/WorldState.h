#pragma once

#include <Map.h>
#include <Vehicle.h>

#include <vector>

namespace LaneZero
{
class WorldState
{
public:
    Map world_map;
    std::unique_ptr<Vehicle> ego_vehicle;
    std::vector<std::unique_ptr<Vehicle>> vehicles;
    double current_time_s;
};
} // namespace LaneZero