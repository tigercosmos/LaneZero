#pragma once

#include <cstdint>

namespace LaneZero
{

class Goal
{
public:
    int32_t target_lane_id = 0;
    double desired_speed_mps = 30.0;

    Goal() = default;
    Goal(int32_t lane_id, double speed_mps)
        : target_lane_id(lane_id)
        , desired_speed_mps(speed_mps)
    {
    }
    ~Goal() = default;
};

} // namespace LaneZero

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
