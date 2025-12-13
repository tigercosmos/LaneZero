#pragma once

#include "Goal.h"
#include "WorldState.h"

namespace LaneZero
{

class Scenario
{
public:
    Scenario() = default;
    virtual ~Scenario() = default;

    virtual Goal update_goal(WorldState const & world_state);
};

class LaneKeepScenario : public Scenario
{
public:
    LaneKeepScenario() = default;
    ~LaneKeepScenario() override = default;

    Goal update_goal(WorldState const & world_state) override;
};

} // namespace LaneZero

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
