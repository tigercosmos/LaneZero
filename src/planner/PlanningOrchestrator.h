#pragma once

#include <memory>

namespace LaneZero
{

class BehaviorPlanner;
class MotionPlanner;
class ControlPlanner;
class WorldState;
class Goal;

struct PlanResult
{
    double acceleration_mps2 = 0.0;
    double steering_angle_rad = 0.0;
};

class PlanningOrchestrator
{
private:
    std::unique_ptr<BehaviorPlanner> m_behavior_planner;
    std::unique_ptr<MotionPlanner> m_motion_planner;
    std::unique_ptr<ControlPlanner> m_control_planner;

public:
    PlanningOrchestrator();
    ~PlanningOrchestrator();

    PlanResult plan(WorldState const & world_state, Goal const & goal, double delta_t_s);
};

} // namespace LaneZero

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
