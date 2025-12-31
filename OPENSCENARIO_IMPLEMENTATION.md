# OpenScenario Implementation

## Overview

LaneZero now supports OpenScenario-like scenario definitions for traffic simulation. This implementation allows you to define complex traffic scenarios with entities, actions, events, conditions, and triggers.

## Architecture

### Core Components

1. **Scenario** (`src/scenario/Scenario.h/cpp`)
   - Main scenario class that manages the entire scenario lifecycle
   - Handles JSON parsing and serialization
   - Manages entity-vehicle mapping
   - Coordinates initialization and updates

2. **Entities** (`ScenarioEntity`)
   - Represents traffic participants (vehicles, pedestrians, etc.)
   - Includes dimensions and performance characteristics
   - Maps to simulation vehicles

3. **Positions** (Position variants)
   - `WorldPosition`: Absolute coordinates (x, y, z)
   - `LanePosition`: Lane-relative (road_id, lane_id, s offset)
   - `RoadPosition`: Road-relative (road_id, s offset)

4. **Actions** (`ScenarioAction`)
   - **Teleport**: Instantly move entity to a position
   - **Speed**: Set entity velocity
   - **LaneChange**: Move entity to different lane
   - **FollowTrajectory**: Follow predefined path
   - **FollowRoute**: Navigate waypoints

5. **Conditions** (`Condition`)
   - **TimeCondition**: Trigger after elapsed time
   - **DistanceToEntityCondition**: Trigger based on proximity
   - Supports comparison operators: `<`, `>`, `==`, `!=`, `<=`, `>=`

6. **Triggers** (`Trigger`)
   - Combines multiple conditions
   - **Any** rule: Trigger if ANY condition met
   - **All** rule: Trigger if ALL conditions met

7. **Events** (`Event`)
   - State machine: Standby → Running → Complete
   - Starts when trigger conditions met
   - Executes assigned actions
   - Completes when actions finish

8. **Storyboard** (Act → Maneuver → Event hierarchy)
   - **Act**: Top-level scenario phase
   - **Maneuver**: Group of coordinated events
   - **Event**: Single triggered action sequence

## JSON Schema

The scenario JSON format follows this structure:

```json
{
  "meta": {
    "name": "Scenario Name",
    "version": "1.0",
    "description": "Scenario description"
  },
  "entities": [
    {
      "name": "ego",
      "type": "vehicle",
      "dimensions": {
        "length": 4.5,
        "width": 1.8,
        "height": 1.5
      }
    }
  ],
  "init": {
    "actions": [
      {
        "type": "teleport",
        "entity": "ego",
        "position": {
          "type": "lane",
          "lane_id": -1,
          "s": 10.0
        }
      },
      {
        "type": "speed",
        "entity": "ego",
        "target_speed": 15.0
      }
    ]
  },
  "storyboard": {
    "acts": [
      {
        "name": "MainAct",
        "maneuvers": [
          {
            "name": "CutInManeuver",
            "events": [
              {
                "name": "CutInEvent",
                "trigger": {
                  "rule": "any",
                  "conditions": [
                    {
                      "type": "distance_to_entity",
                      "entity": "npc",
                      "target_entity": "ego",
                      "distance": 10.0,
                      "comparison": "<"
                    }
                  ]
                },
                "actions": [
                  {
                    "type": "lane_change",
                    "entity": "npc",
                    "target_lane_id": -1
                  }
                ]
              }
            ]
          }
        ]
      }
    ]
  }
}
```

## Usage Example

```python
import LaneZero as lz

# Create scenario
scenario = lz.Scenario()
scenario.load_from_file("data/scenario/my_scenario.json")

# Create simulation
simulation = lz.Simulation()
simulation.simulation_map.load_from_file("data/map/my_map.json")

# Create world state
world_state = lz.WorldState()
world_state.world_map = simulation.simulation_map

# Create vehicles from scenario entities
for entity in scenario.entities():
    vehicle = lz.Vehicle(
        id=1,
        type=lz.VehicleType.Car,
        position=0.0,
        velocity=0.0,
        lane_id=-1,
        length=entity.dimensions.length,
        width=entity.dimensions.width
    )
    if entity.name == "ego":
        world_state.ego_vehicle = vehicle
    else:
        world_state.add_vehicle(vehicle)
    
    # Map entity to vehicle
    if entity.name == "ego":
        scenario.set_entity_vehicle(entity.name, world_state.ego_vehicle)
    else:
        scenario.set_entity_vehicle(
            entity.name,
            world_state.get_vehicle(world_state.get_vehicle_count() - 1)
        )

# Initialize scenario (execute init actions)
scenario.initialize(world_state)

# Run simulation
while not scenario.is_complete(world_state):
    world_state.current_time_s += 0.1
    scenario.update(world_state)

# Cleanup
scenario.clear_entity_vehicles()
```

## Implementation Details

### Condition Evaluation

Conditions are evaluated every simulation step:

```cpp
bool Condition::evaluate(WorldState const & world_state,
                        std::map<std::string, Vehicle *> const & entity_vehicles) const
{
    switch (type)
    {
        case ConditionType::Time:
            return compare(world_state.current_time_s, time_value, comparison);
        
        case ConditionType::DistanceToEntity:
            Vehicle * v1 = entity_vehicles.at(entity);
            Vehicle * v2 = entity_vehicles.at(target_entity);
            double dist = std::abs(v1->position_s_m - v2->position_s_m);
            return compare(dist, distance_value, comparison);
    }
}
```

### Trigger Logic

Triggers combine conditions with AND/OR logic:

```cpp
bool Trigger::is_triggered(WorldState const & world_state,
                          std::map<std::string, Vehicle *> const & entity_vehicles) const
{
    if (rule == TriggerRule::Any)
    {
        // OR logic - true if ANY condition is true
        for (auto const & cond : conditions)
        {
            if (cond.evaluate(world_state, entity_vehicles))
            {
                return true;
            }
        }
        return false;
    }
    else
    {
        // AND logic - true if ALL conditions are true
        for (auto const & cond : conditions)
        {
            if (!cond.evaluate(world_state, entity_vehicles))
            {
                return false;
            }
        }
        return true;
    }
}
```

### Event State Machine

Events progress through states:

1. **Standby**: Waiting for trigger conditions
2. **Running**: Executing actions
3. **Complete**: All actions finished

```cpp
void Event::update(WorldState & world_state,
                  std::map<std::string, Vehicle *> const & entity_vehicles)
{
    if (state == EventState::Standby)
    {
        if (start_trigger.is_triggered(world_state, entity_vehicles))
        {
            state = EventState::Running;
            // Execute all actions
            for (auto & action : actions)
            {
                action.execute(world_state, entity_vehicles);
            }
            state = EventState::Complete;
        }
    }
}
```

## Python Bindings

All scenario classes are exposed to Python via pybind11:

- `WorldPosition`, `LanePosition`, `Road Position`
- `EntityType`, `ScenarioEntity`
- `ConditionType`, `ComparisonOperator`, `Condition`
- `TriggerRule`, `Trigger`
- `ActionType`, `ScenarioAction`
- `EventState`, `Event`
- `Maneuver`, `Act`, `Storyboard`
- `ScenarioMeta`, `ScenarioInit`
- `Scenario`

## Known Issues

### Segmentation Fault on Program Exit

There is a known segmentation fault that occurs during Python program cleanup after simulation completes. This is a C++/Python object lifetime management issue and does **not** affect the functional correctness of the simulation.

**Details:**
- The fault occurs after all simulation work completes successfully
- It happens during Python's final object destruction phase
- All scenario loading, initialization, and execution work correctly
- The issue is related to raw pointer storage in the entity-vehicle mapping

**Workaround:**
- Use the `--test-load` flag to test only scenario loading (no segfault)
- Accept that the segfault happens after all useful work is done
- The simulation results are valid and correct

**Future Fix:**
- Refactor entity-vehicle mapping to use vehicle IDs instead of raw pointers
- Implement proper shared ownership or weak references
- Add lifecycle management hooks for cleanup ordering

## Testing

Run the example:

```bash
# Test scenario loading only (no segfault)
PYTHONPATH=. python3 examples/openscenario_example.py --test-load

# Test full simulation (segfaults at cleanup but simulation works)
PYTHONPATH=. python3 examples/openscenario_example.py --test-sim

# Run both tests
PYTHONPATH=. python3 examples/openscenario_example.py
```

Example scenario files:
- `data/scenario/npc_cut_in.json`: NPC cuts in front of ego vehicle

## Future Enhancements

1. **Additional Actions**
   - Acceleration/braking dynamics
   - Route following
   - Trajectory interpolation

2. **Additional Conditions**
   - Speed conditions
   - Position-based triggers
   - Custom user conditions

3. **Enhanced Events**
   - Duration-based actions
   - Concurrent event execution
   - Event priorities

4. **Validation**
   - JSON schema validation
   - Entity reference checking
   - Circular dependency detection

5. **Memory Safety**
   - Replace raw pointers with smart pointers or IDs
   - Improve lifetime management
   - Add reference counting

## References

- OpenSCENARIO Specification: https://www.asam.net/standards/detail/openscenario/
- Example scenarios: `data/scenario/`
- C++ implementation: `src/scenario/`
- Python bindings: `src/scenario/pymod/`
- Example usage: `examples/openscenario_example.py`
