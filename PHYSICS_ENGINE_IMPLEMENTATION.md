# Physics Engine Implementation Summary

## Overview
A 2D rigid-body vehicle physics module has been successfully integrated into the LaneZero simulation framework. The implementation uses a bicycle model with support for multiple physics engine types through an extensible enum-based system.

## Implementation Details

### Directory Structure
```
src/physics_engine/
├── PhysicsEngine.h           # Base class and common structures
├── BicycleModelEngine.h      # Bicycle model header
├── BicycleModelEngine.cpp    # Bicycle model implementation
└── pymod/
    └── wrap_PhysicsEngine.cpp # Python bindings
```

### Core Components

#### 1. Physics Engine Type Enum
```cpp
enum class PhysicsEngineType
{
    BicycleModel
};
```
Future physics models can be added as new enum values.

#### 2. Rigid Body State
```cpp
struct RigidBodyState
{
    double x_m;              // Global X position (meters)
    double y_m;              // Global Y position (meters)
    double yaw_rad;          // Yaw angle (radians)
    double velocity_x_mps;   // Body-frame X velocity (m/s)
    double velocity_y_mps;   // Body-frame Y velocity (m/s)
    double yaw_rate_radps;   // Yaw rate (rad/s)
};
```

#### 3. Vehicle Control
```cpp
struct VehicleControl
{
    double steering_angle_rad;    // Steering angle (radians)
    double longitudinal_force_n;  // Longitudinal force (Newtons)
};
```

#### 4. Vehicle Physics Parameters
```cpp
struct VehiclePhysicsParameters
{
    double mass_kg;                                    // Vehicle mass
    double yaw_inertia_kgm2;                          // Yaw inertia
    double wheelbase_m;                                // Wheelbase
    double front_axle_distance_m;                     // Front axle to CG
    double rear_axle_distance_m;                      // Rear axle to CG
    double tire_cornering_stiffness_front_n_per_rad;  // Front tire stiffness
    double tire_cornering_stiffness_rear_n_per_rad;   // Rear tire stiffness
    double max_steering_angle_rad;                    // Max steering angle
    double max_steering_rate_radps;                   // Max steering rate
    double max_lateral_acceleration_mps2;             // Max lateral acceleration
};
```

### Bicycle Model Implementation

The bicycle model implements the following physics:

1. **Slip Angle Calculation**
   - Front slip angle: α_f = atan2(v_y + l_f * ψ̇, v_x) - δ
   - Rear slip angle: α_r = atan2(v_y - l_r * ψ̇, v_x)

2. **Tire Forces**
   - Front lateral force: F_yf = -C_f * α_f
   - Rear lateral force: F_yr = -C_r * α_r

3. **Newton-Euler Equations**
   - Longitudinal acceleration: a_x = (F_x - F_yf * sin(δ)) / m + v_y * ψ̇
   - Lateral acceleration: a_y = (F_yf * cos(δ) + F_yr) / m - v_x * ψ̇
   - Yaw acceleration: α_ψ = (l_f * F_yf * cos(δ) - l_r * F_yr) / I_z

4. **Integration**
   - Forward Euler integration with fixed timestep
   - Yaw angle normalized to [-π, π]

5. **Limits**
   - Steering angle clamping
   - Lateral acceleration limiting

## Integration with Vehicle Class

The `Vehicle` class has been enhanced with:

- Physics engine instance (disabled by default for backward compatibility)
- Physics state, control, and parameter accessors
- Automatic physics update during `update_kinematics()`
- Copy constructor and assignment operator to handle unique_ptr members

### API Usage

```python
import LaneZero as lz

# Create vehicle
vehicle = lz.Vehicle(id=0, type=lz.VehicleType.Car, position=0.0, 
                     velocity=20.0, lane_id=0, length=4.5, width=2.0)

# Enable physics engine
vehicle.set_physics_engine_type(lz.PhysicsEngineType.BicycleModel)

# Set parameters
params = lz.VehiclePhysicsParameters()
params.mass_kg = 1500.0
params.wheelbase_m = 2.7
# ... set other parameters
vehicle.set_physics_parameters(params)

# Set initial state
state = lz.RigidBodyState()
state.velocity_x_mps = 20.0
vehicle.set_physics_state(state)

# Set control inputs
control = lz.VehicleControl()
control.steering_angle_rad = 0.1
control.longitudinal_force_n = 0.0
vehicle.set_control(control)

# Simulate
vehicle.update_kinematics(delta_t=0.1)

# Get updated state
final_state = vehicle.get_physics_state()
```
