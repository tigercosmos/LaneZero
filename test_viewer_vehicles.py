#!/usr/bin/env python3
# Test if viewer shows vehicles

import LaneZero as lz

sim = lz.Simulation()
sim.simulation_map.initialize_highway(num_lanes=3, lane_length=2000.0)

# Add ego vehicle
ego = lz.Vehicle(0, lz.VehicleType.Car, 500.0, 25.0, 1, 4.5, 2.0)
sim.set_ego_vehicle(ego)

# Add more vehicles at various positions
for i in range(1, 6):
    v = lz.Vehicle(i, lz.VehicleType.Car, i * 200.0, 25.0, i % 3, 4.5, 2.0)
    sim.add_vehicle_copy(v)

print(f"Created {len(sim.get_vehicles())} vehicles")
for v in sim.get_vehicles():
    print(f"  Vehicle {v.id}: pos={v.position_s_m:.1f}, lane={v.current_lane_id}")

if lz.viewer.enable:
    viewer = lz.viewer.SimulationViewer(sim, title="Vehicle Test")
    viewer.run(duration_s=5.0, delta_t_s=0.1)
else:
    print("Viewer not available")
