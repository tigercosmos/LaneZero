#include "Simulation.h"
#include <iostream>
#include <iomanip>

int main() {
    Simulation sim;
    
    sim.simulation_map.initialize_highway(3, 5000.0);
    
    Vehicle* t2_truck = new Vehicle(0, VehicleType::Truck, 0.0, 25.0, 1, 16.5, 2.5);
    sim.vehicles.push_back(t2_truck);
    
    sim.spawn_traffic(5);
    
    std::cout << "Starting LaneZero Simulation\n";
    std::cout << "=============================\n\n";
    
    double duration_s = 10.0;
    double delta_t_s = 0.1;
    int num_steps = static_cast<int>(duration_s / delta_t_s);
    
    for (int step = 0; step <= num_steps; ++step) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Time: " << sim.current_time_s << "s | ";
        std::cout << "Truck - Lane: " << t2_truck->current_lane_id 
                  << ", Position: " << t2_truck->position_s_m << "m"
                  << ", Velocity: " << t2_truck->velocity_mps << "m/s"
                  << ", Acceleration: " << t2_truck->acceleration_mps2 << "m/s²\n";
        
        if (step < num_steps) {
            for (auto* vehicle : sim.vehicles) {
                vehicle->calculate_control(sim.simulation_map, sim.vehicles);
            }
            
            for (auto* vehicle : sim.vehicles) {
                vehicle->update_kinematics(delta_t_s);
            }
            
            sim.current_time_s += delta_t_s;
            
            if (sim.check_collision()) {
                std::cout << "\n*** COLLISION DETECTED ***\n";
            }
        }
    }
    
    std::cout << "\nSimulation complete.\n";
    
    return 0;
}
