#include <gtest/gtest.h>
#include "Map.h"
#include "Vehicle.h"
#include "Simulation.h"

TEST(MapTest, Initialization) {
    Map map;
    map.initialize_highway(3, 5000.0);
    
    EXPECT_EQ(map.lanes.size(), 3);
    EXPECT_DOUBLE_EQ(map.lanes[0].length_meters, 5000.0);
    EXPECT_EQ(map.lanes[0].speed_limit_kph, 120);
}

TEST(VehicleTest, KinematicsUpdate) {
    Vehicle car(1, VehicleType::Car, 0.0, 20.0, 0, 4.5, 2.0);
    car.acceleration_mps2 = 2.0;
    
    car.update_kinematics(1.0);
    
    EXPECT_DOUBLE_EQ(car.velocity_mps, 22.0);
    EXPECT_DOUBLE_EQ(car.position_s_m, 21.0);
}

TEST(SimulationTest, RunsWithoutCrash) {
    Simulation sim;
    sim.simulation_map.initialize_highway(3, 5000.0);
    
    Vehicle* truck = new Vehicle(0, VehicleType::Truck, 0.0, 25.0, 1, 16.5, 2.5);
    sim.vehicles.push_back(truck);
    
    EXPECT_NO_THROW(sim.run(1.0, 0.1));
}
