/*
 * startNode.cpp
 *
 *  Created on: Apr 15, 2020
 *      Author: antonio
 */

 #include "SimulationController.h"
 #include <rclcpp/rclcpp.hpp>
 #include <vector>
 #include <utility>
 #include <iostream>
 
 int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto sim = std::make_shared<SimulationController>();
    int numRays = sim->getNumSensors();
    int numActuators = sim->getNumActuators();

    std::vector<std::pair<double, double>> outputRanges;
    outputRanges.push_back(std::make_pair(-0.25, 1.0));
    outputRanges.push_back(std::make_pair(-0.5, 0.5));

    RCLCPP_INFO(sim->get_logger(), "Ready to evaluate Xolobot Drivers...");
    
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(sim);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
 