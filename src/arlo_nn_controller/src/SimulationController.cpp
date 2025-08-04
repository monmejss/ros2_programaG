/*
 * SimulationController.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#include "SimulationController.h"


 SimulationController::SimulationController(double maxSTime, int tRate) 
 : Node("simulation_controller"),
    linear_(0),
    angular_(0),
    l_scale_(1.0),
    a_scale_(1.0),
    maxSimTime(maxSTime),
    ticsRate(tRate),
    actuatorValues(NUM_ACTUATORS, 0.0)
 {
    sensorValues.resize(NUM_RAYS * NUM_SONARS);
 
    prev_x = 0;
    prev_y = 0;
    stuck = false;
    stuckCounter = 0;
    this->declare_parameter("scale_angular", a_scale_);
    this->declare_parameter("scale_linear", l_scale_);
    this->get_parameter("scale_angular", a_scale_);
    this->get_parameter("scale_linear", l_scale_);

 
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
   // odom con QoS
   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SimulationController::checkModelPosition, this, std::placeholders::_1));

   // clock con QoS
   rclcpp::QoS qos_clock(rclcpp::KeepLast(10));
   qos_clock.best_effort();
   clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos_clock,
      std::bind(&SimulationController::checkSimulationTime, this, std::placeholders::_1));

    sonar_l_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "arlo/laser/scan_left", 10,
      std::bind(&SimulationController::checkSonarLeftValues, this, std::placeholders::_1));

    sonar_c_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "arlo/laser/scan_center", 10,
      std::bind(&SimulationController::checkSonarCenterValues, this, std::placeholders::_1));

    sonar_r_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "arlo/laser/scan_right", 10,
      std::bind(&SimulationController::checkSonarRightValues, this, std::placeholders::_1));


    service_ = this->create_service<arlo_nn_controller::srv::EvaluateDriver>(
      "evaluate_driver",
      std::bind(&SimulationController::evaluateDriver, this,
                  std::placeholders::_1, std::placeholders::_2));

   // Cliente de servicio
    actuatorClient_ = this->create_client<arlo_interfaces::srv::EvaluateTree>("evaluate_tree");
    reset_sim_client_ = this->create_client<std_srvs::srv::Empty>("/reset_simulation");
    RCLCPP_INFO(this->get_logger(), "SimulationController inicializado");
 }
 
 void SimulationController::actuatorCallback(std_msgs::msg::Float32MultiArray::SharedPtr array )
 {
    int i = 0;
    RCLCPP_INFO(this->get_logger(), "I heard:");
 
    for(auto val : array->data){
       actuatorValues[i] = val;
       std::cout << actuatorValues[i] << std::endl;
       i++;
    } 
 }

 bool SimulationController::evaluateDriver(
   const std::shared_ptr<arlo_nn_controller::srv::EvaluateDriver::Request> req,
   std::shared_ptr<arlo_nn_controller::srv::EvaluateDriver::Response> res)
{
   RCLCPP_INFO(this->get_logger(), "Llamada recibida en evaluateDriver: maxtime=%ld, tree_index=%ld", 
   req->maxtime, req->tree_index);

   startSimulation(req->maxtime, req->tree_index);

   res->time = arloState.finishTime;
   res->dist2go = arloState.distanceToGo;
   res->damage = arloState.robotDamage;
   res->energy = arloState.robotEnergy;

   return true;
}

 
 SimulationController::~SimulationController() {
 }
 
 SimulationState SimulationController::startSimulation(int maxtime, int tree_index)
{
    RCLCPP_INFO(this->get_logger(), "Starting the simulation of a new driver...");
    RCLCPP_INFO(this->get_logger(), "---------------------------");

    // Esperar y llamar a reset_simulation
    while (!reset_sim_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "ROS apagándose mientras esperaba /reset_simulation");
            return arloState;
        }
        RCLCPP_INFO(this->get_logger(), "Esperando servicio /reset_simulation...");
    }
    auto reset_req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto reset_future = reset_sim_client_->async_send_request(reset_req);
    reset_future.wait_for(std::chrono::seconds(5)); // Espera hasta 5 segundos

    maxSimTime = maxtime;
    rclcpp::Rate loop_rate(50);
    linear_ = angular_ = 0;
    arloState.resetState();
    stuckCounter = 0;

    while (rclcpp::ok() && !arloState.hasTimeRunOut && !arloState.finishLineCrossed) {
        // Esperar servicio evaluate_tree
        while (!actuatorClient_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "ROS apagándose mientras esperaba evaluate_tree");
                return arloState;
            }
            RCLCPP_WARN(this->get_logger(), "Esperando servicio evaluate_tree...");
        }

        // Llamada a evaluate_tree
        auto eval_req = std::make_shared<arlo_interfaces::srv::EvaluateTree::Request>();
        for (int i = 0; i < NUM_RAYS; i++) {
            eval_req->sensor_values[i] = sensorValues[i];
        }
        eval_req->tree_index = tree_index;

        auto eval_future = actuatorClient_->async_send_request(eval_req);
        if (eval_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = eval_future.get();
            actuatorValues[0] = response->actuator_values[0];
            actuatorValues[1] = response->actuator_values[1];
        } else {
            actuatorValues[0] = actuatorValues[1] = 0.0;
        }

        // Publicar velocidades
        linear_  = actuatorValues[0];
        angular_ = actuatorValues[1];
        geometry_msgs::msg::Twist twist;
        twist.angular.z = a_scale_ * angular_;
        twist.linear.x = l_scale_ * linear_;
        vel_pub_->publish(twist);

        loop_rate.sleep();
    }

    // Resumen
    if (arloState.hasTimeRunOut) {
        arloState.finishTime = 2 * maxSimTime;
        if (arloState.stuck) {
            RCLCPP_WARN(this->get_logger(), " ---->>> ATASCADO  <<<-----");
        }
    } else {
        arloState.finishTime = arloState.currentTime;
        arloState.robotEnergy = 100;
    }

    // Reset final
    auto final_reset = reset_sim_client_->async_send_request(reset_req);
    final_reset.wait_for(std::chrono::seconds(5));

    return arloState;
}
 
 
 void SimulationController::checkSonarLeftValues(sensor_msgs::msg::LaserScan::SharedPtr msg)
 { 
   for (int i = 0; i < msg->ranges.size(); ++i) {
      sensorValues[i + 0*NUM_RAYS] = msg->ranges[i];  // 0 para el sensor izq.
   }
 }
 
 void SimulationController::checkSonarCenterValues(sensor_msgs::msg::LaserScan::SharedPtr msg)
 {
   for (int i = 0; i < msg->ranges.size(); ++i) {
   sensorValues[i + 1*NUM_RAYS] = msg->ranges[i]; // 1 para el sensor central.
   }
 }
 
 void SimulationController::checkSonarRightValues(sensor_msgs::msg::LaserScan::SharedPtr msg)
 {
   for (int i = 0; i < msg->ranges.size(); ++i) {
      sensorValues[i + 2*NUM_RAYS] = msg->ranges[i]; // 2 para el sensor der.
   }
 }
 
 double SimulationController::dist2Go(double x, double y) {
    double distToGo;
    if ( y < 0.8 && x < 4.7) { // Va en la primera recta
          distToGo = 18 - x;
    }
    else if (y < 6.61) {  // Va en la segunda recta
       distToGo = 18 - (5.25 + y);
    }
    else {   // Va en la recta final
       if (x > 0.0)
          distToGo = x;
       else
          distToGo = 0.0;
    }
    return distToGo;
 }
 
 double SimulationController::distance(double x1, double y1, double x2, double y2) {
    double sum = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
    return sqrt(sum);
 }
 
 void SimulationController::checkModelPosition(const nav_msgs::msg::Odometry::SharedPtr msg)
 {
   double distanceBefore = arloState.distanceTravelled;
   arloState.distanceTravelled += distance(prev_x, prev_y,
      msg->pose.pose.position.x,
      msg->pose.pose.position.y);

   if (abs(distanceBefore-arloState.distanceTravelled) < 0.01) {
      stuckCounter++;
      if (stuckCounter > 80) {
         arloState.stuck = true;
         arloState.hasTimeRunOut = true;
      }
   }
   else
      stuckCounter = 0;

   prev_x = msg->pose.pose.position.x;
   prev_y = msg->pose.pose.position.y;

   arloState.currentPosition = msg->pose.pose.position.x;
   arloState.position[0] = msg->pose.pose.position.x;
   arloState.position[1] = msg->pose.pose.position.y;

   arloState.distanceToGo = dist2Go(msg->pose.pose.position.x, msg->pose.pose.position.y);
   if (arloState.distanceToGo <= 0.0)  
      arloState.finishLineCrossed = true;
}
 
void SimulationController::checkSimulationTime(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
   arloState.currentTime = rclcpp::Time(msg->clock).seconds();
   if (arloState.currentTime >= maxSimTime) {
      arloState.hasTimeRunOut = true;
   }
}
 
 int SimulationController::getNumSensors() {
    return NUM_RAYS * NUM_SONARS;
 }
 
 int SimulationController::getNumActuators() {
    return NUM_ACTUATORS;
 }