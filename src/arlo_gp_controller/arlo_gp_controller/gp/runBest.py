#!/usr/bin/env python3
from __future__ import annotations
import re
from arlo_gp_controller.gp.gpTree import Tree
import rclpy
from rclpy.node import Node
from arlo_gp_controller.gp.gp import GeneticProgram

from arlo_interfaces.srv import EvaluateTree
from arlo_nn_controller.srv import EvaluateDriver

import pickle

if __name__ == "__main__":
    rclpy.init()
    node = Node('gp')

    print("Mejor Individuo \n --------------------------------------\n")
    gp = None
    
    def handleEvaluateTree(request, response):
        individual = None
        evalBest = True if request.tree_index == -1 else False
        if evalBest : 
            individual =  gp.bestParent
        else :
            individual = gp.population[request.tree_index]
        
        actuatorValues = individual.evaluateTree(request.sensor_values)
        response.actuator_values = actuatorValues
        return response

    server = node.create_service(EvaluateTree, 'evaluate_tree', handleEvaluateTree)
    client = node.create_client(EvaluateDriver, 'evaluate_driver')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Esperando al servicio evaluate_driver...')
    
    
    with open ('gp.dat', 'rb') as gpFile:
        gp = pickle.load(gpFile)
        gp.bestParent = gp.bestEver.copyTree()
        print('GP loaded from gp.dat\n')
        gp.population[0].showSymTable()

    print("Best individual")
    print("Aptitud:", gp.bestAptitud)
    gp.bestParent.showTree()

    req = EvaluateDriver.Request()
    req.maxtime = 60
    req.tree_index = -1
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()

