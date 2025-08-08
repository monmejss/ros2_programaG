from __future__ import annotations
import re
from arlo_gp_controller.gp.gpTree import Tree

import rclpy
from rclpy.node import Node

from arlo_gp_controller.gp.gp import GeneticProgram
from arlo_interfaces.srv import EvaluateTree
from arlo_nn_controller.srv import EvaluateDriver

from rclpy.executors import MultiThreadedExecutor

import threading

import pickle

def main():
    rclpy.init()
    node = Node('gp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    print("Programa Genetico \n --------------------------------------\n")
    popSize = int(input("population size: "))
    generations = int(input("number of generations: "))
    treeDepth   = int(input("max tree depth: "))
    mutationProbability = float(input("mutation probability: "))
    gp = GeneticProgram(popSize, generations, treeDepth, 'full', mutationProbability)
    
    def handleEvaluateTree(request, response):
        try:
            node.get_logger().info(f"Solicitud recibida para árbol {request.tree_index}")
            individual = None
            evalBest = True if request.tree_index == -1 else False
            if evalBest: 
                individual = gp.bestParent
            else:
                individual = gp.population[request.tree_index]

            individual.setRosNode(node)
            actuatorValues = individual.evaluateTree(request.sensor_values)
            #print(f" Valores del actuador: {actuatorValues}")
            response.actuator_values = list(actuatorValues)
            return response
        
        except Exception as e:
            node.get_logger().error(f"Excepción en handleEvaluateTree: {e}")
            response.actuator_values = [0.0, 0.0]
            return response

    # servidor del servicio evaluate_tree
    server = node.create_service(EvaluateTree, 'evaluate_tree', handleEvaluateTree)
    client = node.create_client(EvaluateDriver, 'evaluate_driver')
    
    # para que el nodo siga 
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Esperando al servicio evaluate_driver...')
    
    
    f = open("report.txt", "w")
    header = "{\n{popSize: "+str(popSize)+",\ngenerations: "+str(generations)+",\ntreeDepth: "+str(treeDepth)+",\n"+"treeType: full,\nmutationProbability: "+str(mutationProbability)+"},\n[\n"
    f.write(header)
    gp.setInitialPopulation()
    print("Genetic Program Start")
    
    for generation in range(gp.maxGen):
        print("generation:", generation)
        for index in range(gp.popSize):
            print(f"[GP] Solicitando evaluación para individuo {index}")
            print("\tIndividual: ",index)
            #maxtime, treeIndex
            req = EvaluateDriver.Request()
            req.maxtime = 60
            req.tree_index = index
            print(f"[GP] Enviando request → maxtime={req.maxtime}, tree_index={req.tree_index}")
            # prueba para el reset_simulation
            future = client.call_async(req)
            while not future.done():
                #node.get_logger().info("Esperando respuesta de evaluate_driver...")
                executor.spin_once(timeout_sec=0.1)

            response = future.result()
            print(f"[GP] Respuesta recibida: dist2go={response.dist2go}")
            gp.setAptitude(index, response.dist2go) 
            
        gp.setBestAptitud()
        gp.setBestParent()
        gp.setParents('torneo')

        if generation!= 0:  
            gp.sortParents()
        
        aptitudPopulationAverage = sum(gp.aptitudes)/gp.popSize
        populationReport = ",{\nbestAptitud: "+str(gp.bestAptitud)+",\n"+"averageAptitud: "+str(aptitudPopulationAverage)+"\n}\n"
        f.write(populationReport)
        gp.cross()
        gp.mutatePopulation()

        
    f.write("]}")
    print("Best individual")
    print("Aptitud:",gp.bestEver.aptitud)
    gp.bestEver.showTree()
    
    final_req = EvaluateDriver.Request()
    final_req.maxtime = 60
    final_req.tree_index = -1
    future = client.call_async(final_req)
    while not future.done():
        node.get_logger().info("MEJOR INDIVIDUO")
        executor.spin_once(timeout_sec=0.1)
    
    f.close()
    with open ('gp.dat', 'wb') as gpFile:
        pickle.dump(gp, gpFile )
        print('GP saved to gp.dat')

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()