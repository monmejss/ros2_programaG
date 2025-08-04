#!/usr/bin/env python3
from __future__ import annotations
from arlo_gp_controller.gp.gpTree import Tree
import rclpy
from rclpy.node import Node
from arlo_gp_controller.gp.gp import GeneticProgram

from arlo_interfaces.srv import EvaluateTree
from arlo_nn_controller.srv import EvaluateDriver

#used to save object to file
import pickle

class GeneticProgramNode(Node):
    def __init__(self):
        super().__init__('gp')


        print("Programa Genetico \n --------------------------------------\n")
        popSize = int(input("population size: "))
        generations = int(input("number of generations: "))
        treeDepth   = int(input("max tree depth: "))
        mutationProbability = float(input("mutation probability: "))
        
        self.gp = GeneticProgram(popSize, generations, treeDepth, 'full', mutationProbability)
        # Servicio para actuadores
        self.server = self.create_service(EvaluateTree, 'evaluate_tree', self.handleEvaluateTree)
        # cliente para evaluar controladores
        self.evaluateDriverClient = self.create_client(EvaluateDriver, 'evaluate_driver')
        while not self.evaluateDriverClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a evaluate_driver...')


        f = open("report.txt", "w")
        header = "{\n{popSize: "+str(popSize)+",\ngenerations: "+str(generations)+",\ntreeDepth: "+str(treeDepth)+",\n"+"treeType: full,\nmutationProbability: "+str(mutationProbability)+"},\n[\n"
        f.write(header)
        self.gp.setInitialPopulation()
        print("Genetic Program Start")
    
        for generation in range(self.gp.maxGen):
            print("generation:", generation)
            for index in range(self.gp.popSize):
                print("\tIndividual: ",index)
                request = EvaluateDriver.Request()
                request.maxtime = 60
                request.tree_index = index
                future = self.evaluateDriverClient.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                driverResponse = future.result()
                self.gp.setAptitude(index, driverResponse.dist2go)
            
                self.gp.setBestAptitud()
                self.gp.setBestParent()
                self.gp.setParents('torneo')

                if generation!= 0:  
                    self.gp.sortParents()
                aptitudPopulationAverage = sum(self.gp.aptitudes)/self.gp.popSize
                populationReport = ",{\nbestAptitud: "+str(self.gp.bestAptitud)+",\n"+"averageAptitud: "+str(aptitudPopulationAverage)+"\n}\n"
                f.write(populationReport)
                self.gp.cross()
                self.gp.mutatePopulation()
            
                f.write("]}")
                print("Best individual")
                print("Aptitud:",self.gp.bestEver.aptitud)
                self.gp.bestEver.showTree()

        request = EvaluateDriver.Request()
        request.maxtime = 60
        request.tree_index = -1
        future = self.evaluateDriverClient.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # save gp to file
        f.close()
        with open ('gp.dat', 'wb') as gpFile:
            pickle.dump(gp, gpFile )
            print('GP saved to gp.dat')
            
    def handleEvaluateTree(self, request, response):
        #individual = None
        evalBest = True if request.tree_index == -1 else False
        if evalBest : 
            individual =  self.gp.bestParent
        else :
            individual = self.gp.population[request.tree_index]

        actuatorValues = individual.evaluateTree(request.sensor_values)
        response.actuator_values = actuatorValues
        return response
            
            
def main():
    rclpy.init()
    node = GeneticProgramNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()