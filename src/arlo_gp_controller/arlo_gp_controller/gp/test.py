from random import random
import random
from arlo_gp_controller.gp.gp import GeneticProgram
from arlo_gp_controller.gp.gpTree import Tree


if __name__ == "__main__":
    # t1 = Tree()
    # t1.createTreeFull(5)
    # print("[ Full method tree ]: ")
    # t1.showTree(spaces=2)
    # sensorValue = 5.0
    # robotAction = t1.evaluateTree(sensorValue)

    # print("\n\nResultado evaluación","\n\tAccción: ", robotAction)
    # print("\n\n")
    
    # print("\n")
    # node = t1.getRandomNode()
    # tt = Tree()
    # tt.root = node
    # print("\n Node: \n")
    # tt.showTree()
    # print("\n")


    # print(nodeArray)

    gp = GeneticProgram(4, 3, 5, 'full', 0.2)
    gp.showPopulation()
    # t1,t2 = gp.crossTest()

    # print("árboles con cruza")
    # t1.showTree()
    # t2.showTree()

