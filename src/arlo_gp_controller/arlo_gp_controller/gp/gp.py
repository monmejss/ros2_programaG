#!/usr/bin/env python3
from __future__ import annotations
from arlo_gp_controller.gp.gpTree import Tree
from arlo_gp_controller.gp.gpNode import Node
import random
import numpy

#Import ros libraries
class GeneticProgram:
    def __init__(self, popSize, maxGen, maxDepth, treeType, pm) -> None:
        """ popsize: population size
            maxGen: Maximum amount of generations
            maxDepth: max depth for the trees in population
            treeType: tree type, full or grow
            pm: probability of mutation
        """
        self.popSize: int   = popSize
        self.maxGen: int    = maxGen
        self.maxDepth: int  = maxDepth
        self.treeType: str  = treeType
        self.pm: float      = pm
        self.population:list[Tree] = []
        self.aptitudes:list = None
        self.bestAptitud = -9999
        self.bestParent:Tree = None
        self.bestEver: Tree = None
        self.parents: list[int] = []

    def sortParents(self):
        parentDict = []
        for parentIndex in self.parents:
            parentDict.append({'parent': parentIndex,'aptitud':self.aptitudes[parentIndex]})

        parentDict = sorted(parentDict, key=lambda x: x['aptitud'], reverse=True)
        print(parentDict)
        for i in range(len(parentDict)):
            self.parents[i]=parentDict[i]['parent']
        print("best replaces worst: ", )
        self.population[self.parents[-1]] = self.bestEver
        self.aptitudes[-1]=self.bestEver.aptitud
        parentDict = []
        for parentIndex in self.parents:
            parentDict.append({'parent': parentIndex,'aptitud':self.aptitudes[parentIndex]})
        print(parentDict)

    def sortPop(self):
        parentDict = []
        for parentIndex in self.parents:
            parentDict.append({'parent': parentIndex,'aptitud':self.aptitudes[parentIndex]})

        parentDict = sorted(parentDict, key=lambda x: x['aptitud'], reverse=True)
        print(parentDict)
        for i in range(len(parentDict)):
            self.parents[i]=parentDict[i]['parent']
            self.aptitudes[i] =parentDict[i]['aptitud']
        print("best replaces worst")
        self.population[self.parents[-1]] = self.bestEver
        self.aptitudes[-1]=self.bestEver.aptitud
        parentDict = []
        for parentIndex in self.parents:
            parentDict.append({'parent': parentIndex,'aptitud':self.aptitudes[parentIndex]})
        print(parentDict)




    
    def mutatePopulation(self):
        for individual in self.population:
            individual.mutate(self.pm)

    def cross(self):
        
        offspring:list[Tree] = []
        for i in range(0,self.popSize,2):
            parent1 = self.population[self.parents[i]]
            parent2= self.population[self.parents[i+1]]
            # print(parent1,i)
            # print(parent2,i+1)setAptitude
            newOffspring1, newOffspring2 = self.__cross(parent1, parent2)
            offspring.append(newOffspring1)
            offspring.append(newOffspring2)
        #will use generational selection for the moment
        #now using elitism
        self.population = offspring

    def setInitialPopulation(self):
        return self._initialPopulation(self.popSize, self.maxDepth, self.treeType)

    def _initialPopulation(self, popSize,maxDepth,treeType):
        #Generate random population
        self.aptitudes = [None]*popSize
        if treeType == "full":
            for i in range(popSize):
                self.population.append(Tree())
                self.population[i].createTreeFull(maxDepth)
        else:
            for i in range(popSize):
                self.population.append(Tree())
                self.population[i].createTreeGrow(maxDepth)
        self.bestEver = self.population[0]
        return self.population
            

    def setAptitudes(self):
        #evaluate Individual
        for i, individual in enumerate(self.population):
            #get sensor value from sim
            sensorValue= 5 #simulates getting closer to target for everyone
            #evaluate
            individual.evaluateTree(sensorValue)
            #get new sensor value from sim
            newSensorValue = sensorValue * random.uniform(0.0,1.0)
            individual.aptitud = 1.0/ sensorValue
            self.aptitudes[i]= individual.aptitud
        
        self.bestAptitud = max(self.aptitudes)
        self.bestParent  = self.population[self.aptitudes.index(self.bestAptitud)]

    
    def setBestAptitud(self):
        self.bestAptitud = max(self.aptitudes)
    
    def setBestParent(self):
        self.bestParent  = self.population[self.aptitudes.index(self.bestAptitud)]
    
    
    def setAptitude(self, individualIndex: int, dist2go) -> None:
        """ sets aptitude for invidual given in population
            individualIndex: index in population list in
        """
        #gets sensor value from simulation, has to call evaluateDriver service
        # sensorValue= 5 #simulates getting closer to target for everyone
        #evaluate
        individual = self.population[individualIndex]
        #get dist2go value from sim
        if dist2go == 0.0:
            dist2go = 0.0000000000000000001
            self.aptitud = float('inf')
            individual.aptitud = 1.0/ dist2go
        else:
            individual.aptitud = 1.0/ dist2go
        print('apt: ', individual.aptitud)
        self.aptitudes[individualIndex]= individual.aptitud
        if self.bestEver != None and individual.aptitud > self.bestEver.aptitud:
            print('new best ever:',individual.aptitud, 'index: ',individualIndex)
            self.bestEver = individual
        elif self.bestEver == None:
            self.bestEver = individual
    
    def __cross(self, A:Tree, B:Tree):
        #copies of A and B
        A2 = A.copyTree()
        B2 = B.copyTree()

        node1 = A2.getRandomNode()
        node2 = B2.getRandomNode()

        p1 = node1.parent
        p2 = node2.parent
        index1 = 0
        index2 = 0
        if p1 != None:
            # print(node1.info, node2.info)
            for i, child in enumerate(p1.children):
                # print(child, node1)
                if child == node1: 
                    index1 = i
        if p2 != None:
            for i, child in enumerate(p2.children):
                # print(child, node2)
                if child == node2: 
                    index2 = i
        
        # print("\n\n",p1.children[index1].info, node1.info)
        # print("",p2.children[index2], node2)
        #Exchange nodes
        #node1 is root node and node2 is root node
        if p1 == None and p2 == None:
            A2.root = node2
            B2.root = node1
        #node1 is root node node2 is not root
        elif p1 == None and p2 != None:
            A2.root = node2
            p2.children[index2] = node1
            node1.parent = p2
        #node2 is root node node1 is not root
        elif p1 != None and p2 == None:
            B2.root = node1
            p1.children[index1] = node2
            node2.parent=p1
        #node1 and node2 are not root nodes
        else: 
            p1.children[index1] = node2
            node2.parent=p1
            p2.children[index2] = node1
            node1.parent = p2
        # print("\n\n",p1.children[index1], node1)
        # print("",p2.children[index2], node2)

        return A2, B2

    def setParents(self, selectionType):
        """ sets the parent list for new generation in population
        selectionType: 'torneo', 'ruleta'
        """
        self.parents = self.parentSelection(self.popSize, self.aptitudes,selectionType)

    def parentSelection(self,m, aptitud, sel):
        """m: size of population
            aptitud: parent's list of aptitud
            sel: type of selection, tornament"""
        ParentIndexes = [None] * m
        for i in range(m):
            ParentIndexes[i]= self.selection(aptitud, sel)
        return ParentIndexes

    def selection (self,aptitud, sel):
        if sel == "torneo":
            ind = self.tournament(aptitud, 2)
        else:
            ind = self.roulette(aptitud)
        return ind


    # Seleccion por torneo.
    # Devuelve el indice del mejor individuo de t seleccionados aleatoriamente.
    def tournament(self,aptitud, t):
        # Se eligen t individuos de forma aleatoria
        sel = random.sample(range(len(aptitud)), t)
        
        # Se obtienen las aptitudes de los individuos seleccionados
        aptitudsel = list(aptitud[i] for i in sel)
        
        #Se encuentra al individuo con mejor aptitud
        win = aptitudsel.index(max(aptitudsel))

        return sel[win]


    # Seleccion por ruleta.
    # Devuelve el indice del individuo que gano en la ruleta.
    def roulette (self, aptitud):
        phi = sum(aptitud)
        rho = random.random()
        suma = 0
        while suma < rho:
            i = random.choice(range(len(aptitud)))
            suma = suma + aptitud[i] / phi
        return i

    def crossTest(self):
        return self.__cross(self.population[0],self.population[1])

    def showPopulation(self):
        for i,tree in enumerate(self.population):
            print("arbol ", i,": ")
            tree.showTree()
            print("\n\n")
        