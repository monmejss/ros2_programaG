#!/usr/bin/env python3
#---------------------------------------------------------Trayectoria Obstaculo--------------------------------------------------------
# license removed for brevity
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import isinf
import message_filters
from numpy import linalg as LA
import math
import time


#sensores
rangosF = np.zeros(shape = (32,1)) #front
rangosL = np.zeros(shape = (32,1)) #left
rangosR = np.zeros(shape = (32,1)) #right

tp = 3 
tfin_p = 6
tfin = 9

d1 = 0.5
d2 = 0.5

Vi = 1.5
w  = 0.01


def fx1(t):
    return Vi*t + (2*w) * (t)**2

def d1x1(t):
    return Vi + 4*t*w

#Constantes
A = 0.09
a1 = 0
b1 = math.sqrt( ( Vi / (abs(d1x1(0)))  ) ** 2  - 1)
c1 = (math.sqrt ( (A/2) - w**2) )/Vi**2

def fy1(t):
    return (c1 * (fx1(t))**2)
def C1(t):
    return np.array ([fx1(t), fy1(t)])

def tramo1(t):
    x1 = fx1(t)
    y1 = fy1(t)
    x2 = fx1(t+0.5)
    y2 = fy1(t+0.5)
 
    vector = np.array ([x2-x1, y2-y1])
    
    magnitud = LA.norm(vector)

    angulo = math.degrees(math.atan((y2-y1)/(x2-x1)))

    angulo = (math.pi * (angulo))/180

    theta = angulo

    vl = magnitud/3
    va = (angulo)/3

    return np.array ([vl, va])


#-----------------------------------------------------Segundo tramo----------------------------------------------------
Beta = 2*c1*fx1(tp)

def fx2(t):
    #return fx1(tp) + (Vi + ((w*tp)/2)*t)
    #global theta
    global tp
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    #Beta = 2*c1*fx1(tp)


    return fx1(tp) + (Vi + (2*w*tp))*t

def fy2(t):
    #global theta
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    global tp
    #Beta = 2*c1*fx1(tp)
    return fy1(tp) + (Beta * ( fx2(t) - fx1(tp)))

def C2(t):
    #global theta
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    global tp
    #Beta = 2*c1*fx1(tp)
    return np.array ([fx2(t), fy2(t)])

def tramo2(t):
    x1 = fx2(t)
    y1 = fy2(t)
    x2 = fx2(t+0.5)
    y2 = fy2(t+0.5)

    #print("f(t): ",y1, "f(t+x): ",y2)
    #x1, y1 inicial | x2, y2 final 
    vector = np.array ([x2-x1, y2-y1])

    #global theta
    global tp
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    
    #Beta = 2*c1*fx1(tp)
    ##global theta
    #print("Valor de theta2: ", theta[0])

    #print(vector)
    magnitud = LA.norm(vector)
    #print("M: ",magnitud)
    angulo = math.degrees(math.atan((y2-y1)/(x2-x1)))
    #print("A: ",angulo)

    angulo = (math.pi * (angulo))/180
    
    vl = magnitud/3
    va = (angulo)/3

    return np.array ([vl, 0]) #Regresamos 0 en la va pues es un movimiento rectilineo uniforme


#-----------------------------------------------Tercer Tramo----------------------------------------------------------

#Si comenzamos con fx1 y fy1 se obtienen los resultados de la primera curva pero invertidos
def fx3(t):
    global tfin
    global p
    global tfin_p
    #global theta
    global tp
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    #Beta = 2*c1*fx1(tp)
    return fx1(tfin_p) + fx1(tp) - fx1(tp - t) 


def fy3(t):
    global tfin
    global p
    global tfin_p
    #global theta
    global tp
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    #Beta = 2*c1*fx1(tp)
    return fy1(tfin_p) + fy1(tp) - fy1(tp - t)


def C3(t):
    return np.array ([fx3(t), fy3(t)])

def tramo3(t):
    x1 = fx3(t)
    y1 = fy3(t)
    x2 = fx3(t+0.5)
    y2 = fy3(t+0.5)

    #print("f(t): ",y1, "f(t+x): ",y2)
    #x1, y1 inicial | x2, y2 final 
    vector = np.array ([x2-x1, y2-y1])

    #global theta
    global tp
    #tp = ((-c1*Vi) + math.sqrt( (c1**2)*(Vi**2) + 2*c1*w*math.tan(theta) )) / (2*c1*w) 
    #Beta = 2*c1*fx1(tp)

    #print(vector)
    magnitud = LA.norm(vector)
    #print("M: ",magnitud)
    angulo = math.degrees(math.atan((y2-y1)/(x2-x1)))
    #print("A: ",angulo)

    angulo = (math.pi * (angulo))/180
    
    vl = magnitud/3
    va = ((angulo)/3) * -1

    return np.array ([vl, va]) 
#---------------------------------------------------
def callbackObstaculo(scanner_center,scanner_left, scanner_right):
    global rangosF
    global rangosL
    global rangosR
    rangosF = scanner_center.ranges
    rangosL = scanner_left.ranges
    rangosR = scanner_right.ranges 


def controlarRobotObstaculo(node):
   global rangosF
   global rangosL
   global rangosR

   pubVel = node.create_publisher(Twist, '/cmd_vel', 10)
   time.sleep(0.5)

   t = 0
   t2 = 0
   t3 = 0
   retraso = 0
   while rclpy.ok():
        vel_msg = Twist()
        if(retraso > 4 or not(isinf(rangosF[16]))):
            if((isinf(rangosF[16]) and isinf(rangosR[16])) and t >= 8 ): #or rangosF[16]>2.5
                if(t>=8 and t<11):
                    print("----Tramo4 ----")
                    vel_msg.linear.x, vel_msg.angular.z = tramo1(t2)
                    vel_msg.angular.z = vel_msg.angular.z *(-1)   
                    t2 = t2 + 0.5 
                    t = t + 0.5       
                elif(t>=8 and t<13):
                    print("----Tramo5----")
                    vel_msg.linear.x, vel_msg.angular.z = tramo2(t2)
                    vel_msg.angular.z = vel_msg.angular.z *(-1)  
                    t2 = t2 + 0.5
                    t = t + 0.5

                elif(t>=8 and t<16):
                    print("----Tramo6----")
                    vel_msg.linear.x, vel_msg.angular.z = tramo3(t3)
                    vel_msg.angular.z = vel_msg.angular.z 
                    t3 = t3+0.5
                    t = t + 0.5
                elif(t>=8 and t>=16):
                    print("----Se acabo la trayectoria----")
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                    t = t + 0.5
                else:
                    vel_msg.linear.x = 0.4
                    vel_msg.angular.z = 0
                    t = t + 0.5
                pass
            else:

                if(t<3):
                    print("----Tramo1----")
                    vel_msg.linear.x, vel_msg.angular.z = tramo1(t) 
                elif (t<5):
                    print("----Tramo2----")
                    vel_msg.linear.x, vel_msg.angular.z = tramo2(t)
                elif (t<8):
                    print("----Tramo3----")
                    vel_msg.linear.x, vel_msg.angular.z = tramo3(t3)
                    t3 = t3+0.5
                t = t + 0.5
                pass
        else:
            print("--Retraso--")
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0  
            t = 0
            retraso = retraso + 1  



        pubVel.publish(vel_msg)
        print(f"Vl: {vel_msg.linear.x}, Va: {vel_msg.angular.z}")
        print("SensorF: ", rangosF[16])
        print("t: ", t)
        rate.sleep()


def funcion1(t):
    return 0.3 + ((-1.5-0.3)/(1+(t/0.1)**2.4))

def getVel(t):
    x1 = t
    y1 = funcion1(x1)
    x2 = x1 + 0.5
    y2 = funcion1(x2)    
    vector = np.array ([x2-x1, y2-y1])
    magnitud = LA.norm(vector)
    angulo = math.degrees(math.atan((y2-y1)/(x2-x1)))
    angulo = (math.pi * (90-angulo))/180
    
    vl = magnitud/3
    va = (angulo)/3
    v = np.array ([vl, va])
    return v


def callback(scanner_center,scanner_left, scanner_right):
    global rangosF
    global rangosL
    global rangosR
    rangosF = scanner_center.ranges
    rangosL = scanner_left.ranges
    rangosR = scanner_right.ranges 


def controlarRobot(node):
   global rangosF
   global rangosL
   global rangosR
   pubVel = node.create_publisher(Twist, '/cmd_vel', 10)   
   vel_msg = Twist()
   time.sleep(0.5)

   while rclpy.ok():
    if(isinf(rangosF[16])):
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = 0.0
        pubVel.publish(vel_msg)
        print("not turning anymore")
        
        if(isinf(rangosL[16]) and isinf(rangosR[16])):
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            pubVel.publish(vel_msg)
            print("not moving anymore")
            break
        break

    else:
        print("rango menor a un metro, vuelta")
        print(rangosF[16], rangosL[16], rangosR[16])
        if((rangosF[16]<=0.16 or rangosL[16] <= 0.20 or rangosR[16] <= 0.20 )): 
            print("robot atorado")
            break
        if(rangosF[16]<1.2): 
          vel_msg.linear.x, vel_msg.angular.z = getVel(0.5)
        else:
            print("Se detecto")
            vel_msg.linear.x, vel_msg.angular.z = getVel(0)


    pubVel.publish(vel_msg)
    print(f"Vl: {vel_msg.linear.x}, Va: {vel_msg.angular.z}")
    time.sleep(0.02)  # 50 Hz en vez de rclpy.Rate


def turnLeft(node):
    pubVel = node.create_publisher(Twist, '/cmd_vel', 10)
    vel_msg = Twist()

    # Gira hacia la izquierda en el lugar
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.5

    # Mantener giro durante 1 segundo
    for _ in range(10):
        pubVel.publish(vel_msg)
        time.sleep(0.1)

    # Detener después del giro
    vel_msg.angular.z = 0.0
    pubVel.publish(vel_msg)


def main():
    rclpy.init()
    node = rclpy.create_node('trayectorias_robot')
    scanner_center = message_filters.Subscriber(node, LaserScan, '/arlo/laser/scan_center')
    scanner_left = message_filters.Subscriber(node, LaserScan, '/arlo/laser/scan_left')
    scanner_right = message_filters.Subscriber(node, LaserScan, '/arlo/laser/scan_right')
    ts = message_filters.TimeSynchronizer([scanner_center, scanner_left, scanner_right], 10)
    ts.registerCallback(callback)

    controlarRobot(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()