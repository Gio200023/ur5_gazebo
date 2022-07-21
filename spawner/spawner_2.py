#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import *
from std_msgs.msg import String
import rospy
import random
import numpy as np
import time
import delete
import os
from pathlib import Path
import sys

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
positions = []
path = Path(__file__).parent.absolute()
rotation=[0,-1.58,1.58,3.14]

def get_quaternion_from_euler(*params):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(params[0]/2) * np.cos(params[1]/2) * np.cos(params[2]/2) - np.cos(params[0]/2) * np.sin(params[1]/2) * np.sin(params[2]/2)
  qy = np.cos(params[0]/2) * np.sin(params[1]/2) * np.cos(params[2]/2) + np.sin(params[0]/2) * np.cos(params[1]/2) * np.sin(params[2]/2)
  qz = np.cos(params[0]/2) * np.cos(params[1]/2) * np.sin(params[2]/2) - np.sin(params[0]/2) * np.sin(params[1]/2) * np.cos(params[2]/2)
  qw = np.cos(params[0]/2) * np.cos(params[1]/2) * np.cos(params[2]/2) + np.sin(params[0]/2) * np.sin(params[1]/2) * np.sin(params[2]/2)
 
  return qx, qy, qz, qw

def spawn1():
    
    for i in range(11):
        for m in range(4):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i])+'-'+str(m))
            print("Deleted"+str(blocks[i])+'-'+str(m))

    area=[0,1,2,3]
    for m in range(4):
        cicli = random.randint(1,2)
        if m==0:
            x_in = 0.35
            x_fin = 0.55
            y_in = -0.3
            y_fin = 0
        elif m ==1:
            x_in = 0.35
            x_fin = 0.55
            y_in = 0
            y_fin = 0.3
        elif m==3:
            x_in = 0.55
            x_fin = 0.75
            y_in = 0
            y_fin = 0.3
        elif m==2:
            x_in = 0.55
            x_fin = 0.75
            y_in = -0.3
            y_fin = 0

        blocco=[]
        blocco= random.sample(blocks,cicli)
        for i in range(cicli):
            f=True
            #Generate random position
            if i==0:
                x,y,z,w= get_quaternion_from_euler(*random.choices(rotation,k=3))
                pos = Pose(Point(random.uniform(x_in, x_fin), random.uniform(y_in, y_fin), 0.880), Quaternion(x,y,z,w))
                positions.append(pos)
            else:
                while f==True:
                    x,y,z,w= get_quaternion_from_euler(*random.choices(rotation,k=3))
                    pos = Pose(Point(random.uniform(x_in, x_fin), random.uniform(y_in, y_fin), 0.880), Quaternion(x,y,z,w))
                    for k in range(i):
                        threshold = 0.3
                        if np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2) < threshold:
                            break
                        if k == i-1:
                            positions.append(pos)
                            f = False

            #Get a random lego block from all legos
            print("Il blocco per l'area "+str(m)+" è "+str(blocco[i]))
            brick=blocco[i]
            print(pos)
            print(str(brick))
            #Call rospy spawn function to spawn objects in gazebo
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_client(model_name=''+str(brick)+'-'+str(m), 
                model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=pos,
                reference_frame='world')

def spawn(msg):

    for i in range(11):
        for m in range(4):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i])+'-'+str(m))
            print("Deleted"+str(blocks[i])+'-'+str(m)+'\n')

    area=[0,1,2,3]
    for m in range(4):
        cicli = random.randint(1,2)
        if m==0:
            x_in = 0.30
            x_fin = 0.50
            y_in = -0.28
            y_fin = -0.03
        elif m ==1:
            x_in = 0.30
            x_fin = 0.50
            y_in = 0.03
            y_fin = 0.32
        elif m==2:
            x_in = 0.58
            x_fin = 0.78
            y_in = 0.05
            y_fin = 0.32
        elif m==3:
            x_in = 0.58
            x_fin = 0.78
            y_in = -0.28
            y_fin = 0.03

        blocco=[]
        blocco= random.sample(blocks,cicli)
        for i in range(cicli):
            f=True
            #Generate random position
            if i==0:
                pos = Pose(Point(random.uniform(x_in, x_fin), random.uniform(y_in, y_fin), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
                positions.append(pos)
            else:
                while f==True:
                    pos = Pose(Point(random.uniform(x_in, x_fin), random.uniform(y_in, y_fin), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
                    for k in range(i):
                        threshold = 0.15
                        if np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2) < threshold:
                            break
                        if k == i-1:
                            positions.append(pos)
                            f = False

            #Get a random lego block from all legos
            print("Il blocco per l'area "+str(m)+" è "+str(blocco[i]))
            brick=blocco[i]
            # print(pos)
            print(str(brick))
            #Call rospy spawn function to spawn objects in gazebo
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_client(model_name=''+str(brick)+'-'+str(m), 
                model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=pos,
                reference_frame='world')
    os._exit(0)

def start_node():
    rospy.init_node('spawner_2')
    rospy.loginfo('Aspetto il topic che mi dice di spawnare i prossimi blocchi')
    rospy.Subscriber("/spawner/blocchi", String, spawn) #si iscrive al topic del kinect
    rospy.spin() #Continua a ciclare, evita la chiusura del nodo
    

if __name__ == '__main__':
    try:
        spawn1()
        start_node()
    except rospy.ROSInterruptException:
        pass
