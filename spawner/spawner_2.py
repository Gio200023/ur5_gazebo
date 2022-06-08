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

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
positions = []
path = Path(__file__).parent.absolute()

def spawn1():
    
    for i in range(11):
        for m in range(2):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i])+'-'+str(m))
            print("Deleted"+str(blocks[i])+'-'+str(m))

    brick_number=[0,0,0,0,0,0,0,0,0,0,0]
    cicli = random.randint(4,6)
    for i in range(cicli):
        f=True
        #Generate random position
        if i==0:
            pos = Pose(Point(random.uniform(0.35, 0.75), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
            positions.append(pos)
        else:
            while f==True:
                pos = Pose(Point(random.uniform(0.35, 0.75), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
                for k in range(i%11):
                    threshold = 0.125
                    if np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2) < threshold:
                        break
                    if k == (i%11)-1:
                        positions.append(pos)
                        f = False

        
        #Get a random lego block from all legos
        blocco=random.randint(0,10)
        brick=blocks[blocco]
        print(pos)
        print(brick)
        #Call rospy spawn function to spawn objects in gazebo
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name=''+str(brick)+'-'+str(brick_number[blocco]), 
            model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')
        brick_number[blocco]+=1

def spawn(msg):

    for i in range(11):
        for m in range(2):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i])+'-'+str(m))
            print("Deleted"+str(blocks[i])+'-'+str(m)+'\n')

    brick_number=[0,0,0,0,0,0,0,0,0,0,0]
    cicli = random.randint(4,6)
    for i in range(cicli):
        f=True
        #Generate random position
        if i==0:
            pos = Pose(Point(random.uniform(0.35, 0.75), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
            positions.append(pos)
        else:
            while f==True:
                pos = Pose(Point(random.uniform(0.35, 0.75), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
                for k in range(i%11):
                    threshold = 0.125
                    if np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2) < threshold:
                        break
                    if k == (i%11)-1:
                        positions.append(pos)
                        f = False

        
        #Get a random lego block from all legos
        blocco=random.randint(0,10)
        brick=blocks[blocco]
        print(pos)
        print(brick)
        #Call rospy spawn function to spawn objects in gazebo
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name=''+str(brick)+'-'+str(brick_number[blocco]), 
            model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')
        brick_number[blocco]+=1
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
