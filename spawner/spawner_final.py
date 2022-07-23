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
from find_edge.msg import brick

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
positions = []
path = Path(__file__).parent.absolute()
rotation1_x=[0]
rotation1_y=[0]
rotation1_z=[0]
rotation2_x=[0,-1.58,1.58,3.14]
rotation2_y=[0,-1.58,1.58,3.14]
rotation2_z=[0,-1.58,1.58,3.14]
posizione=[]
rotazione=[]
placement=[]


def get_quaternion_from_euler(a,b,c):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(a/2) * np.cos(b/2) * np.cos(c/2) - np.cos(a/2) * np.sin(b/2) * np.sin(c/2)
  qy = np.cos(a/2) * np.sin(b/2) * np.cos(c/2) + np.sin(a/2) * np.cos(b/2) * np.sin(c/2)
  qz = np.cos(a/2) * np.cos(b/2) * np.sin(c/2) - np.sin(a/2) * np.sin(b/2) * np.cos(c/2)
  qw = np.cos(a/2) * np.cos(b/2) * np.cos(c/2) + np.sin(a/2) * np.sin(b/2) * np.sin(c/2)
 
  return [qx, qy, qz, qw]

def spawn1():
    
    for i in range(11):
        for m in range(4):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i])+'-'+str(m))
            print("Deleted"+str(blocks[i])+'-'+str(m))

    area=[0]
    for m in range(len(area)):
        cicli = 1
        x_in = 0.35
        x_fin = 0.55
        y_in = -0.3
        y_fin = 0

        blocco=[]
        blocco= random.sample(blocks,cicli)
        for i in range(1):
            f=True
            #Generate random position
            if i==0:
                x,y,z,w= get_quaternion_from_euler(random.choice(rotation1_x),random.choice(rotation1_y),random.choice(rotation1_z))
                pos = Pose(Point((x_in+x_fin)/2, (y_in+y_fin)/2, 0.880), Quaternion(x,y,z,w))
                positions.append(pos)

            #Get a random lego block from all legos
            print("Il blocco per l'area "+str(m)+" è "+str(blocco[i]))
            brick=blocco[i]
            # print(pos)
            # print(str(brick))
            #Call rospy spawn function to spawn objects in gazebo
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_client(model_name=''+str(brick)+'-'+str(m), 
                model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=pos,
                reference_frame='world')
    
def spawn(msg):

    for i in range(11):
        for m in range(8):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i])+'-'+str(m))
            print("Deleted"+str(blocks[i])+'-'+str(m)+'\n')
            
    if msg.data== "a":
        print("sono dentro")
        area=[0,1,2,3]
        count=0
        cicli = 2
        for m in range(len(area)):
            if m==0:
                x_pos = 0.42
                y_pos = -0.22
            elif m==1:
                x_pos = 0.42
                y_pos = 0.079
            elif m==2:
                x_pos = 0.65
                y_pos = -0.22
            elif m==3:
                x_pos = 0.65
                y_pos = 0.079
           
            for i in range(cicli):
                f=True
                #Generate random position
                if i==0:
                    x,y,z,w= get_quaternion_from_euler(random.choice(rotation1_x),random.choice(rotation1_y),random.choice(rotation1_z))
                    pos = Pose(Point(x_pos, y_pos, 0.880), Quaternion(x,y,z,w))
                    positions.append(pos)
                    brick=blocks[count]
                    print("Il blocco per l'area "+str(m)+" è "+str(blocks[count]))
                    count+=1
                else:
                    x,y,z,w= get_quaternion_from_euler(random.choice(rotation1_x),random.choice(rotation1_y),random.choice(rotation1_z))
                    pos = Pose(Point(x_pos, y_pos+0.15, 0.880), Quaternion(x,y,z,w))
                    brick=blocks[count]
                    print("Il blocco per l'area "+str(m)+" è "+str(blocks[count]))
                    count+=1

                #Call rospy spawn function to spawn objects in gazebo
                spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_client(model_name=''+str(brick)+'-'+str(m), 
                    model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=pos,
                    reference_frame='world')

    elif msg.data=="b":
        area=[0,1,2,3]
        for m in range(4):
            cicli = 1
            if m==0:
                a=random.choice(blocks)
                x_in = 0.30
                x_fin = 0.50
                y_in = -0.28
                y_fin = -0.03
            elif m==1:
                a=random.choice(blocks)
                x_in = 0.30
                x_fin = 0.50
                y_in = 0.03
                y_fin = 0.32
            elif m==3:
                a=random.choice(blocks)
                x_in = 0.58
                x_fin = 0.78
                y_in = 0.05
                y_fin = 0.32
            elif m==2:
                x_in = 0.58
                x_fin = 0.78
                y_in = -0.28
                y_fin = 0.03

            x,y,z,w= get_quaternion_from_euler(random.choice(rotation2_x),random.choice(rotation2_y),random.choice(rotation2_z))
            pos = Pose(Point((x_in+x_fin)/2, (y_in+y_fin)/2, 0.880), Quaternion(x,y,z,w))
            positions.append(pos)
            #Get a random lego block from all legos
            print("Il blocco per l'area "+str(m)+" è "+str(a))
            brick=a
            # print(pos)
            print(str(brick))
            #Call rospy spawn function to spawn objects in gazebo
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_client(model_name=''+str(brick)+'-'+str(m), 
                model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=pos,
                reference_frame='world')

    elif msg.data=="c":
        area=[0,1,2,3]
        for m in range(len(area)):
            if m==0:
                cicli = 2
                y_pos=-0.22
                x_pos=0.42
            elif m ==1:
                cicli = 2
                y_pos=0.079
                x_pos=0.42
            elif m==2:
                cicli = 1
                y_pos=-0.15
                x_pos=0.65
            elif m==3:
                cicli = 1
                y_pos=0.15
                x_pos=0.65

            for i in range(cicli):
                f=True
                #Generate random position
                if i==0:
                    count=9
                    x,y,z,w= get_quaternion_from_euler(random.choice(rotation2_x),random.choice(rotation2_y),random.choice(rotation2_z))
                    pos = Pose(Point(x_pos, y_pos, 0.880), Quaternion(x,y,z,w))
                    positions.append(pos)
                else:
                    count=8
                    x,y,z,w= get_quaternion_from_euler(random.choice(rotation1_x),random.choice(rotation1_y),random.choice(rotation1_z))
                    pos = Pose(Point(x_pos, y_pos+0.15, 0.880), Quaternion(x,y,z,w))

                #Get a random lego block from all legos
                print("Il blocco per l'area "+str(m)+" è "+str(blocks[count]))
                brick=blocks[count]
                # print(pos)
                print(str(brick))
                #Call rospy spawn function to spawn objects in gazebo
                spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_client(model_name=''+str(brick)+'-'+str(m), 
                    model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=pos,
                    reference_frame='world')
        

def start_node():
    # rospy.init_node('spawner_2')
    # rospy.loginfo('Aspetto il topic che mi dice di spawnare i prossimi blocchi')
    rospy.Subscriber("/spawner/blocchi", String, spawn) #si iscrive al topic del kinect
    rospy.spin() #Continua a ciclare, evita la chiusura del nodo
    

if __name__ == '__main__':
    rospy.init_node('spawner_finale')
    rospy.loginfo('Aspetto il topic che mi dice di spawnare i prossimi blocchi')
    try:
        spawn1()
        start_node()
    except rospy.ROSInterruptException:
        pass
