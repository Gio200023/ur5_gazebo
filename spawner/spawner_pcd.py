#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import *
from std_msgs.msg import String
from std_msgs.msg import Int64
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

pub = rospy.Publisher('/semaforo',Int64, queue_size=11)

contatore=0

def spawn(msg):
    print("sono dentro spawn)")
    global contatore
    print("msg è ======"+str(msg))
    if msg == Int64(1):
        for i in range(11):
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_client(str(blocks[i]))
        print("dopo eliminazione")
        time.sleep(1)
        if contatore < 12:
            pos = Pose(Point(0.55, 0, 0.775), Quaternion(0,0,0, 0))

            #Get a random lego block from all legos
            brick=blocks[contatore]
            #Call rospy spawn function to spawn objects in gazebo
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_client(model_name=''+str(brick), 
                model_xml=open(str(path)+'/../models/'+brick+'/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=pos,
                reference_frame='world')
            contatore+=1
        else:
            os._exit(1)

def start_node():
    rospy.init_node('spawner_pcd')
    rospy.loginfo('Aspetto il topic che mi dice di spawnare i prossimi blocchi')
    print("prima di sub")
    rospy.Subscriber("/semaforo",Int64,spawn)
    pub.publish(1)
    rospy.spin() #Continua a ciclare, evita la chiusura del nodo
    

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
