# import logging
# import sys

# # import config

# import droneapp.controllers.server

# logging.basicConfig(level=logging.INFO,
#                     stream=sys.stdout)
# # filename=config.LOG_FILE)

# if __name__ == '__main__':
#     droneapp.controllers.server.run()

#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess
from geometry_msgs.msg import PoseStamped
import pytello.droneapp.controllers.server
import time
import math
from datetime import datetime

time_fichero = datetime.now().strftime('%Y%m%d_%H%M%S')
fichero_tello = f"tello_pose_{time_fichero}.txt"
fichero_turtle = f"turtle_pose_{time_fichero}.txt"
tello_trajectory = f"tello_trajectory_{time_fichero}.txt"
turtle_trajectory = f"turtle_trajectory_{time_fichero}.txt"
fichero_resultado = f"resultados_{time_fichero}.txt"


tello_position = None
turtle_position = None

def calculate_error(pos1, pos2):
    # Calcular la diferencia en posición y orientación
    pos_error = math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)
    return pos_error

def guardar_resultados(tello, turtle, error):
    with open(fichero_resultado, 'a') as fichero:
        fichero.write(f"{error}, {tello.x}, {tello.y}, {tello.z}, {turtle.x}, {turtle.y}, {turtle.z}\n")

def callbackTello(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard from tello %s", data)
    
    with open(fichero_tello, 'a') as fichero:
        # Escribe la información recibida
        fichero.write("Tello data: " + str(data) + "\n")

    with open(tello_trajectory, 'a') as fichero:
        global tello_position, turtle_position
        # Escribe la información recibida
        tello_position = data.pose.position
        orientation = data.pose.orientation
        fichero.write(str(time.time()) + " " + str(tello_position.x) + " " + str(tello_position.y) + " " +
            str(tello_position.z) + " " + str(orientation.x) + " " + str(orientation.y) + " " + str(orientation.z) + 
            " " + str(orientation.w) + "\n")

        # rospy.loginfo("tello pos %s", tello_position)
        # rospy.loginfo("turtle pos %s", turtle_position)

        if tello_position != None and turtle_position != None:
            error = calculate_error(tello_position, turtle_position)
            guardar_resultados(tello_position, turtle_position, error)


def callbackTurtle(data):
    global turtle_position
    # rospy.loginfo(rospy.get_caller_id() + "I heard from turtle %s", data)
    with open(fichero_turtle, 'a') as fichero:
        # Escribe la información recibida
        fichero.write("Turtle data: " + str(data) + "\n")

    with open(turtle_trajectory, 'a') as fichero:
        # Escribe la información recibida
        turtle_position = data.pose.position
        orientation = data.pose.orientation
        fichero.write(str(time.time()) + " " + str(turtle_position.x) + " " + str(turtle_position.y) + " " +
            str(turtle_position.z) + " " + str(orientation.x) + " " + str(orientation.y) + " " + str(orientation.z) + 
            " " + str(orientation.w) + "\n")

def publish_terminal_output():
    # Inicializa el nodo ROS
    
    
    # Ejecuta la aplicación que controla el dron y redirige la salida
    proceso = subprocess.Popen(['python3', 'src/pytello/scripts/init.py'], stdout=subprocess.PIPE)
    # pytello.droneapp.controllers.server.run()
    rospy.init_node('subscriber', anonymous=True)
    
    subTello = rospy.Subscriber('/vrpn_client_node/tello_3/pose', PoseStamped, callbackTello)
    subTurtle = rospy.Subscriber('/vrpn_client_node/turtlebot/pose', PoseStamped, callbackTurtle)
    # while not rospy.is_shutdown():
    #     # Lee la salida de la aplicación línea por línea
    #     linea = proceso.stdout.readline().strip()
    #     if not linea:
    #         break
        
    #     # Publica la línea como un mensaje en ROS
    #     rospy.loginfo(linea)
    #     sub.publish(linea.decode('utf-8'))
        
    # Espera a que el proceso finalice
    proceso.wait()

if __name__ == '__main__':
    try:
        publish_terminal_output()
    except rospy.ROSInterruptException:
        pass