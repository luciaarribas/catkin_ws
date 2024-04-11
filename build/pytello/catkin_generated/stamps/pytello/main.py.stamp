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

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def publish_terminal_output():
    # Inicializa el nodo ROS
    rospy.init_node('subscriber', anonymous=True)
    
    sub = rospy.Subscriber('/vrpn_client_node/Tello_1/pose', PoseStamped, callback)
    
    # Ejecuta la aplicación que controla el dron y redirige la salida
    proceso = subprocess.Popen(['python3', 'src/pytello/scripts/init.py'], stdout=subprocess.PIPE)
    # pytello.droneapp.controllers.server.run()
    
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