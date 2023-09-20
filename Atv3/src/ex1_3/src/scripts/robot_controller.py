#! /usr/bin/env python3

# Bibliotecas
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion

# Nó e Tópico
rospy.init_node('smb_simple_controller') 
pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
rate = rospy.Rate(100) 
move = Twist()

# Número de pontos
n = 4

# Definição dos pontos
point_1 = [-3, 1]
point_2 = [3, -2]
point_3 = [5, 4]
point_4 = [-4, -4]
points = [point_1, point_2, point_3, point_4]

# Tolerância de distância
tol_dist = 0.35

# Constantes do controlador P
kp_dist = 1
kp_ang = 3

# Limites de saturação
lim_vel_lin = 0.5
lim_vel_ang = 2

# Contador de pontos
cont = 0

# Função de Controle de posição do robô
def Position_Controller(odom_data):
    rate.sleep()

    # Definição da variável cont como global
    global cont

    # Posição do robô
    position_ = odom_data.pose.pose.position
    pos = [position_.x, position_.y, position_.z]
    x_robot = pos[0]
    y_robot = pos[1]

    # Orientação do robô
    orientation_ = odom_data.pose.pose.orientation
    orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
    orient = euler_from_quaternion(orientation)
    ang_rob = orient[2]

    # Cálculo da diferença de oposição e orientação entre o robô e o ponto
    dist_X = points[np.clip(cont, 0, n-1)][0] - x_robot
    dist_Y = points[np.clip(cont, 0, n-1)][1] - y_robot
    dist = np.sqrt(dist_X**2 + dist_Y**2)
    ang = np.arctan2(dist_Y, dist_X)

    # Diferença de ângulo (erro)
    ang_error = np.arctan2(np.sin(ang - ang_rob), np.cos(ang - ang_rob))

    # Controle proporcional
    vel_lin = kp_dist * dist
    vel_ang = kp_ang * ang_error

    # Saturação
    vel_lin_sat = np.clip(vel_lin, -lim_vel_lin, lim_vel_lin)
    vel_ang_sat = np.clip(vel_ang, -lim_vel_ang, lim_vel_ang)

    # Acionamento do carrinho
    move.linear.x = vel_lin_sat
    move.angular.z = vel_ang_sat

    # Condição de parada ou troca de ponto
    if (dist < tol_dist):
        if cont == n:
            move.linear.x = 0.0
            move.angular.z = 0.0
        else:    
            cont += 1     

    # Publicação da velocidade no Tópico
    pub_cmd.publish(move)

# Chamada da função Position_Controller, cujo parâmetro é a pose do robô
sub_odom = rospy.Subscriber('/odom', Odometry, Position_Controller)    
rospy.spin()