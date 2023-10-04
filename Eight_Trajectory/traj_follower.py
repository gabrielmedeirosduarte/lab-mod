#! /usr/bin/env python3

# Bibliotecas
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty

# ==========================================================================

# Geração da trajetória

# Tempo da trajetória
t0 = 0
tf = 100
Ts = 0.01
t = np.arange(t0, tf, Ts)

# Alteração da origem da trajetória
t1 = t[7501:-1]
t2 = t[0:2500]
t3 = t[2501:5000]
t4 = t[5001:7500]

# união do vetor de tempo
t = np.hstack((t1,t2,t3,t4))

# Cálculo da velocidade necessária para cumprir a trajetória em tf segundos
omega = 2*np.pi/tf

# Função da trajetória em x
def ref_x_traj(t):
    x = np.cos(omega*t)*2
    return x

# Função da trajetória em y
def ref_y_traj(t):
    y = -np.sin(2*omega*t)
    return y

# Trajetória de referência
pos = Odometry()

# inicialização da iteração do vetor de tempo
i = 0

# ==========================================================================

# Nó e Tópico
rospy.init_node('smb_simple_controller') 
pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub_traj = rospy.Publisher('/objetivo', Point, queue_size=10)
rate = rospy.Rate(100) 
move = Twist()

# Função pra pausar
def pause_simulation():
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_physics()
        rospy.loginfo("Simulation paused")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to pause_physics failed: %s", e)

# Função pra tirar a pausa
def unpause_simulation():
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
        rospy.loginfo("Simulation unpaused")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to unpause_physics failed: %s", e)
        
# Set the pause and unpause durations in seconds
pause_duration = 2.0  
unpause_duration = 0.01

# inicialização das variáveis de pose do robô e da traj
x_robot = 0
y_robot = 0
ang_robot = 0
x_traj = 0
y_traj = 0

# Função que retorna a posição e orientação do robô
def get_PosRobo(odom_data_r):
    
    rate.sleep()

    # Definição das variáveis do robô como globais
    global x_robot, y_robot, ang_robot

    # Posição do robô
    position_ = odom_data_r.pose.pose.position
    pos = [position_.x, position_.y, position_.z]
    x_robot = pos[0]
    y_robot = pos[1]

    # Orientação do robô
    orientation_ = odom_data_r.pose.pose.orientation
    orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
    orient = euler_from_quaternion(orientation)
    ang_robot = orient[2]

# Chamada da função get_PosRobo, cujo parâmetro é a pose do robô
sub_odom_robot = rospy.Subscriber('/odom', Odometry, get_PosRobo)  

# Constantes do controlador P
kp_dist = 1
kp_ang = 3

# Limites de saturação
lim_vel_lin = 0.5
lim_vel_ang = 3

# Controle de Trajetória
while (not rospy.is_shutdown()):
    
    rate.sleep()
    
    # Pausa a simulação
    pause_simulation()
    
    # Cálculo de x e y da trajetória
    x_traj = 5*ref_x_traj(t[i])
    y_traj = 5*ref_y_traj(t[i])
    
    # Incrementa a posição da trajetória
    i+= 1
    
    # Volta a trajetória pro início
    if i == len(t)-1:
        i = 0
    
    # Cálculo da diferença de posição e orientação entre o robô e o ponto atual da traj
    dist_X = x_traj - x_robot
    dist_Y = y_traj - y_robot
    dist = np.sqrt(dist_X**2 + dist_Y**2)
    ang = np.arctan2(dist_Y, dist_X)

    # Diferença de ângulo (erro)
    ang_error = np.arctan2(np.sin(ang - ang_robot), np.cos(ang - ang_robot))

    # Controle proporcional
    vel_lin = kp_dist * dist
    vel_ang = kp_ang * ang_error

    # Saturação
    vel_lin_sat = np.clip(vel_lin, -lim_vel_lin, lim_vel_lin)
    vel_ang_sat = np.clip(vel_ang, -lim_vel_ang, lim_vel_ang)

    # Acionamento do carrinho
    move.linear.x = vel_lin_sat
    move.angular.z = vel_ang_sat

    # Despausa a simulação e Publica da velocidade no Tópico
    unpause_simulation()
    pub_cmd.publish(move)
    rospy.sleep(unpause_duration)
    
    # Publicando a trajetória
    pub_traj.publish(x_traj, y_traj, 0)

# Finalização do rospy    
rospy.spin()