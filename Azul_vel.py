import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

#Definición de la clase 
class Velocity(Node):

    #Constructor 
    def __init__(self):

        #Parametros velocidad
        self.thp = 0.0
        self.xp = 0.0
        self.yp = 0.0

        #Parametros posicion
        self.th = 0.0
        self.x = 0.0
        self.y = 0.0

        #Parámetros físicos
        self.r = 0.19
        self.l = 0.05

        #Velocidades angulares de encoders 
        self.velR = 0
        self.velL = 0

        #Definición del tópico
        super().__init__('Azul_Velocity')

        #Creación de suscriptores para leer datos de encoders R L
        self.subR = self.create_subscription(Float32, 'VelocityEncR', self.listener_callbackR, rclpy.qos.qos_profile_sensor_data)
        self.subL = self.create_subscription(Float32, 'VelocityEncL', self.listener_callbackL, rclpy.qos.qos_profile_sensor_data)

        #Confirmación de la creación del nodo
        self.get_logger().info('Velocity node successfully initialized!!!')

        #Definición del periodo de tiempo
        timer_period = 0.5

        #Timer para operaciones
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Creación del tópico publicador: Azul_Velocity
        self.publisher = self.create_publisher(Pose2D, 'Azul_Velocity', 10)

        #Tipo de mensaje 
        self.msg = Float32()
    
    #Lectura del encoder Velocity R
    def listener_callbackR(self, msg):

        #Guarda el dato
        self.velR = msg.data

    #mLectura del encoder Velocity L
    def listener_callbackL(self, msg):

        #Guarda el dato
        self.velL = msg.data

    #Método del timer
    def timer_callback(self):

        #Fórmulas para obtener las velocidades
        self.thp = self.r * ((self.velR - self.velL)/self.l)
        self.th += self.thp * 0.5

        self.xp = self.r * ((self.velR + self.velL)/2) * np.cos(self.th)
        self.x += self.xp * 0.5

        self.yp = self.r * ((self.velR + self.velL)/2) * np.sin(self.th)
        self.y += self.yp * 0.5

        #Publicación del mensaje en formato Pose2D
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.th
        self.publisher.publish(pose_msg)

#Main Fnc
def main(args=None):
    #Inicialiation for rclpy 
    rclpy.init(args=args)
    #create node
    m_p = Velocity()
    #Spin method for publisher calback
    rclpy.spin(m_p)
    #Destoy node
    m_p.destroy_node()
    #rclpy shutdown
    rclpy.shutdown()

#main call method
if __name__ == 'main':    
    main()