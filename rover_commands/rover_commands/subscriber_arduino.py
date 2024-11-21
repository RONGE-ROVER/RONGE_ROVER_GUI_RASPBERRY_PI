#Code pour la rpi4 en tant que master

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')

        # Créer un subscriber de type Twist
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_arduino',  # Le topic auquel s'abonner
            self.listener_callback,
            10  # Taille de la file d'attente
        )

        self.get_logger().info('Subscriber node has been started.')

        # Paramètres fixes du véhicule
        self.L = 2.5  # Empattement (en mètres)
        self.W = 1.5  # Voie (en mètres)

        # Position initiale pour l'angle et la vitesse
        self.data1 = 0.0  # Angle accumulé autour de l'axe z on envoie une valeur en radians
        self.data2 = 0.0  # Vitesse longitudinale (centre de masse)

        # Configuration des ports série pour plusieurs Arduino slaves
        try:
            self.serial_ports = [
                serial.Serial('/dev/ttyUSB0', 9600, timeout=1),  # Arduino Nano 1
                serial.Serial('/dev/ttyUSB1', 9600, timeout=1),  # Arduino Nano 2
                serial.Serial('/dev/ttyUSB2', 9600, timeout=1),  # Arduino Nano 3
            ]
            time.sleep(2)  # Attendre pour stabiliser les connexions série
            self.get_logger().info('Serial ports connected successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to one or more serial ports: {e}')
            self.serial_ports = []

    def listener_callback(self, msg):
        # Mettre à jour l'angle accumulé et la vitesse en fonction des commandes reçues 
        #NE PAS OUBLIER DE METTRE DES BORNES POUR NE PAS INCREMENTER DANS LE VIDE
        self.data1 += msg.angular.z  # ry permet de donner l'angle de rotation
        self.data2 += msg.linear.x   # x donne la vitesse de x

        # Calcul du rayon de braquage R
        if self.data1 == 0:
            R = 1e3  # Une valeur suffisamment grande (1 km) pour simuler une trajectoire rectiligne
        else:
            R = self.L / math.tan(self.data1) #tan(θ) est calculé en radians. Assure-toi que data1 (l'angle) est bien en radians avant ce calcul.


        # Log des nouvelles données calculées
        self.get_logger().info(f'L={self.L}, W={self.W}, v={self.data2:.2f}, R={R:.2f}')

        # Format des données à envoyer : "L,W,v,R"
        data_string = f'{self.L},{self.W},{self.data2:.2f},{R:.2f}\n'

        # Envoi des données à chaque Arduino slave
        for i, serial_port in enumerate(self.serial_ports):
            try:
                serial_port.write(data_string.encode())
                self.get_logger().info(f'Sent to Arduino {i + 1}: {data_string.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending data to Arduino {i + 1}: {e}')

    def destroy_node(self):
        # Fermer proprement les ports série lors de la destruction du nœud
        for serial_port in self.serial_ports:
            if serial_port:
                serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
