import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math


class TrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_subscriber')

        # Création d'un subscriber pour recevoir les commandes de vitesse et rotation
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel_arduino', self.listener_callback, 10)
        self.get_logger().info('Subscriber node has been started.')

        # Configuration des paramètres fixes du véhicule
        self.L = 2.5  # Empattement en mètres
        self.W = 1.5  # Voie en mètres
        self.data1 = 0.0  # Angle accumulé (rotation autour de l'axe Z)
        self.data2 = 0.0  # Vitesse longitudinale

        # Configuration du port série pour l'Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Attente pour la connexion
            self.get_logger().info('Serial port connected successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to the serial port: {e}')
            self.serial_port = None

    def listener_callback(self, msg):
        # Mise à jour des données d'angle et de vitesse
        self.data1 += msg.angular.z
        self.data2 += msg.linear.x

        # Calcul du rayon de braquage
        R = 1e3 if self.data1 == 0 else self.L / math.tan(self.data1)

        # Calcul des angles et vitesses pour chaque roue
        if R != 0:
            R_int = R - self.W / 2
            R_ext = R + self.W / 2

            angle_int_haut = math.atan(self.L / R_int)
            angle_ext_haut = math.atan(self.L / R_ext)
            angle_int_bas = -angle_int_haut
            angle_ext_bas = -angle_ext_haut

            vitesse_int_haut = self.data2 * R_int / R
            vitesse_ext_haut = self.data2 * R_ext / R
            vitesse_int_bas = vitesse_int_haut
            vitesse_ext_bas = vitesse_ext_haut
            vitesse_int_mid = vitesse_int_haut
            vitesse_ext_mid = vitesse_ext_haut
        else:
            # Trajectoire rectiligne
            angle_int_haut = angle_ext_haut = angle_int_bas = angle_ext_bas = 0.0
            vitesse_int_haut = vitesse_ext_haut = vitesse_int_bas = vitesse_ext_bas = self.data2
            vitesse_int_mid = vitesse_ext_mid = self.data2

        # Journalisation des résultats calculés
        self.get_logger().info(f'L = {self.L}, W = {self.W}, v = {self.data2:.2f}, R = {R:.2f}')
        self.get_logger().info(f'Angles: int_haut = {angle_int_haut:.2f}, ext_haut = {angle_ext_haut:.2f}, '
                               f'int_bas = {angle_int_bas:.2f}, ext_bas = {angle_ext_bas:.2f}')
        self.get_logger().info(f'Vitesses: int_haut = {vitesse_int_haut:.2f}, ext_haut = {vitesse_ext_haut:.2f}, '
                               f'int_bas = {vitesse_int_bas:.2f}, ext_bas = {vitesse_ext_bas:.2f}, '
                               f'int_mid = {vitesse_int_mid:.2f}, ext_mid = {vitesse_ext_mid:.2f}')

        # Préparation des données pour l'envoi
        data_string = f'{angle_int_haut:.2f},{vitesse_int_haut:.2f},' \
                      f'{angle_ext_haut:.2f},{vitesse_ext_haut:.2f},' \
                      f'{angle_int_bas:.2f},{vitesse_int_bas:.2f},' \
                      f'{angle_ext_bas:.2f},{vitesse_ext_bas:.2f},' \
                      f'{vitesse_int_mid:.2f},{vitesse_ext_mid:.2f}\n'

        # Envoi des données au microcontrôleur Arduino
        if self.serial_port:
            try:
                self.serial_port.write(data_string.encode())
                self.get_logger().info(f'Sent to Arduino: {data_string.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending data to Arduino: {e}')

    def destroy_node(self):
        # Fermeture propre du port série
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
