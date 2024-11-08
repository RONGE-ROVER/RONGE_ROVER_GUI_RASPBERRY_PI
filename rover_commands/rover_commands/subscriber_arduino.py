import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')

        # Créer un subscriber de type Twist qui appelle listener_callback
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_arduino',  # Le topic auquel s'abonner
            self.listener_callback,
            10  # Taille de la file d'attente
        )

        self.get_logger().info('Subscriber node has been started.')
       
        # Position initiale (renommée en data1, data2, data3)
        self.data1 = 0.0  # Correspond à x
        self.data2 = 0.0  # Correspond à z
        self.data3 = 0.0  # Correspond à ry

        # Configuration du port série
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Attendre un peu pour que la connexion série se stabilise
            self.get_logger().info('Serial port connected successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.serial_port = None

    def listener_callback(self, msg):
        # Mettre à jour les variables data1, data2, et data3 en fonction des commandes reçues
        self.data1 += msg.linear.x  # x
        self.data2 += msg.linear.z  # z
        self.data3 += msg.angular.z  # ry

        # Log de la nouvelle position
        self.get_logger().info(f'New Data: data1={self.data1}, data2={self.data2}, data3={self.data3}')

        # Envoyer les données à l'Arduino si le port série est connecté
        if self.serial_port:
            data_string = f'{self.data1},{self.data2},{self.data3}\n'
            try:
                self.serial_port.write(data_string.encode())
                self.get_logger().info(f'Sent to Arduino: {data_string}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending data to Arduino: {e}')

    def destroy_node(self):
        # Fermer le port série proprement lors de la destruction du nœud
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