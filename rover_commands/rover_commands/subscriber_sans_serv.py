import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        # Créer un subscriber de type Twist qui appelle listener_callback
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Le topic auquel s'abonner
            self.listener_callback,
            10)  # Taille de la file d'attente
        self.subscription  # Empêche l'objet d'être détruit par le garbage collector

        self.get_logger().info('Subscriber node has been started.')
        # Position initiale
        self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}

    def listener_callback(self, msg):
        # Mettre à jour la position en fonction des commandes reçues
        self.position['x'] += msg.linear.x
        self.position['z'] += msg.linear.z
        self.position['ry'] += msg.angular.z

        # Log de la nouvelle position
        self.get_logger().info(f'New Position: x={self.position["x"]}, z={self.position["z"]}, ry={self.position["ry"]}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
