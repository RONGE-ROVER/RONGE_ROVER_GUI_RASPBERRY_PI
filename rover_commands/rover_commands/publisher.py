import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Crée un Publisher de type Twist, sur le topic 'topic'
        self.publisher_ = self.create_publisher(Twist, 'topic', 10)
        
        self.get_logger().info('Publisher node has been started.')

        # Crée un timer pour appeler cmd_acquisition périodiquement
        self.timer = self.create_timer(0.1, self.cmd_acquisition)

    # Fonction qui demande une commande à l'utilisateur et envoie le message correspondant
    def cmd_acquisition(self):
        command = input("Enter command (z/q/s/d/t/y - max 2 characters): ")
        
        # Création du message Twist
        msg = Twist()
        
        # Traitement des commandes de direction
        if command == 'z':
            msg.linear.x = 1.0  # Avance
        elif command == 's':
            msg.linear.x = -1.0  # Recule
        elif command == 'q':
            msg.linear.y = 1.0  # Gauche (si applicable)
        elif command == 'd':
            msg.linear.y = -1.0  # Droite (si applicable)
        elif command == 't':
            msg.angular.z = 1.0  # Rotation horaire
        elif command == 'y':
            msg.angular.z = -1.0  # Rotation antihoraire
        else:
            self.get_logger().info("Invalid command. Please use z, q, s, d, t, or y.")
            return  # Ignore la commande invalide
        
        # Publication du message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published message: {msg}")

def main(args=None):
    rclpy.init(args=args)   # Initialisation de ROS 2
    node = TrajectoryPublisher()  # Création d'une instance de Node
    rclpy.spin(node)  # Exécution du nœud en continu
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
