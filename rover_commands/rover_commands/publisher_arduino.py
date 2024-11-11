import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses

class TrajectoryPublisher(Node):

    def __init__(self, stdscr):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_arduino', 10)
        self.get_logger().info('Publisher node has been started.')
        
        # Configuration de curses pour le mode non-bloquant
        self.stdscr = stdscr
        self.stdscr.nodelay(True)  # Ne pas bloquer en attente d'une touche
        self.stdscr.clear()

        # Timer pour vérifier les entrées de touches
        self.timer = self.create_timer(0.1, self.cmd_acquisition)

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
                msg.angular.z = 1.0  # Rotation horaire
            elif command == 'd':
                msg.angular.z = -1.0  # Rotation antihoraire
            else:
                self.get_logger().info("Invalid command. Please use z, q, s, d, t, or y.")
                return  # Ignore la commande invalide
            
            # Publication du message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published message: {msg}")

def main(args=None):
    rclpy.init(args=args)  # Initialiser ROS 2 pour Python
    # Initialiser curses pour capturer les entrées sans "Entrée"
    curses.wrapper(lambda stdscr: rclpy.spin(TrajectoryPublisher(stdscr)))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
