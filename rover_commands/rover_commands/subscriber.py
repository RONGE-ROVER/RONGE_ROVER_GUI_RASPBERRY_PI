import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.srv import CheckPosition

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        
        # Création d'un Subscriber de type Twist qui appelle listener_callback
        self.subscription_ = self.create_subscription(
            Twist,
            'topic',  # Assurez-vous que le nom du topic correspond à celui du Publisher
            self.listener_callback,
            10
        )

        # Création d'un client de service de type CheckPosition
        self.client = self.create_client(CheckPosition, 'check_position')
        
        # Attendre que le service soit disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Subscriber node has been started.')
        
        # Initialisation de la position du robot
        self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}

    def send_request(self, x, z, ry):
        # Préparer la requête pour le service CheckPosition
        request = CheckPosition.Request()
        request.x = x
        request.z = z
        request.ry = ry

        self.get_logger().info('Envoi de la requête au service.')
        
        # Envoyer la requête au service de manière asynchrone
        self.future = self.client.call_async(request)
        
        # Ajouter un callback qui sera appelé lorsque le service répondra
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            # Obtenir la réponse du Future
            response = future.result()
            if response is None:
                self.get_logger().error("Erreur : le serveur a renvoyé une réponse vide.")
            else:
                self.get_logger().info(f"Réponse du serveur : is_allowed={response.is_allowed}, suggestion='{response.suggestion}'")
                
                # Mise à jour de la position réelle si la réponse est autorisée
                if response.is_allowed:
                    self.position = self.temp_position  # Mettre à jour la position réelle
                    self.get_logger().info(f'New Position: {self.position}')
                else:
                    self.get_logger().info(f'Suggestion: {response.suggestion}')
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'appel au service : {e}")

    def listener_callback(self, msg):
        # Mettre à jour la position temporaire en fonction du message reçu
        self.temp_position = {
            'x': self.position['x'] + msg.linear.x,
            'z': self.position['z'] + msg.linear.y,
            'ry': self.position['ry'] + msg.angular.z
        }

        # Appeler le service pour vérifier si la position temporaire est autorisée
        self.send_request(self.temp_position['x'], self.temp_position['z'], self.temp_position['ry'])

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
