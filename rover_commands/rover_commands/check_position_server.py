import rclpy
from rclpy.node import Node
from custom_msgs.srv import CheckPosition

class CheckPositionServer(Node):

    def __init__(self):
        super().__init__('check_position_server')
        
        # Création du serveur de service de type CheckPosition
        self.srv = self.create_service(CheckPosition, 'check_position', self.check_position_callback)
        
        self.get_logger().info('Service server has been started.')

    def check_position_callback(self, request, response):
        # Log les valeurs de la requête reçue
        self.get_logger().info(f'Received request: x={request.x}, z={request.z}, ry={request.ry}')

        # Définir les limites et vérifier si les coordonnées sont dans les limites
        x_min, x_max = -10.0, 10.0
        z_min, z_max = -10.0, 10.0
        ry_min, ry_max = -3.14, 3.14

        is_within_x = x_min <= request.x <= x_max
        is_within_z = z_min <= request.z <= z_max
        is_within_ry = ry_min <= request.ry <= ry_max

        # Déterminer si la position est autorisée
        response.is_allowed = is_within_x and is_within_z and is_within_ry

        # Bonus: Fournir une suggestion
        suggestion = []
        if not is_within_x:
            suggestion.append("Déplacez-vous vers la droite" if request.x < x_min else "Déplacez-vous vers la gauche")
        if not is_within_z:
            suggestion.append("Avancez" if request.z < z_min else "Reculez")
        if not is_within_ry:
            suggestion.append("Augmentez l'angle" if request.ry < ry_min else "Diminuez l'angle")

        response.suggestion = " | ".join(suggestion) if suggestion else "Position correcte"

        # Log pour vérifier le format de la réponse
        self.get_logger().info(f"Response format: is_allowed (type: {type(response.is_allowed)}), "
                            f"suggestion (type: {type(response.suggestion)})")
        
        # Log final de la réponse
        self.get_logger().info(f'Sending response: is_allowed={response.is_allowed}, suggestion="{response.suggestion}"')

        return response



def main(args=None):
    rclpy.init(args=args)
    node = CheckPositionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
