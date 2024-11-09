import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float32

class ArduinoPublisher(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        # Initialisation de la classe Node
        super().__init__('publisher_recup_arduino')
        #aaa
        # Configuration du port série
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.port, self.baudrate)

        # Création du publisher
        self.pub = self.create_publisher(Float32, '/arduino_data', 10)

        # Attente pour établir la communication série
        time.sleep(2)

        # Initialisation du timer pour appeler la méthode de lecture périodiquement
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        # Lecture des données du port série
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()
            try:
                valeur_recue = float(data)
                self.get_logger().info(f"Valeur reçue de l'Arduino : {valeur_recue}")
                self.pub.publish(Float32(data=valeur_recue))
            except ValueError:
                self.get_logger().warn("Données non valides reçues.")

def main(args=None):
    rclpy.init(args=args)

    # Instanciation du noeud
    node = ArduinoPublisher()

    # Exécution du noeud ROS
    rclpy.spin(node)

    # Fermeture du noeud proprement
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
