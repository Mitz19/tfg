from controller import Supervisor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class WebotsROS2Node(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('webots_ros2_node')

        print("Initializing Robot...")
        self.robot = Supervisor()
        print("Robot initialized successfully.")
        self.timestep = int(self.robot.getBasicTimeStep())

        # Inicializar motores
        self.left_motor = self.robot.getDevice("wheel_left_joint")
        self.right_motor = self.robot.getDevice("wheel_right_joint")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        print("Motors initialized successfully.")

        # Inicializar LIDAR
        self.lidar = self.robot.getDevice("Hokuyo URG-04LX-UG01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        print("LIDAR initialized successfully.")

        # Inicializar cámara de profundidad Astra
        self.depth_camera = self.robot.getDevice("Astra depth")
        self.depth_camera.enable(self.timestep)
        print("Depth camera initialized successfully.")

        # Lógica de batería
        self.battery_level = 100
        self.step_count = 0
        self.battery_drain_interval = 50

        # Variables
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.current_command = None
        self.obstacle_detected = False
        self.avoidance_state = 0
        self.stopped_by_human = False

        # Variables para giro de 180 grados
        self.turning_180 = False
        self.turn_steps_remaining = 0

        # Variables para esquivar objetos
        self.avoiding_object = False
        self.avoidance_start_time = None
        self.avoidance_duration = 3.0  # Duración total de la maniobra en segundos

        # Variables para mensajes de estado
        self.last_message = None  # Almacena el último mensaje mostrado

        # Suscripción a /commands
        self.create_subscription(String, '/commands', self.command_callback, 10)
        print("Subscribed to /commands.")

    def reduce_battery(self):
        self.step_count += 1
        if self.step_count >= self.battery_drain_interval:
            self.battery_level = max(0, self.battery_level - 1)
            self.step_count = 0
            print(f"Battery level: {self.battery_level}%")

            if 0 < self.battery_level <= 10:
                print("Battery critically low! Stopping motors.")
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)

            if self.battery_level == 0:
                print("Battery depleted. Shutting down the robot.")
                rclpy.shutdown()
                exit(0)

    def detect_obstacle_with_lidar(self, min_distance=0.3, max_distance=1.5):
        depth_data = self.lidar.getRangeImage()
        for i, distance in enumerate(depth_data):
            if min_distance < distance < max_distance:
                return True, i, distance
        return False, None, None

    def identify_object_by_def(self, sensor_index, distance):
        robot_node = self.robot.getSelf()
        robot_position = robot_node.getPosition()
        lidar_fov = self.lidar.getFov()
        lidar_resolution = len(self.lidar.getRangeImage())

        # Calcular ángulo del LIDAR
        lidar_angle = ((sensor_index + 0.5) / lidar_resolution) * lidar_fov - (lidar_fov / 2)

        # Calcular posición aproximada del objeto detectado
        object_position = [
            robot_position[0] + distance * math.cos(lidar_angle),
            robot_position[1],
            robot_position[2] + distance * math.sin(lidar_angle)
        ]
        for def_id in range(1, 31):
            def_name = f"_{def_id}"
            node = self.robot.getFromDef(def_name)
            if node:
                node_position = node.getPosition()
                distance_to_node = math.sqrt(
                    sum((object_position[i] - node_position[i])**2 for i in range(3))
                )
                if distance_to_node < 1.5:  # Ajustar el rango permitido
                    if 1 <= def_id <= 10:
                        return "human"
                    elif 11 <= def_id <= 20:
                        return "object"
                    elif 21 <= def_id <= 30:
                        return "wall"
        return "unknown"


    def avoid_obstacle(self):
        print("Avoiding obstacle: Turning right.")
        self.linear_velocity = 0.0
        self.angular_velocity = 1.0
        self.control_logic()

    def turn_around(self):
        self.print_message_once("Wall detected. Starting 180-degree turn.")
        self.linear_velocity = 0.0
        self.angular_velocity = 1.0  # Velocidad angular en rad/s

        # Calcular los pasos necesarios para un giro de 180 grados
        angular_velocity = 1.0  # rad/s
        turn_duration_seconds = math.pi / angular_velocity  # Tiempo necesario para girar 180 grados
        self.turn_steps_remaining = int(turn_duration_seconds / (self.timestep / 1000))  # Pasos necesarios

        self.turning_180 = True

    def avoid_object(self):
        self.print_message_once("Object detected. Starting simple avoidance maneuver.")
        self.avoiding_object = True
        self.avoidance_start_time = self.robot.getTime()  # Guardar tiempo inicial de la maniobra


    def command_callback(self, msg):
        command = msg.data.lower()
        if command != self.current_command:
            self.current_command = command
            print(f"Processing command: {command}")

            self.obstacle_detected = False
            self.stopped_by_human = False

            if command == "avanza":
                self.linear_velocity = 5.0
                self.angular_velocity = 0.0
            elif command == "para":
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
            elif command == "gira_izquierda":
                self.linear_velocity = 0.0
                self.angular_velocity = 0.5
            elif command == "gira_derecha":
                self.linear_velocity = 0.0
                self.angular_velocity = -0.5
            else:
                print("Unknown command received. Ignoring.")
            self.control_logic()

    def print_message_once(self, message):
        if self.last_message != message:
            print(message)
            self.last_message = message

    def control_logic(self):
        if self.battery_level > 10:
            if self.turning_180:
            # Durante el giro de 180 grados
                if self.turn_steps_remaining > 0:
                    self.print_message_once("Turning 180 degrees.")
                    self.left_motor.setVelocity(-1.0)
                    self.right_motor.setVelocity(1.0)
                    self.turn_steps_remaining -= 1
                else:
                    # Finaliza el giro de 180 grados
                    self.print_message_once("Completed 180-degree turn.")
                    self.turning_180 = False
                    self.angular_velocity = 0.0
                    self.linear_velocity = 5.0  # Reanuda el movimiento hacia adelante

            elif self.avoiding_object:
                # Tiempo transcurrido desde el inicio de la maniobra
                elapsed_time = self.robot.getTime() - self.avoidance_start_time

                if elapsed_time < 1.0:
                    # Paso 1: Girar hacia la derecha (1 segundo)
                    self.print_message_once("Avoidance: Turning right.")
                    self.left_motor.setVelocity(1.0)
                    self.right_motor.setVelocity(-1.0)
                elif elapsed_time < 2.0:
                    # Paso 2: Avanzar hacia adelante (1 segundo)
                    self.print_message_once("Avoidance: Moving forward.")
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(2.0)
                elif elapsed_time < 3.0:
                    # Paso 3: Girar hacia la izquierda (1 segundo)
                    self.print_message_once("Avoidance: Turning left.")
                    self.left_motor.setVelocity(-1.0)
                    self.right_motor.setVelocity(1.0)
                else:
                    # Fin de la maniobra
                    self.print_message_once("Avoidance maneuver completed.")
                    self.avoiding_object = False
                    self.obstacle_detected = False  # Reactiva la detección
                    self.left_motor.setVelocity(0.0)
                    self.right_motor.setVelocity(0.0)
            else:
                wheel_distance = 0.5
                left_speed = ((self.linear_velocity - self.angular_velocity) * wheel_distance) / 2
                right_speed = ((self.linear_velocity + self.angular_velocity) * wheel_distance) / 2
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
        elif self.obstacle_detected:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)

    def step(self):
        rclpy.spin_once(self, timeout_sec=0.01)

        if self.robot.step(self.timestep) == -1:
            rclpy.shutdown()
            return False

        self.reduce_battery()

        # Manejar el giro de 180 grados
        if self.turning_180:
            if self.turn_steps_remaining > 0:
                self.turn_steps_remaining -= 1
                self.control_logic()  # Actualiza las velocidades de las ruedas
            else:
                # Termina el giro
                self.print_message_once("Completed 180-degree turn.")
                self.turning_180 = False
                self.angular_velocity = 0.0
                self.linear_velocity = 5.0  # Configura velocidad hacia adelante
                self.obstacle_detected = False  # Reactiva la detección
                self.control_logic()
            return True

        # Manejar la evasión de objetos
        if self.avoiding_object:
            self.control_logic()
            return True

        detected, sensor_index, distance = self.detect_obstacle_with_lidar()
        if detected:
            obj_type = self.identify_object_by_def(sensor_index, distance)
            if obj_type == "human":
                if not self.stopped_by_human:
                    print("Human detected. Stopping.")
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self.stopped_by_human = True
                self.avoiding_object = False  # Cancela cualquier maniobra
                self.control_logic()
            elif obj_type == "wall":
                self.turn_around()  # Inicia el giro de 180 grados
            elif obj_type == "object" and not self.avoiding_object:
                self.avoid_object()  # Inicia la maniobra de evasión
        else:
            self.control_logic()

        return True


def main():
    print("Starting controller...")
    node = WebotsROS2Node()
    print("Controller initialized successfully.")
    while node.step():
        pass


if __name__ == "__main__":
    main()