from controller import Robot, Receiver
import math
import numpy as np
import matplotlib.pyplot as plt
import json

class RobotController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.timestep)

        self.emitter = self.robot.getDevice("emitter")
        self.emitter.setChannel(100)

        #self.camera = self.robot.getDevice("camera")
        #self.camera.enable(self.timestep)
        
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.params = {
            'ROBOT_REPULSIVE_GAIN': 5.0,
            'ROBOT_REPELLING_RADIUS': 1,
            'BOX_REPULSIVE_GAIN': 2.0,
            'BOX_REPELLING_RADIUS': 0.5,
            'WALL_REPULSIVE_GAIN': 10.0,
            'WALL_REPELLING_RADIUS': 0.2,
            'GOAL_ATTRACTIVE_GAIN': 50.0,
            'GOAL_ATTRACTIVE_RADIUS': 0.4,
            'HAZARD_DETECTION_RADIUS': 0.3,
            'MAX_VELOCITY': 6.28,
            'MAX_FORCE': 10000000.0,
            'DAMPING_FACTOR': 0.1,
            'RANDOM_PERTURBATION': 0.0,
            'WHEEL_DISTANCE': 0.053,
            'WHEEL_RADIUS': 0.0205
        }

        self.WALLS = [
            ([-1, -1, 0], [1, -1, 0]),
            ([1, -1, 0], [1, 1, 0]),
            ([1, 1, 0], [-1, 1, 0]),
            ([-1, 1, 0], [-1, -1, 0])
        ]
    
    def repulsive_potential_from_robots(self, robots, current_position, radius):
        force_x, force_y = 0.0, 0.0
        for pos in robots:
            dx = current_position[0] - pos[0]
            dy = current_position[1] - pos[1]
            distance = math.hypot(dx, dy)
            if distance < radius:
                force_magnitude = self.params['ROBOT_REPULSIVE_GAIN'] * ((1 / distance) - (1 / radius))
                force_x += force_magnitude * (dx / distance)
                force_y += force_magnitude * (dy / distance)
        return force_x, force_y

    def repulsive_potential_from_boxes(self, boxes, current_position, radius):
        force_x, force_y = 0.0, 0.0
        for pos in boxes:
            dx = current_position[0] - pos[0]
            dy = current_position[1] - pos[1]
            distance = math.hypot(dx, dy)
            if distance < radius:
                force_magnitude = self.params['BOX_REPULSIVE_GAIN'] * ((1 / distance) - (1 / radius))
                force_x += force_magnitude * (dx / distance)
                force_y += force_magnitude * (dy / distance)
        return force_x, force_y

    def wall_repulsive_potential(self, walls, current_position, radius):
        force_x, force_y = 0.0, 0.0
        for wall in walls:
            (x1, y1, _), (x2, y2, _) = wall
            wx = x2 - x1
            wy = y2 - y1
            wx1 = current_position[0] - x1
            wy1 = current_position[1] - y1
            projection = (wx1 * wx + wy1 * wy) / (wx * wx + wy * wy)
            projection = max(0, min(1, projection))
            closest_x = x1 + projection * wx
            closest_y = y1 + projection * wy
            dx = current_position[0] - closest_x
            dy = current_position[1] - closest_y
            distance = math.hypot(dx, dy)
            if distance < radius:
                force_magnitude = self.params['WALL_REPULSIVE_GAIN'] * ((1 / distance) - (1 / radius))
                force_x += force_magnitude * (dx / distance)
                force_y += force_magnitude * (dy / distance)
        return force_x, force_y

    def attractive_potential_from_goals(self, goals, current_position, radius):
        force_x, force_y = 0.0, 0.0
        for goal in goals:
            dx = goal[0] - current_position[0]
            dy = goal[1] - current_position[1]
            distance = math.hypot(dx, dy)
            if distance < radius:
                force_magnitude = self.params['GOAL_ATTRACTIVE_GAIN'] * distance
                force_x += force_magnitude * (dx / distance)
                force_y += force_magnitude * (dy / distance)
        return force_x, force_y

    def compute_velocities_from_forces(self, force_x, force_y, robot_rotation):
        # Parameters
        wheel_distance = self.params['WHEEL_DISTANCE']
        max_velocity = self.params['MAX_VELOCITY']
        
        # Convert the robot rotation from radians
        theta = robot_rotation
        
        # Define the rotation matrix for converting global forces to local frame
        R = np.array([
                [math.cos(theta), -math.sin(theta)],
                [math.sin(theta),  math.cos(theta)]
            ])
        invR = np.linalg.inv(R)    
        # Global force vector
        global_force = np.array([force_x, force_y])
        
        # Transform global forces to local frame using the rotation matrix
        local_force = invR @ global_force
        
        # Extract local forces
        force_local_x, force_local_y = local_force
        
        # Compute linear and angular velocities
        v = force_local_x
        omega = force_local_y / wheel_distance
        
        # Compute left and right wheel velocities
        left_velocity = v - (omega * wheel_distance / 2)
        right_velocity = v + (omega * wheel_distance / 2)
        
        # Clamp the velocities
        left_velocity = np.clip(left_velocity, -max_velocity, max_velocity)
        right_velocity = np.clip(right_velocity, -max_velocity, max_velocity)
        
        return left_velocity, right_velocity

    def process_message(self, message):
        # Load JSON message
        data = json.loads(message)
        
        # Parse positions and rotations
        box_positions = [(box['x'], box['y'], box['z']) for box in data.get("Boxes", [])]
        hazard_positions = [(box['x'], box['y']) for box in data.get("Hazards", [])]
        robot_positions = [(robot['x'], robot['y'], robot['z']) for robot in data.get("Robots", [])]
        goal_positions = [(goal['x'], goal['y']) for goal in data.get("Goals", [])]  # Extract goals from message
        current_position = (
            data["Current"]["position"]["x"], 
            data["Current"]["position"]["y"], 
            data["Current"]["position"]["z"]
        )
        current_rotation = (
            data["Current"]["rotation"]["z"] * data["Current"]["rotation"]["w"]
        )
        
        return box_positions, robot_positions, goal_positions, hazard_positions, current_position, current_rotation

    def update_motors(self, left_velocity, right_velocity):
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def plot_apf_2d(self, repulsive_positions, box_positions, wall_positions, current_position, goal_positions):
        x = np.linspace(-1.5, 1.5, 20)
        y = np.linspace(-1.5, 1.5, 20)
        X, Y = np.meshgrid(x, y)
        U_repulsive = np.zeros(X.shape)
        V_repulsive = np.zeros(Y.shape)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                pos = (X[i, j], Y[i, j], 0)  # z is 0 for 2D plot
                fx_repulsive_robots, fy_repulsive_robots = self.repulsive_potential_from_robots(repulsive_positions, pos, self.params['ROBOT_REPELLING_RADIUS'])
                fx_repulsive_boxes, fy_repulsive_boxes = self.repulsive_potential_from_boxes(box_positions, pos, self.params['BOX_REPELLING_RADIUS'])
                fx_wall, fy_wall = self.wall_repulsive_potential(self.WALLS, pos, controller.params['WALL_REPELLING_RADIUS'])
                fx_attractive, fy_attractive = self.attractive_potential_from_goals(goal_positions, pos, controller.params['GOAL_ATTRACTIVE_RADIUS'])
                U_repulsive[i, j] = fx_repulsive_robots + fx_repulsive_boxes + fx_wall + fx_attractive
                V_repulsive[i, j] = fy_repulsive_robots + fy_repulsive_boxes + fy_wall + fy_attractive

        plt.figure(figsize=(10, 8))
        plt.quiver(X, Y, U_repulsive, V_repulsive, color='r', alpha=0.5)
        
        if box_positions:
            box_x, box_y = zip(*[(pos[0], pos[1]) for pos in box_positions])
            plt.scatter(box_x, box_y, c='blue', marker='o', label='Box Positions')

        for wall in self.WALLS:
            (x1, y1, _), (x2, y2, _) = wall
            plt.plot([x1, x2], [y1, y2], 'k-', lw=2, label='Wall')

        if current_position:
            plt.scatter(current_position[0], current_position[1], c='black', marker='o', label='Current Position')

        if goal_positions:
            goal_x, goal_y = zip(*[(pos[0], pos[1]) for pos in goal_positions])
            plt.scatter(goal_x, goal_y, c='green', marker='x', label='Goal Positions')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Artificial Potential Field - 2D')
        plt.grid(True)
        plt.show()
    
    def is_near_hazard(self, current_position, hazard_positions, radius):
        for hx, hy in hazard_positions:
            distance = math.hypot(hx - current_position[0], hy - current_position[1])
            if distance < radius:
                return True
        return False
    
    def emit_warning(self, robot_id, current_position):
        warning_message = {
            "type": "warning",
            "content": "Near hazard",
            "robot_id": robot_id,
            "position": {"x": current_position[0], "y": current_position[1], "z": current_position[2]}
        }
        json_message = json.dumps(warning_message)
        self.emitter.send(json_message.encode('utf-8'))
        print(f"Emitted warning: {json_message}")


if __name__ == "__main__":
    controller = RobotController()

    while controller.robot.step(controller.timestep) != -1:
        if controller.receiver.getQueueLength() > 0:
            message = controller.receiver.getString()
            controller.receiver.nextPacket()
            
            box_positions, robot_positions, goal_positions, hazard_positions, current_position, current_rotation = controller.process_message(message)

            repulsive_x_robots, repulsive_y_robots = controller.repulsive_potential_from_robots(robot_positions, current_position, controller.params['ROBOT_REPELLING_RADIUS'])
            repulsive_x_boxes, repulsive_y_boxes = controller.repulsive_potential_from_boxes(box_positions, current_position, controller.params['BOX_REPELLING_RADIUS'])
            wall_repulsive_x, wall_repulsive_y = controller.wall_repulsive_potential(controller.WALLS, current_position, controller.params['WALL_REPELLING_RADIUS'])
            attractive_x_goals, attractive_y_goals = controller.attractive_potential_from_goals(goal_positions, current_position, controller.params['GOAL_ATTRACTIVE_RADIUS'])

            total_force_x = repulsive_x_robots + repulsive_x_boxes + wall_repulsive_x + attractive_x_goals
            total_force_y = repulsive_y_robots + repulsive_y_boxes + wall_repulsive_y + attractive_y_goals

            force_magnitude = math.hypot(total_force_x, total_force_y)
            if force_magnitude > controller.params['MAX_FORCE']:
                total_force_x *= controller.params['MAX_FORCE'] / force_magnitude
                total_force_y *= controller.params['MAX_FORCE'] / force_magnitude

            distance_to_obstacle = min(math.hypot(repulsive_x_robots, repulsive_y_robots), controller.params['ROBOT_REPELLING_RADIUS'])
            damping = 1 - (controller.params['DAMPING_FACTOR'] * (1 - distance_to_obstacle / controller.params['ROBOT_REPELLING_RADIUS']))
            total_force_x *= damping
            total_force_y *= damping

            total_force_x += controller.params['RANDOM_PERTURBATION'] * (2 * np.random.rand() - 1)
            total_force_y += controller.params['RANDOM_PERTURBATION'] * (2 * np.random.rand() - 1)

            left_velocity, right_velocity = controller.compute_velocities_from_forces(total_force_x, total_force_y, current_rotation)
            controller.update_motors(left_velocity, right_velocity)

            if controller.is_near_hazard(current_position, hazard_positions, controller.params['HAZARD_DETECTION_RADIUS']):
                robot_id = controller.receiver.getChannel()  # Get the robot's ID
                controller.emit_warning(robot_id, current_position)

            # Uncomment the line below to visualize the potential field
            # controller.plot_apf_2d(robot_positions, box_positions, controller.WALLS, current_position, goal_positions)
