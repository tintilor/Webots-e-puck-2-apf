from controller import Supervisor
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import json
import random

# Helper functions
def find_all_wooden_boxes(node, boxes):
    if node.getTypeName() == 'WoodenBox':
        boxes.append(node)
    elif node.getTypeName() in ['Group', 'Transform', 'Solid']:
        children_field = node.getField("children")
        if children_field:
            for i in range(children_field.getCount()):
                find_all_wooden_boxes(children_field.getMFNode(i), boxes)

def find_all_robots(node, robots):
    if node.getTypeName() == 'E-puck':
        robots.append(node)
    elif node.getTypeName() in ['Group', 'Transform']:
        children_field = node.getField("children")
        if children_field:
            for i in range(children_field.getCount()):
                find_all_robots(children_field.getMFNode(i), robots)

def distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

def point_to_line_distance(point, line_start, line_end):
    px, py = point[0], point[1]
    sx, sy = line_start[0], line_start[1]
    ex, ey = line_end[0], line_end[1]
    line_length = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2)
    if line_length == 0:
        return math.sqrt((px - sx) ** 2 + (py - sy) ** 2)
    return abs((ey - sy) * px - (ex - sx) * py + ex * sy - ey * sx) / line_length

class Goal:
    def __init__(self, x, y, goal_type, creation_timestep):
        self.x = x
        self.y = y
        self.goal_type = goal_type
        self.creation_timestep = creation_timestep  # Timestep when the goal was created
        self.last_detected_timestep = creation_timestep  # Timestep when the goal was last detected

    def update_last_detected(self, timestep):
        self.last_detected_timestep = timestep

    def has_timed_out(self, current_timestep, timeout):
        return (current_timestep - self.last_detected_timestep) > timeout

# Goal Manager class to handle goal placement and removal
class GoalManager:
    def __init__(self, supervisor, walls, goal_timeout=15, max_goals=15):
        self.supervisor = supervisor
        self.goals = []
        self.walls = walls
        self.goal_placement_threshold = 0.1  # Minimum distance from walls for goal placement
        self.goal_min_distance = 0.6 # Minimum distance between goals
        self.robot_min_distance = 0.2  # Minimum distance from robots for goal placement
        self.obstacle_min_distance = 0.1  # Minimum distance from obstacles for goal placement
        self.goal_timeout = goal_timeout  # Number of timesteps before a goal is removed
        self.max_goals = max_goals  # Maximum number of goals allowed at any time

    def is_near_wall(self, x, y):
        for wall_start, wall_end in self.walls:
            if point_to_line_distance((x, y, 0), wall_start, wall_end) < self.goal_placement_threshold:
                return True
        return False

    def is_near_goal(self, x, y):
        for goal in self.goals:
            if math.sqrt((goal.x - x) ** 2 + (goal.y - y) ** 2) <= self.goal_min_distance:
                return True
        return False

    def is_near_robot(self, x, y, robots):
        for robot in robots:
            rx, ry, _ = robot["position"]
            if math.sqrt((rx - x) ** 2 + (ry - y) ** 2) <= self.robot_min_distance:
                return True
        return False

    def is_near_obstacle(self, x, y, obstacles):
        for obstacle in obstacles:
            ox, oy, _ = obstacle
            if math.sqrt((ox - x) ** 2 + (oy - y) ** 2) <= self.obstacle_min_distance:
                return True
        return False

    def add_goal(self, x, y, goal_type, robots, obstacles):
        if len(self.goals) < self.max_goals:  # Only add a goal if under the max limit
            if (not self.is_near_wall(x, y) and not self.is_near_goal(x, y) 
                and not self.is_near_robot(x, y, robots) and not self.is_near_obstacle(x, y, obstacles)):
                
                creation_timestep = self.supervisor.getTime()  # Get the current timestep
                new_goal = Goal(x, y, goal_type, creation_timestep)
                self.goals.append(new_goal)

    def update_goals(self, robots, obstacles):
        current_time = self.supervisor.getTime()

        # Remove goals that are near robots, near walls, near obstacles, or have timed out
        self.goals = [goal for goal in self.goals 
                      if (goal.goal_type != 'explore' or (
                            not self.is_near_wall(goal.x, goal.y) and 
                            not self.is_near_robot(goal.x, goal.y, robots) and 
                            not self.is_near_obstacle(goal.x, goal.y, obstacles)))
                      and not goal.has_timed_out(current_time, self.goal_timeout)]

        # Update last detected time for goals near robots
        for goal in self.goals:
            if self.is_near_robot(goal.x, goal.y, robots):
                goal.update_last_detected(current_time)

        # Add new explore goals, but only if the number of goals is below the maximum
        for _ in range(5):  # Try adding 5 new explore goals per update
            if len(self.goals) < self.max_goals:  # Check again before each addition
                x = random.uniform(-1, 1)  # Adjust to the size of your arena
                y = random.uniform(-1, 1)  # Adjust to the size of your arena
                if (not self.is_near_wall(x, y) and not self.is_near_goal(x, y) 
                    and not self.is_near_robot(x, y, robots) and not self.is_near_obstacle(x, y, obstacles)):
                    self.add_goal(x, y, 'explore', robots, obstacles)

        print(f"Debug: Updated Goals - Total: {len(self.goals)}")

    def get_goals(self):
        # Return goal data in the expected format
        return [(goal.x, goal.y, goal.goal_type) for goal in self.goals]

# Create the Supervisor instance
supervisor = Supervisor()
root = supervisor.getRoot()
root_children_field = root.getField("children")
wooden_boxes = []
robots = []

# Initialize the environment
for i in range(root_children_field.getCount()):
    node = root_children_field.getMFNode(i)
    find_all_wooden_boxes(node, wooden_boxes)
    find_all_robots(node, robots)

emitter = supervisor.getDevice("emitter")

# Wall coordinates (update according to your arena)
WALLS = [
    ([-1, -1, 0], [1, -1, 0]),
    ([1, -1, 0], [1, 1, 0]),
    ([1, 1, 0], [-1, 1, 0]),
    ([-1, 1, 0], [-1, -1, 0])
]

# Initialize the Goal Manager
goal_manager = GoalManager(supervisor, WALLS)

# Function to plot the environment
def plot_environment(box_positions, robot_positions, goals, walls):
    plt.figure(figsize=(10, 10))
    
    # Plot walls
    for wall_start, wall_end in walls:
        wall_x = [wall_start[0], wall_end[0]]
        wall_y = [wall_start[1], wall_end[1]]
        plt.plot(wall_x, wall_y, 'k-', linewidth=2, label='Wall')
    
    # Plot boxes
    for box in box_positions:
        plt.plot(box[0], box[1], 's', markersize=10, color='brown', label='Box')
    
    # Plot robots
    for robot in robot_positions:
        plt.plot(robot[0], robot[1], 'o', markersize=8, color='blue', label='Robot')
    
    # Plot goals
    for goal in goals:
        x, y, goal_type = goal
        if goal_type == 'explore':
            plt.plot(x, y, 'x', markersize=10, color='red', label='Explore Goal')
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Environment Plot')
    plt.legend()
    plt.grid(True)
    plt.show(block=False)
    plt.pause(1)
    plt.close()

display = supervisor.getDevice("display")

def draw_on_display(display, box_positions, robot_positions, goals, walls):
    # Define colors using hex codes
    COLOR_BACKGROUND = 0x000000  # Black
    COLOR_WALL = 0xffffff  # White
    COLOR_BOX = 0x854e00  # Brown
    COLOR_ROBOT = 0x0000ff  # Blue
    COLOR_GOAL = 0xff0000  # Red
    COLOR_EXTRA = 0xff00fb  # Extra color

    # Set background color and clear display
    display.setColor(COLOR_BACKGROUND)
    display.fillRectangle(0, 0, display.getWidth(), display.getHeight())

    # Draw walls
    display.setColor(COLOR_WALL)
    for wall_start, wall_end in walls:
        x1 = int((wall_start[0] + 1) * display.getWidth() / 2)
        y1 = int((1 - wall_start[1]) * display.getHeight() / 2)
        x2 = int((wall_end[0] + 1) * display.getWidth() / 2)
        y2 = int((1 - wall_end[1]) * display.getHeight() / 2)
        display.drawLine(x1, y1, x2, y2)

    # Draw boxes
    display.setColor(COLOR_BOX)
    for box in box_positions:
        x = int((box[0] + 1) * display.getWidth() / 2)
        y = int((1 - box[1]) * display.getHeight() / 2)
        display.drawRectangle(x - 3, y - 3, 6, 6)

    # Draw robots
    display.setColor(COLOR_ROBOT)
    for robot in robot_positions:
        x = int((robot[0] + 1) * display.getWidth() / 2)
        y = int((1 - robot[1]) * display.getHeight() / 2)
        display.drawOval(x - 1, y - 1, 2, 2)  # Center the oval on the coordinate

    # Draw goals
    display.setColor(COLOR_GOAL)
    for goal in goals:
        x, y, goal_type = goal
        if goal_type == 'explore':
            x = int((x + 1) * display.getWidth() / 2)
            y = int((1 - y) * display.getHeight() / 2)
            display.drawOval(x - 1, y - 1, 1, 1)  # Center the oval on the coordinate

    # Draw an extra pixel for testing
    display.setColor(COLOR_EXTRA)
    display.drawPixel(0, 0)


# Main loop
timestep = int(supervisor.getBasicTimeStep())
last_send_time = 0
send_interval = 1000  # 1 second

while supervisor.step(timestep) != -1:
    current_time = supervisor.getTime() * 1000

    # Update goal manager with current robot positions
    robot_data = []
    for robot in robots:
        translation_field = robot.getField("translation")
        rotation_field = robot.getField("rotation")
        if translation_field and rotation_field:
            position = translation_field.getSFVec3f()
            rotation = rotation_field.getSFRotation()
            robot_data.append({"position": position, "rotation": rotation})

    

    # Collect data for plotting
    box_positions = [wooden_box.getField("translation").getSFVec3f() for wooden_box in wooden_boxes]
    robot_positions = [robot.getField("translation").getSFVec3f() for robot in robots]
    goal_manager.update_goals(robot_data,box_positions)
    goals = goal_manager.get_goals()
    #goals = [(0.5, 0.5, 'explore')]
    draw_on_display(display, box_positions, robot_positions, goals, WALLS)
    # Check if it's time to send data
    if current_time - last_send_time >= send_interval:
        # Plot the environment
        # plot_environment(box_positions, robot_positions, goals, WALLS)

        box_positions = [{"x": pos[0], "y": pos[1], "z": pos[2]} for pos in [wooden_box.getField("translation").getSFVec3f() for wooden_box in wooden_boxes]]
        
        robot_positions = []
        robot_rotations = []
        robot_channels = []
        for robot in robots:
            translation_field = robot.getField("translation")
            rotation_field = robot.getField("rotation")
            if translation_field and rotation_field:
                position = translation_field.getSFVec3f()
                rotation = rotation_field.getSFRotation()
                robot_positions.append(position)
                robot_rotations.append(rotation)
                
                receiver_channel_field = robot.getField("receiver_channel")
                if receiver_channel_field:
                    robot_channels.append(receiver_channel_field.getSFInt32())
        
        robot_position_dict = dict(zip(robot_channels, robot_positions))        
        robot_rotation_dict = dict(zip(robot_channels, robot_rotations))

        # Include goals in the message
        goals = goal_manager.get_goals()
        #goals = [(-0.5, -0.5, 'explore'),(0.5, 0.5, 'explore')]
        goal_data = [{"x": x, "y": y, "type": t} for x, y, t in goals]

        # Combine box positions, robot positions, and goals into a single message
        for channel in robot_channels:
            filtered_positions = {ch: pos for ch, pos in robot_position_dict.items() if ch != channel}
    
            message = {
                "Boxes": box_positions,
                "Robots": [{"x": pos[0], "y": pos[1], "z": pos[2]} for pos in filtered_positions.values()],
                "Goals": goal_data,
                "Current": {
                    "position": {
                        "x": robot_position_dict[channel][0],
                        "y": robot_position_dict[channel][1],
                        "z": robot_position_dict[channel][2]
                    },
                    "rotation": {
                        "x": robot_rotation_dict[channel][0],
                        "y": robot_rotation_dict[channel][1],
                        "z": robot_rotation_dict[channel][2],
                        "w": robot_rotation_dict[channel][3]
                    }
                }
            }
    
            # Convert the message to JSON string and send
            json_message = json.dumps(message)
            emitter.setChannel(channel)
            emitter.send(json_message.encode('utf-8'))

            print(f"Sent JSON data to channel {channel}: {json_message}")
            

        last_send_time = current_time
