from controller import Supervisor

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

# Create the Supervisor instance
supervisor = Supervisor()
root = supervisor.getRoot()
root_children_field = root.getField("children")
wooden_boxes = []
robots = []

for i in range(root_children_field.getCount()):
    node = root_children_field.getMFNode(i)
    find_all_wooden_boxes(node, wooden_boxes)
    find_all_robots(node, robots)

emitter = supervisor.getDevice("emitter")

# Get the time step of the current world
timestep = int(supervisor.getBasicTimeStep())

last_send_time = 0
send_interval = 1000  # 1 second

# Main loop
while supervisor.step(timestep) != -1:
    current_time = supervisor.getTime() * 1000

    # Check if it's time to send data
    if current_time - last_send_time >= send_interval:
        # Prepare data to send
        box_positions = []
        for wooden_box in wooden_boxes:
            translation_field = wooden_box.getField("translation")
            position = translation_field.getSFVec3f()
            box_positions.append(position)
        
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

        # Combine box positions and robot positions into a single message
        for channel in robot_channels:
            # Filter out the current robot's position
            filtered_positions = {ch: pos for ch, pos in robot_position_dict.items() if ch != channel}
    
            # Construct the message
            message = "Boxes:" + ";".join(f"{pos[0]},{pos[1]},{pos[2]}" for pos in box_positions)
            message += " | Robots:" + ";".join(f"{pos[0]},{pos[1]},{pos[2]}" for pos in filtered_positions.values())
    
            # Add the current robot's position and rotation
            current_pos = robot_position_dict[channel]
            current_rot = robot_rotation_dict[channel]
            message += f" | Current:{current_pos[0]},{current_pos[1]},{current_pos[2]},{current_rot[0]},{current_rot[1]},{current_rot[2]},{current_rot[3]}"
    
            # Set the emitter channel and send the message
            emitter.setChannel(channel)
            emitter.send(message.encode('utf-8'))

            print(f"Sent data to channel {channel}: {message}")

        last_send_time = current_time
