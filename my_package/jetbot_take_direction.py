import math

class TakeDirection:
    def __init__(self, node):
        self.node = node
    
    def take_direction(self, robot_coordinates, cell_coordinates):

        dx =  robot_coordinates.x - cell_coordinates[0] 
        dy =  robot_coordinates.y - cell_coordinates[1] 

        angle = math.atan2(dy, dx) - math.pi

        angle = math.degrees(angle)

        self.node.get_logger().info(f'Kat azymutu {angle}')   
        
        q = self.node.my_orientation

        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

        yaw = math.degrees(yaw)

        self.node.get_logger().info(f'Kat robota {yaw}')

        angle = (angle + 180 - yaw) % 360

        if angle < 0:
            angle += 360

        self.node.get_logger().info(f'Kat obrotu {angle}')

        return angle