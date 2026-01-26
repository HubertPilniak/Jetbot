import numpy as np

class Search:
    def __init__(self, node):
        self.node = node

    def search(self):        

        front_center_min = np.min(self.node.front_ranges[15:76]) # ranges[15:76] -> front scan from lidar; 61 degrees;

        self.node.twist.linear.x = 0.0
        self.node.twist.angular.z = 0.0

        if front_center_min > self.node.obstacle_distance_mid:
            self.node.twist.linear.x = 0.2

            self.avoid_side_collisions()

        elif front_center_min > self.node.obstacle_distance_min:
            self.node.twist.linear.x = 0.05

            self.avoid_front_side_collisions()

            self.avoid_side_collisions()
            
        else:
            self.node.cmd_vel_pub.publish(self.node.twist)

            self.node.start_changing_direction(None)

            return
        
        self.node.cmd_vel_pub.publish(self.node.twist)

    def avoid_side_collisions(self):
        left_min = np.min(self.node.left_ranges)
        right_min = np.min(self.node.right_ranges)

        # if something is close from left side
        if left_min < self.node.side_obstacle_distance_min:
            self.node.twist.linear.x = 0.0
            self.node.twist.angular.z -= 0.1  # turn a little bit right

        # if something is close from right side
        if right_min < self.node.side_obstacle_distance_min:
            self.node.twist.linear.x = 0.0
            self.node.twist.angular.z += 0.1  # turn a little bit left

    def avoid_front_side_collisions(self):  # if obstacle is not in front of robot, turn from it
        front_right_min = np.min(self.node.front_ranges[:15])
        front_left_min = np.min(self.node.front_ranges[76:])

        if front_left_min < self.node.obstacle_distance_min: 
            self.node.twist.angular.z = -0.1 # turn right

        elif front_right_min < self.node.obstacle_distance_min:

            self.node.twist.angular.z = 0.1 # turn left
    
    