#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from map_interaction.msg import InspectHolonomicAction, InspectHolonomicFeedback, InspectHolonomicResult
import tf
import math

class InspectHolonomicActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('inspect_holonomic', InspectHolonomicAction, self.execute, False)
        self.server.start()
        self.scan_sub       = rospy.Subscriber('/scan',            LaserScan, self.scan_callback)
        self.scan_left_sub  = rospy.Subscriber('/scan_left_side',  LaserScan, self.scan_left_callback)
        self.scan_right_sub = rospy.Subscriber('/scan_right_side', LaserScan, self.scan_right_callback)
        self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.listener = tf.TransformListener()
        self.inspection_distance = rospy.get_param('/inspect_distance', 0.5)
        self.scan_data = None
        self.scan_left_data = None
        self.scan_right_data = None
        self.base_frame = 'base_link'
        self.odom_frame = 'odom'
        self.map_frame = 'map'
        self.obstacle_distance = 0.3

    def scan_callback(self, msg):
        self.scan_data = msg

    def scan_left_callback(self, msg):
        self.scan_left_data = msg

    def scan_right_callback(self, msg):
        self.scan_right_data = msg

    def execute(self, goal):
        target_pose = goal.target_pose
        if not self.orient_toward_object(target_pose):
            self.server.set_aborted()
            return
        print('stap 2')
        if not self.move_to_inspection_pose(target_pose):
            self.server.set_aborted()
            return
        print('stap 3')
        if self.inspect_around_object(target_pose):
            self.server.set_succeeded(InspectHolonomicResult(success=True))
        else:
            rospy.sleep(1)

    def orient_toward_object(self, target_pose):
        # Calculate required yaw to face the target_pose
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                yaw_error = self.calculate_yaw_error(target_pose)

                if abs(yaw_error) < 0.1:   # magic tolerance number, TODO: put in external config file
                    break

                twist = Twist()
                twist.angular.z = 1 if yaw_error > 0 else -1    #magic number: rotation speed. TODO: put in external config file
                self.cmd_pub.publish(twist)
                rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.cmd_pub.publish(Twist())
        return True

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calculate_distance_to_object(self, target_pose):
        (trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        dx = target_pose.pose.position.x - trans[0]
        dy = target_pose.pose.position.y - trans[1]        
        distance = (dx**2+dy**2)**(0.5)
        return distance

    def calculate_yaw_error(self, target_pose):
        (trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        dx = target_pose.pose.position.x - trans[0]
        dy = target_pose.pose.position.y - trans[1]       
        desired_yaw = math.atan2(dy, dx)
        _, _, current_yaw = tf.transformations.euler_from_quaternion(rot)
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)
        return yaw_error

    def move_to_inspection_pose(self, target_pose):
        print('moving to inspection pose')
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        inspection_pose = self.calculate_inspection_pose(target_pose)
        initial_goal = MoveBaseGoal()
        initial_goal.target_pose = inspection_pose

        client.send_goal(initial_goal)
        client.wait_for_result()
        print(client.get_state(), GoalStatus.SUCCEEDED)
        if client.get_state() != GoalStatus.SUCCEEDED:
            print('returning False')
            return False
        return True

    def calculate_inspection_pose(self, target_pose):
        # Calculate the pose at the inspection distance
        dx = self.inspection_distance * math.cos(target_pose.pose.orientation.z)
        dy = self.inspection_distance * math.sin(target_pose.pose.orientation.z)

        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = target_pose.header.frame_id
        inspection_pose.header.stamp = rospy.Time.now()
        inspection_pose.pose.position.x = target_pose.pose.position.x - dx
        inspection_pose.pose.position.y = target_pose.pose.position.y - dy
        inspection_pose.pose.position.z = target_pose.pose.position.z
        inspection_pose.pose.orientation = target_pose.pose.orientation

        return inspection_pose

    def inspect_around_object(self, target_pose):
        print('Inspecting around object')
        rate = rospy.Rate(10)
        angle_covered = 0
        done_inspecting = False
        touched_obstacle_right = False
        touched_obstacle_left = False
        (trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        _, _, previous_yaw = tf.transformations.euler_from_quaternion(rot)

        while not rospy.is_shutdown() and not done_inspecting:
            if self.server.is_preempt_requested():
                print('preempted while inspecting around object')
                self.server.set_preempted()
                self.cmd_pub.publish(Twist())
                return False

            # check for obstacles on side that we are moving toward
            if not touched_obstacle_left:  #we must still be moving to the left, apparently
                if not self.is_free_left(): 
                    touched_obstacle_left = True
            else:
                if not self.is_free_right():
                    touched_obstacle_right = True

            twist = Twist()
            k_x =   1 #2              #magic numbers: proportional gains
            k_yaw = 2

            # decide rotation direction, start clockwise, reverse when there is an obstacle
            if not touched_obstacle_left: twist.linear.y = 0.05  # TODO: magic number
            else: twist.linear.y = -0.05        # TODO: magic number

            max_x_vel = 0.1   # TODO: magic number
            max_yaw_vel = 0.3  # TODO: magic number
            twist.linear.x =   min(max_x_vel,  (max(-max_x_vel,   k_x   * (self.calculate_distance_to_object(target_pose) - self.inspection_distance))))
            twist.angular.z =  min(max_yaw_vel,(max(-max_yaw_vel, k_yaw * self.calculate_yaw_error(target_pose))))
            self.cmd_pub.publish(twist)

            # keep track of how far we have rotated.
            (trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))            
            _, _, current_yaw = tf.transformations.euler_from_quaternion(rot)
            angle_covered += self.normalize_angle(current_yaw - previous_yaw)
            previous_yaw = current_yaw
            
            print('angle_covered: ',angle_covered)
            if (touched_obstacle_left and touched_obstacle_right) or abs(angle_covered)>6.28: done_inspecting = True
            rate.sleep()
                 
        self.cmd_pub.publish(Twist())
        return True

    def is_free_right(self):
        if not self.scan_right_data:
            return True
        min_distance = self.scan_right_data.range_max
        for distance in self.scan_right_data.ranges:
            if distance > self.scan_right_data.range_min:
                min_distance = min(min_distance,distance)
                if distance < self.obstacle_distance:  
                    print('STOP right distance = ', distance)
                    return False
        print('right_distance = ',min_distance)
        return True

    def is_free_left(self):
        if not self.scan_left_data:
            return True
        min_distance = self.scan_left_data.range_max
        for distance in self.scan_left_data.ranges:
            if distance > self.scan_left_data.range_min:
                min_distance = min(min_distance,distance)
                if distance < self.obstacle_distance:  
                    print('STOP left distance = ', distance)
                    return False
        print('left_distance = ',min_distance)
        return True

if __name__ == '__main__':
    rospy.init_node('inspect_holonomic_action_server')
    server = InspectHolonomicActionServer()
    rospy.spin()



# #!/usr/bin/env python3

# import rospy
# import actionlib
# from geometry_msgs.msg import PoseStamped, Twist
# from sensor_msgs.msg import LaserScan
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from map_interaction.msg import InspectHolonomicAction, InspectHolonomicFeedback, InspectHolonomicResult
# import tf
# import math

# class InspectHolonomicActionServer:
#     def __init__(self):
#         self.server = actionlib.SimpleActionServer('inspect_holonomic', InspectHolonomicAction, self.execute, False)
#         self.server.start()
#         self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
#         self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
#         self.listener = tf.TransformListener()
#         self.inspection_distance = rospy.get_param('/inspect_distance', 0.5)
#         self.scan_data = None
#         self.base_frame = 'base_link'
#         self.odom_frame = 'odom'
#         self.map_frame = 'map'

#     def scan_callback(self, msg):
#         self.scan_data = msg

#     def execute(self, goal):
#         target_pose = goal.target_pose
#         success = self.orient_toward_object(target_pose)
#         if not success:
#             self.server.set_aborted()
#             return
        
#         success = self.move_to_inspection_pose(target_pose)
#         if not success:
#             self.server.set_aborted()
#             return

#         success = self.inspect_around_object(target_pose)
#         if success:
#             self.server.set_succeeded(InspectHolonomicResult(success=True))
#         else:
#             self.server.set_aborted()

#     def orient_toward_object(self, target_pose):
#         # Code to orient the robot towards the object
#         # Implement this using TF to get the current robot pose and calculate the required rotation
#         # Use cmd_vel to publish the necessary Twist messages to achieve the orientation
#         return True

#     def move_to_inspection_pose(self, target_pose):
#         # Calculate the required position and orientation to be at the inspection distance from the object
#         # Use move_base to move to this pose
#         client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         client.wait_for_server()

#         initial_goal = MoveBaseGoal()
#         initial_goal.target_pose.header.frame_id = "map"
#         initial_goal.target_pose.header.stamp = rospy.Time.now()
#         initial_goal.target_pose.pose = self.calculate_inspection_pose(target_pose)

#         client.send_goal(initial_goal)
#         wait = client.wait_for_result()

#         if not wait or client.get_state() != GoalStatus.SUCCEEDED:
#             return False
#         return True

#     def calculate_inspection_pose(self, target_pose):
#         # Calculate the pose based on the inspection distance
#         # Return a PoseStamped
#         return target_pose.pose

#     def inspect_around_object(self, target_pose):
#         # Rotate around the object while keeping oriented towards it
#         # Implement obstacle avoidance using /scan data
#         # Handle preemption
#         return True

# if __name__ == '__main__':
#     rospy.init_node('inspect_holonomic_action_server')
#     server = InspectHolonomicActionServer()
#     rospy.spin()
