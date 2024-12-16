#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import cos, sin, radians
import actionlib
import smach
import time
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class ApproachNearestObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.p_ang = 0.03
        self.p_vel = 0.8
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.stop = False
        
    def execute(self, userdata):
        rospy.loginfo("Approaching the nearest object...")
        rate = rospy.Rate(10)
        while True:
            if self.stop:
                rospy.loginfo("Finished approaching the nearest object.")
                time.sleep(2)
                return 'succeeded'
            scan = rospy.wait_for_message('/scan', LaserScan, timeout=5.0)
            self.process_scan(scan)


    def process_scan(self, scan):
        ranges = scan.ranges
        curvatures = []
        for i in range(len(ranges) - 1):
            if ranges[i] == 0 or ranges[i+1] == 0 or ranges[i+1] >= 0.8:
                curvatures.append(0)
            else:
                curvatures.append(abs(ranges[i] - ranges[i+1]))
        max_curvature_index = curvatures.index(max(curvatures)) + 1
        # ranges = scan.ranges
        # curvatures = []
        # for i in range(len(ranges) - 1):
        #     if ranges[i] == 0 or ranges[i+1] == 0:
        #         curvatures.append(0)
        #     else:
        #         curvatures.append(abs(ranges[i] - ranges[i+1]))

        # top_5_curvatures = sorted(range(len(curvatures)), key=lambda i: curvatures[i], reverse=True)[:5]
        # top_5_indices = []
        # top_5_distances = []

        # for index in top_5_curvatures:
        #     if index < len(ranges) - 1:  # 确保索引有效
        #         top_5_indices.append(index + 1)  # 加1是因为索引从1开始
        #         top_5_distances.append(ranges[index + 1])  # 选择下一个距离，因为曲率是计算当前和下一个之间的差异
        # max_curvature_index = min(range(len(top_5_distances)), key=lambda i: top_5_distances[i]) + 1 if top_5_distances else None
        dist = ranges[max_curvature_index]
        if (max_curvature_index >= 180):
            angle = max_curvature_index - 360
        else:
            angle = max_curvature_index
        print("dist: ", dist)
        print("angle: ", angle)
        diff = angle
        message = Twist()
        message.angular.z = self.p_ang * diff
        vel = self.p_vel * (dist - 0.20)
        non_zero_elements = [x for x in scan.ranges if x != 0]
        if vel > 0.5:
            vel = 0.5
        if (min(non_zero_elements) <= 0.22):
            print(min(non_zero_elements))
            message.angular.z = 0
            vel = 0.0
            print("stop")
            self.stop = True
        message.linear.x = vel
        self.vel_pub.publish(message)

        
class MoveToPoseState(smach.State):
    def __init__(self, X, Y):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.x = X
        self.y = Y
        # self.controller = HuskyHighlevelController()
    def execute(self, userdata):
        result = move_to_poseStamped(self.x, self.y)
        if result:
            rospy.loginfo("Pausing for 2 seconds after reaching the pose.")
            # time.sleep(2)
            return 'succeeded'
        else:
            return 'failed'
def move_to_poseStamped(X, Y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = X
    goal.target_pose.pose.position.y = Y
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def create_smach_state_machine():
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    p1_x = 1.559999942779541
    p1_y = -0.4699997901916504

    p2_x = -1.2400002479553223
    p2_y = -1.910000205039978

    p3_x = -1.6800004243850708
    p3_y = -5.109999656677246

    p4_x = 2.159999370574951
    p4_y = -4.179999828338623

    with sm:
        # smach.StateMachine.add('MOVE_TO_P1', MoveToPoseState(p1_x,p1_y), 
        #                       transitions={'succeeded':'MOVE_TO_P2', 'failed':'aborted'})

        smach.StateMachine.add('MOVE_TO_P2', MoveToPoseState(p2_x,p2_y), 
                              transitions={'succeeded':'APPROACH_NEAREST_OBJECT_ONE', 'failed':'aborted'})

        smach.StateMachine.add('APPROACH_NEAREST_OBJECT_ONE', ApproachNearestObject(),
                              transitions={'succeeded':'MOVE_TO_P3', 'failed':'aborted'})
        
        smach.StateMachine.add('MOVE_TO_P3', MoveToPoseState(p3_x,p3_y), 
                              transitions={'succeeded':'APPROACH_NEAREST_OBJECT_TWO', 'failed':'aborted'})
        
        smach.StateMachine.add('APPROACH_NEAREST_OBJECT_TWO', ApproachNearestObject(),
                              transitions={'succeeded':'MOVE_TO_P4', 'failed':'aborted'})

        smach.StateMachine.add('MOVE_TO_P4', MoveToPoseState(p4_x,p4_y), 
                              transitions={'succeeded':'APPROACH_NEAREST_OBJECT_THREE', 'failed':'aborted'})
        
        smach.StateMachine.add('APPROACH_NEAREST_OBJECT_THREE', ApproachNearestObject(),
                              transitions={'succeeded':'BACK_TO_P1', 'failed':'aborted'})
        
        smach.StateMachine.add('BACK_TO_P1', MoveToPoseState(p1_x,p1_y), 
                              transitions={'succeeded':'succeeded', 'failed':'aborted'})

    return sm


if __name__ == '__main__':
    rospy.init_node('smach_state_machine_node')

    sm = create_smach_state_machine()

    outcome = sm.execute()

    if outcome == 'succeeded':
        rospy.loginfo("All states executed successfully")
    else:
        rospy.loginfo("State machine failed or was preempted")