#!/usr/bin/env python3

# 导入必要的库
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf
from math import cos, sin, radians, pi

# 定义一个追踪物体的类
class ObjectFollower:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('object_follower_type2')

        # 订阅里程计和物体检测主题
        self.odom_sub = rospy.Subscriber('/gazebo/ground_truth/state', Odometry, self.odom_callback)
        self.obj_sub = rospy.Subscriber('/objects', Float32MultiArray, self.obj_callback)

        # 发布器，用于发布机器人的目标位置
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # 初始化变量，用于跟踪机器人状态和检测到的物体
        self.current_pose = None
        self.last_published_time = rospy.Time.now()
        self.object_detected_once = False
        self.is_rotating = False
        self.rotate_timer = None  # 用于控制旋转状态的计时器

    # 处理里程计消息的回调函数
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # 处理物体检测消息的回调函数
    def obj_callback(self, msg):
        current_time = rospy.Time.now()
        if len(msg.data) > 0:
            self.object_detected_once = True
            if self.is_rotating:
                # 如果正在旋转中检测到物体，立即中断旋转并发布目标位置
                if self.rotate_timer:
                    self.rotate_timer.shutdown()  # 中断当前的旋转计时器
                self.is_rotating = False  # 重置旋转状态
            # 无论是否旋转，只要检测到物体就发布新的目标位置
            if (current_time - self.last_published_time).to_sec() >= 2.0:
                self.publish_goal(current_time)
                self.last_published_time = current_time
        elif self.object_detected_once and (current_time - self.last_published_time).to_sec() >= 7.0 and not self.is_rotating:
            # 如果一段时间内没有检测到物体，并且不在旋转状态，则开始右旋
            self.rotate_right_degrees(current_time)

    # 根据当前位置和检测到的物体发布目标位置的函数
    def publish_goal(self, current_time):
        if self.current_pose:
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            # 计算并发布新的目标位置，位于当前位置前方
            goal_pose = PoseStamped()
            goal_pose.header.stamp = current_time
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = self.current_pose.position.x + 1.5 * cos(yaw)
            goal_pose.pose.position.y = self.current_pose.position.y + 1.5 * sin(yaw)
            goal_pose.pose.orientation = self.current_pose.orientation

            self.goal_pub.publish(goal_pose)

    # 将机器人向右旋转一定角度的函数
    def rotate_right_degrees(self, current_time):
        if self.current_pose and (current_time - self.last_published_time).to_sec() >= 4.0:
            self.is_rotating = True
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            new_yaw = (euler[2] - radians(75)) % (2 * pi)  # 计算旋转后的新偏航角

            new_quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], new_yaw)
            
            # 发布目标位置以旋转机器人
            goal_pose = PoseStamped()
            goal_pose.header.stamp = current_time
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position = self.current_pose.position
            goal_pose.pose.orientation.x = new_quaternion[0]
            goal_pose.pose.orientation.y = new_quaternion[1]
            goal_pose.pose.orientation.z = new_quaternion[2]
            goal_pose.pose.orientation.w = new_quaternion[3]

            self.goal_pub.publish(goal_pose)
            self.last_published_time = current_time

            # 设置一个计时器，在旋转完成后重置旋转状态
            self.rotate_timer = rospy.Timer(rospy.Duration(2), self.end_rotation, oneshot=True)

    # 计时器结束后重置旋转状态的函数
    def end_rotation(self, event):
        self.is_rotating = False
        self.rotate_timer = None  # 清理计时器引用

# 主函数，运行ROS节点
if __name__ == '__main__':
    try:
        ObjectFollower()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        pass
