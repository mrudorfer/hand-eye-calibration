import rospy
import numpy as np
import ros_numpy
import tf2_ros
import geometry_msgs.msg


class RobotMarkerTF:
    """
    This class provides a transform of a marker attached to the robot end effector to the tf tree.
    Used for hand-eye-calibration of robot.
    The actual transform is hard-coded; it has to be measured according to your exact setup.
    """
    def __init__(self):
        rospy.init_node('robot_marker_tf', anonymous=True)
        self.robot_frame = rospy.get_param('~robot_frame', default='panda_EE')
        self.marker_frame = rospy.get_param('~marker_frame', default='marker_frame')

        self.marker_in_robot = np.eye(4)
        self.marker_in_robot[0:3, 0] = [0, -1, 0]  # x axis
        self.marker_in_robot[0:3, 1] = [0, 0, -1]  # y axis
        self.marker_in_robot[0:3, 2] = [1, 0, 0]  # z axis
        self.marker_in_robot[0:3, 3] = [0.032, 0.098, 0.092]  # offset in robot frame

        print('initialising RobotMarkerTF with following parameters:')
        for key, val in vars(self).items():
            print(f'\t{key}: {val}')

        tf_pub = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_frame
        t.child_frame_id = self.marker_frame
        t.transform = ros_numpy.msgify(geometry_msgs.msg.Transform, self.marker_in_robot)
        tf_pub.sendTransform(t)
        rospy.spin()


if __name__ == '__main__':
    try:
        RobotMarkerTF()
    except rospy.ROSInterruptException:
        print('RobotMarkerTF got interrupted...')

