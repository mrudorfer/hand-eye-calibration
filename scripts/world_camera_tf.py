import numpy as np
import rospy
import ros_numpy
import tf2_ros
import geometry_msgs.msg


class WorldCameraTF:
    """
    This class provides the calibrated transform of the camera.
    The actual transform is hard-coded; it has to be measured according to your exact setup.
    """
    def __init__(self):
        rospy.init_node('world_camera_tf', anonymous=True)
        self.world_frame = rospy.get_param('~world_frame', default='world')
        self.camera_frame = rospy.get_param('~camera_frame', default='kinect2_link')

        self.pos = np.array([0.3186263, 0.33035292, 0.79509764])
        self.quat = np.array([0.21645533, -0.85911871, 0.45153532, 0.10572597])

        self.rate = rospy.Rate(10)

        print('initialising WorldCameraTF with following parameters:')
        for key, val in vars(self).items():
            print(f'\t{key}: {val}')

        tf_pub = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.world_frame
        t.transform.translation = ros_numpy.msgify(geometry_msgs.msg.Vector3, self.pos)
        t.transform.rotation = ros_numpy.msgify(geometry_msgs.msg.Quaternion, self.quat)
        while not rospy.is_shutdown():
            t.header.stamp = rospy.Time.now()
            tf_pub.sendTransform(t)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        WorldCameraTF()
    except rospy.ROSInterruptException:
        print('WorldCameraTF got interrupted...')

