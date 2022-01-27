import cv2
import cv2.aruco
import numpy as np
import rospy
import tf2_ros
import ros_numpy
import geometry_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge
import message_filters


class ArucoBoardDetector:
    """
    Connects to camera input topic and tries to detect an aruco board based on cv2.aruco package.
    Specify the aruco board using the parameters; DICT type is currently hard coded to DICT_4x4_100.
    The recognised marker frame is published as parent of camera frame to tf.
    Image with marker frames is output for debug purposes.
    """
    def __init__(self):
        rospy.init_node('aruco_board_detector', anonymous=True)

        # read params
        self.input_image = rospy.get_param('~input_image', default='/kinect2/qhd/image_color')
        self.camera_info = rospy.get_param('~camera_info', default='/kinect2/qhd/camera_info')
        self.marker_image = rospy.get_param('~marker_image', default='marker_image')
        self.camera_frame = rospy.get_param('~camera_frame', default='kinect2_link')
        self.marker_frame = rospy.get_param('~marker_frame', default='marker_frame')
        self.marker_count_x = rospy.get_param('~marker_count_x', default=3)
        self.marker_count_y = rospy.get_param('~marker_count_y', default=2)
        self.marker_size_mm = rospy.get_param('~marker_size_mm', default=57)
        self.marker_spacing_mm = rospy.get_param('~marker_spacing_mm', default=19)
        self.aruco_dict_type = cv2.aruco.DICT_4X4_100  # default
        self._cv_bridge = CvBridge()

        self.aruco_board = cv2.aruco.GridBoard_create(
            self.marker_count_x, self.marker_count_y, self.marker_size_mm, self.marker_spacing_mm, self.aruco_dict)
        self.parameters = cv2.aruco.DetectorParameters_create()  # default params

        print('initialising ArucoBoardDetector with following parameters:')
        for key, val in vars(self).items():
            print(f'\t{key}: {val}')

        # transform broadcaster and image publisher
        self.pub_tf = tf2_ros.TransformBroadcaster()
        self.pub_image = rospy.Publisher(self.marker_image, sensor_msgs.msg.Image, queue_size=10)

        # subscribe to both camera image and camera info topics
        image_sub = message_filters.Subscriber(self.input_image, sensor_msgs.msg.Image)
        info_sub = message_filters.Subscriber(self.camera_info, sensor_msgs.msg.CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.detect_scene)
        rospy.spin()

    @property
    def aruco_dict(self):
        return cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)

    def detect_scene(self, image_msg, camera_info_msg):
        # marker recognition based on this tutorial:
        # https://docs.opencv.org/3.4/db/da9/tutorial_aruco_board_detection.html

        # parse camera info
        camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
        distortion_coeffs = np.array(camera_info_msg.D)

        # get image
        frame = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected_image_points = \
            cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters, cameraMatrix=camera_matrix,
                                    distCoeff=distortion_coeffs)

        if ids is None or len(ids) < self.marker_count_x * self.marker_count_y:  # or required if ids is None
            print(f'found markers: {ids} -- some are missing??')
        # only if any markers have been found
        if len(corners) > 0:
            # try to refine the detection - since we have grid board, we know where to expect markers
            corners, ids, rejected_image_points, recovered_indices = \
                cv2.aruco.refineDetectedMarkers(gray, self.aruco_board, corners, ids, rejected_image_points,
                                                camera_matrix, distortion_coeffs)

            # estimate pose of the board using all detected markers
            num_markers, rvec, tvec = \
                cv2.aruco.estimatePoseBoard(corners, ids, self.aruco_board, camera_matrix, distortion_coeffs,
                                            rvec=None, tvec=None)
            self.publish_pose(rvec, tvec)

            # drawing markers
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            frame = cv2.aruco.drawAxis(frame, camera_matrix, distortion_coeffs, rvec, tvec, 20)

        # publish image (even if no markers found)
        if not rospy.is_shutdown():
            self.pub_image.publish(self._cv_bridge.cv2_to_imgmsg(frame))

    def publish_pose(self, rvec, tvec):
        # rvec tvec are pose of markers wrt camera
        pose = np.eye(4)
        pose[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        pose[0:3, 3] = tvec.flatten() / 1000  # marker size was given in mm

        # pose is actual the inverse, as we use marker as parent and camera as child
        # otherwise some node in tf tree would have multiple parents
        pose = np.linalg.inv(pose)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.marker_frame
        t.child_frame_id = self.camera_frame
        t.header.stamp = rospy.Time.now()
        t.transform = ros_numpy.msgify(geometry_msgs.msg.Transform, pose)
        self.pub_tf.sendTransform(t)


if __name__ == '__main__':
    try:
        ArucoBoardDetector()
    except rospy.ROSInterruptException:
        print('ArucoBoardDetector got interrupted...')

