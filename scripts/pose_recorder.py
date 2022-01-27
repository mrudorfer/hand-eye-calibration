import rospy
import numpy as np
import ros_numpy
import tf2_ros


def average_quaternions(quaternions):
    # we can average quaternions as explained here: https://stackoverflow.com/a/49690919/1264582
    # assume input is list of quaternions
    q = np.array(quaternions)  # should have shape (n, 4)
    n = q.shape[0]
    w = np.ones(n) / n  # weights
    avg = np.linalg.eigh(np.einsum('ij,ik,i->...jk', q, q, w))[1][:, -1]
    return avg


def compute_average_pose(poses):
    # this assumes poses is (n, 7) with translations first then quaternions
    print(f'computing average over {len(poses)} poses')
    translations, quaternions = poses[:, :3], poses[:, 3:]
    avg_trans = translations.mean(axis=0)
    trans_errors = np.linalg.norm(translations - avg_trans, axis=-1)

    avg_quat = average_quaternions(quaternions)
    angular_errors = pairwise_angular_distances(avg_quat, quaternions)[0]

    return np.concatenate([avg_trans, avg_quat]), trans_errors, angular_errors


def pairwise_angular_distances(quaternions_1, quaternions_2):
    """
    Computes the pairwise radian distances between the provided quaternions from set1 and set2.
    If the sets are too large, consider processing in chunks.
    :param quaternions_1: (N, 4) ndarray with w x y z
    :param quaternions_2: (M, 4) ndarray with w x y z
    :return: (N, M) matrix of radian distances
    """
    # make sure correct shape (can be different if only one element in set)
    quaternions_1 = quaternions_1.reshape(-1, 4)
    quaternions_2 = quaternions_2.reshape(-1, 4)

    # using the minimum to prevent nans due to numerical precision (sometimes values are slightly larger than 1)
    angles = 2 * np.arccos(np.minimum(1, np.abs(quaternions_1.dot(np.transpose(quaternions_2, [1, 0])))))
    return angles


def print_pose(pose):
    print(f't: {pose[:3]}, q: {pose[3:]}')


class PoseRecorder:
    """
    This class allows to record the transformation from given source frame to given target frame guided by the user.
    Upon completion, the poses are averaged and the averaged result is printed.
    """
    def __init__(self):
        rospy.init_node('average_poses', anonymous=True)
        self.source_frame = rospy.get_param('~source_frame', default='world')
        self.intermediate_frame = rospy.get_param('~intermediate_frame', default='panda_EE')
        self.optimise_frame = rospy.get_param('~optimise_frame', default='marker_frame')
        self.target_frame = rospy.get_param('~target_frame', default='kinect2_link')
        self.filename = rospy.get_param('~filename', default='/home/rudorfem/ros/recorded_poses.npy')

        print('initialising PoseRecorder with following parameters:')
        for key, val in vars(self).items():
            print(f'\t{key}: {val}')

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        rate = rospy.Rate(10)

        # record poses
        print('***')
        print('you can now start recording poses')
        print('each time you hit Enter, I will record:')
        print(f'\t{self.source_frame} --> {self.intermediate_frame}')
        print(f'\t{self.optimise_frame} --> {self.target_frame}')
        print(f'\t{self.source_frame} --> {self.target_frame}')
        print('press p (followed by enter) to finish recording and process recorded poses')
        print('recording...')
        transforms = {
            'source2target': [],
            'source2intermediate': [],
            'optimise2target': []
        }
        recorded_poses = 0
        while not rospy.is_shutdown():
            key = input()
            if key in [' ', '']:
                if rospy.is_shutdown():
                    print('rospy shut down in the meantime. sorry mate.')
                    break
                else:
                    print('looking up transforms...')
                    try:
                        time = rospy.Time()
                        timeout = rospy.Duration(secs=2)
                        tf_st = buffer.lookup_transform(self.target_frame, self.source_frame, time, timeout)
                        tf_si = buffer.lookup_transform(self.intermediate_frame, self.source_frame, time, timeout)
                        tf_ot = buffer.lookup_transform(self.target_frame, self.optimise_frame, time, timeout)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        print('sorry mate, could not look up all the transforms:', e)
                        rate.sleep()
                        continue

                    # store recorded poses
                    def store_pose(pose_list, msg):
                        q = ros_numpy.numpify(msg.transform.rotation)
                        t = ros_numpy.numpify(msg.transform.translation)
                        pose_list.append(np.concatenate([t, q]))

                    store_pose(transforms['source2target'], tf_st)
                    store_pose(transforms['source2intermediate'], tf_si)
                    store_pose(transforms['optimise2target'], tf_ot)
                    recorded_poses += 1
                    print(f'recorded pose set #{recorded_poses}')

            elif key in ['p', 'P']:
                break
        if recorded_poses <= 6:
            print(f'please record many more than just {recorded_poses} poses...')
            print('rage quit!!11')
            return

        np.save(self.filename, transforms)
        print(f'saved recorded transforms to {self.filename}')

        print('processing poses...')
        # concatenating lists
        transforms['source2target'] = np.array(transforms['source2target'])
        transforms['source2intermediate'] = np.array(transforms['source2intermediate'])
        transforms['optimise2target'] = np.array(transforms['optimise2target'])

        # direct averaging
        print('directly averaging all source-->target transforms yields the following:')
        avg_pose_direct, t_err, q_err = compute_average_pose(transforms['source2target'])
        print_pose(avg_pose_direct)
        print(f'translational errors: avg {t_err.mean()}, std {t_err.std()}')
        q_err = np.rad2deg(q_err)
        print(f'angular errors (degrees): avg {q_err.mean()}, std {q_err.std()}')
        transforms['avg_pose_direct'] = {
            'pose': avg_pose_direct,
            't_err': t_err,
            'q_err_deg': q_err
        }

        np.save(self.filename, transforms)
        print(f'saved recorded and processed transforms to {self.filename}')


if __name__ == '__main__':
    try:
        PoseRecorder()
    except rospy.ROSInterruptException:
        print('PoseRecorder got interrupted...')

