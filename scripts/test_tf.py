#!/usr/bin/env python
import rospy
import tf
import numpy as np


def test_tf():
    rospy.init_node('tf_test', anonymous=True)
    rate = rospy.Rate(100)
    init_time = rospy.get_rostime().to_sec()

    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        t = rospy.get_rostime().to_sec() - init_time
        a = (0.62832 * t) % (2 * 3.14159)  # 10s per revolution

        radius = 0.5
        x = np.cos(a) * radius
        y = np.sin(a) * radius

        broadcaster.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(-3.14/2, 0, a + 3.14), rospy.Time.now(),
                                  "laser1", "world")

        broadcaster.sendTransform((0, 0, radius), tf.transformations.quaternion_from_euler(0, 3.14/2, a), rospy.Time.now(),
                                  "laser2", "world")

        print('time: ' + str(t))
        print('angle: ' + str(a))
        rate.sleep()


if __name__ == '__main__':
    try:
        test_tf()
    except rospy.ROSInterruptException:
        pass
