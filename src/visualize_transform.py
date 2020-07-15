#!/usr/bin/env python
'''
Uses dynamic reconfigure to allow the user to edit a transformation and see the
result in rviz.
'''

import rospy
import tf

from dynamic_reconfigure.server import Server
from ros_tools.cfg import VisualizeTransformConfig

class VisualizeTransform():
    def __init__(self):
        rospy.init_node('visualize_transform', anonymous=True)
        self.frequency = 30
        self.rate = rospy.Rate(self.frequency)

        self.fixed_frame = rospy.get_param('~fixed_frame', 'world')
        self.moving_frame = rospy.get_param('~moving_frame', 'frame')

        self.translate_x = 0.0
        self.translate_y = 0.0
        self.translate_z = 0.0
        self.euler_x = 0.0
        self.euler_y = 0.0
        self.euler_z = 0.0

        self.parameter_server = Server(VisualizeTransformConfig, self.paramCallback)

        self.tfBroadcaster = tf.TransformBroadcaster()

    def spin(self):
        while not rospy.is_shutdown():
            quaternion = tf.transformations.quaternion_from_euler(self.euler_x, self.euler_y, self.euler_z, 'rxyz')
            self.tfBroadcaster.sendTransform([self.translate_x, self.translate_y, self.translate_z], quaternion, rospy.Time.now(), self.moving_frame, self.fixed_frame)
            self.rate.sleep()

    def paramCallback(self, config, level):
        self.translate_x = config.x
        self.translate_y = config.y
        self.translate_z = config.z
        self.euler_x = config.euler_x
        self.euler_y = config.euler_y
        self.euler_z = config.euler_z

        return config

if __name__ == '__main__':
    try:
        visualize_transform = VisualizeTransform()
        visualize_transform.spin()
    except rospy.ROSInterruptException:
        pass
