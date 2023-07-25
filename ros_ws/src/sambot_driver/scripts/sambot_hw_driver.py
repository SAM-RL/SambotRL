#!/usr/bin/env python3

import rospy
from base_controller.base_controller import SamBotBaseController

if __name__ == '__main__':
    rospy.init_node('sambot_base_controller')
    base_controller = SamBotBaseController()
    while not rospy.is_shutdown():
        command_dt = rospy.Time().now() - base_controller.lastUpdateTime
        if command_dt.to_sec() >= rospy.Duration(1.0 / base_controller.publishRate).to_sec():
            base_controller.read()
            base_controller.write()
            base_controller.lastUpdateTime = rospy.Time.now()
