#!/usr/bin/env python3

import rospy
from gitver import gitver
import os
import rospkg
from importlib_metadata import version


class version_node(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()

    def getAcomms(self):
        rospy.loginfo(
            "\nros_acomms version: %s"
            % gitver.getVer(os.path.dirname(os.path.abspath(__file__)))
        )
    
    def getLtCodecs(self):
        try:
            rospy.loginfo("ltcodecs version: %s" % (version("ltcodecs")))
        except:
            rospy.loginfo("ltcodecs version: ltcodecs not installed")

    def getPyAcomms(self):
        try:
            rospy.loginfo(
                "pyacomms version: %s" % gitver.getVer(self.rospack.get_path("pyacomms"))
            )
        except:
            rospy.loginfo("pyacomms version: pyacomms not installed")
    
    def getHelpfulTools(self):
        try:
            rospy.loginfo(
                "helpful_tools version: %s\n"
                % gitver.getVer(self.rospack.get_path("helpful_tools"))
            )
        except:
            rospy.loginfo("helpful_tools version: helpful_tools not installed")


if __name__ == "__main__":
    try:
        display_version_node = version_node()
        display_version_node.getAcomms()
        display_version_node.getLtCodecs()
        display_version_node.getPyAcomms()
        rospy.loginfo("version_node shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("version_node shutdown (interrupt)")
