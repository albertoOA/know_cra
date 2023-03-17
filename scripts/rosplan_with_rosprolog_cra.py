#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from std_srvs.srv import Empty, EmptyResponse

from know_cra.rosplan_wrapper import ROSPlanWrapper

class ROSPlanCRA:
    def __init__(self):
        rospy.init_node("rosplan_for_cra_node", sys.argv)
        rospy.loginfo(rospy.get_name() + ": ROSPlan + rosprolog node has been initialized.")

        # services
        

        # variables
        self.plan_is_generated_ = False
        self.rpwp_ = ROSPlanWrapper()
        

if __name__ == "__main__":
    rpcra = ROSPlanCRA()

    rpcra.rpwp_.planning_pipeline("{}") # generate and parse planning

    rospy.spin()

