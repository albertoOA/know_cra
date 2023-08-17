#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from know_cra.rosplan_wrapper import ROSPlanWrapper
from know_cra.rosprolog_wrapper_for_rosplan import ROSPrologWrapperForROSPlanCRA

class ROSPlanCRA:
    def __init__(self):
        # services
        

        # variables
        self.plan_is_generated_ = False
        self.rosplan_wrapper_ = ROSPlanWrapper()
        self.rosprolog_wrapper_for_rosplan_cra_ = ROSPrologWrapperForROSPlanCRA()

        if (rospy.has_param('~plan_adaptation_case')):
            self.plan_adaptation_case_ = rospy.get_param('~plan_adaptation_case')
        else:
            pass