#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from std_srvs.srv import Empty, EmptyResponse
from know_cra.srv import OwlreadyLoadOntology, OwlreadyLoadOntologyResponse

from know_cra.owlready_wrapper import OWLreadyWrapper


class ROSowlreadyExecutor:
    def __init__(self):
        rospy.init_node("ros_owlready_node", sys.argv)
        rospy.loginfo(rospy.get_name() + ": ROS owlready node has been initialized.")

        # services
        self.load_ontology_ = rospy.Service("~loading_ontology", OwlreadyLoadOntology, self.load_ontology)
        self.load_ontology_ = rospy.Service("~syncronize_reasoner", Empty, self.syncronize_reasoner)

        # variables
        self.ontology_is_loaded_ = False
        self.orwp_ = OWLreadyWrapper()


    def load_ontology(self, req):
        rospy.loginfo(rospy.get_name() + ": Received request to load an ontology.")

        if not self.ontology_is_loaded_:
            try:
                rospy.loginfo(rospy.get_name() + ": Loading an ontology")
                self.orwp_.get_ontology_wp(req.ont_str)
                self.ontology_is_loaded_ = True
            except rospy.ServiceException as e:
                rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        else:
            rospy.loginfo(rospy.get_name() + ": None ontology has been loaded, because there is already a loaded ontology.")

        return OwlreadyLoadOntologyResponse()

    def syncronize_reasoner(self, req):
        rospy.loginfo(rospy.get_name() + ": Received request to syncronize the reasoner.")

        if self.ontology_is_loaded_:
            try:
                rospy.loginfo(rospy.get_name() + ": Syncronizing the reasoner.")
                self.orwp_.sync_reasoner_wp()
            except rospy.ServiceException as e:
                rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        else:
            rospy.loginfo(rospy.get_name() + ": The reasoner has not been syncronized because none ontology has been loaded.")

        return EmptyResponse()


if __name__ == "__main__":
    ror = ROSowlreadyExecutor()
    rospy.spin()

