#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from std_srvs.srv import EmptyResponse
from know_cra.srv import OwlreadyLoadOntology

from know_cra.owlready_wrapper import OWLreadyWrapper


class ROSowlreadyExecutor:
    def __init__(self):
        rospy.init_node("ros_owlready_node", sys.argv)
        rospy.loginfo("ROS owlready node has been initialized.")

        # services
        self.load_ontology_ = rospy.Service("~loading_ontology", OwlreadyLoadOntology, self.load_ontology)

        # variables
        self.ontology_is_loaded_ = False
        self.orwp_ = OWLreadyWrapper()


    def load_ontology(self, req):
        rospy.loginfo("Received request to load an ontology.")

        if not self.ontology_is_loaded_:
            self.orwp_.get_ontology_wp(req.ont_str)
            self.ontology_is_loaded_ = True
            rospy.loginfo("An ontology has been loaded.")
        else:
            rospy.loginfo("None ontology has been loaded, because there is already a loaded ontology.")

        return EmptyResponse()


if __name__ == "__main__":
    ror = ROSowlreadyExecutor()
    rospy.spin()

