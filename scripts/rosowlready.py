#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from std_srvs.srv import Empty, EmptyResponse
from know_cra.srv import OwlreadyLoadOntology, OwlreadyLoadOntologyResponse
from know_cra.srv import OwlreadyInsertDeleteQuery, OwlreadyInsertDeleteQueryResponse
from know_cra.srv import OwlreadySelectQuery, OwlreadySelectQueryResponse


from know_cra.owlready_wrapper import OWLreadyWrapper
from know_cra.json_module import JSONmodule


class ROSowlready:
    def __init__(self):
        rospy.init_node("ros_owlready_node", sys.argv)
        rospy.loginfo(rospy.get_name() + ": ROS owlready node has been initialized.")

        # services
        self.load_ontology_ = rospy.Service("~load_ontology", OwlreadyLoadOntology, self.load_ontology)
        self.syncronize_reasoner_ = rospy.Service("~syncronize_reasoner", Empty, self.syncronize_reasoner)
        self.query_ontology_insert_delete = rospy.Service("~query_ontology_insert_delete", OwlreadyInsertDeleteQuery, self.query_ontology_insert_delete)
        self.query_ontology_select = rospy.Service("~query_ontology_select", OwlreadySelectQuery, self.query_ontology_select)

        # variables
        self.ontology_is_loaded_ = False
        self.orwp_ = OWLreadyWrapper()
        self.json_mod_ = JSONmodule()


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

    def query_ontology_insert_delete(self, req):
        rospy.loginfo(rospy.get_name() + ": Received request to update the ontology (using SPARQL).")

        if self.ontology_is_loaded_:
            try:
                rospy.loginfo(rospy.get_name() + ": Updating the ontology.")
                self.orwp_.sparql_insert_delete_query_wp(req.query_str)
            except rospy.ServiceException as e:
                rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        else:
            rospy.loginfo(rospy.get_name() + ": The query has not been performed because none ontology has been loaded.")

        return OwlreadyInsertDeleteQueryResponse()

    def query_ontology_select(self, req):
        rospy.loginfo(rospy.get_name() + ": Received request to query the ontology (using SPARQL).")

        if self.ontology_is_loaded_:
            try:
                rospy.loginfo(rospy.get_name() + ": Querying the ontology.")
                answer_dict = self.orwp_.sparql_select_query_wp(req.query_str)
                answer_json_str = self.json_mod_.dictionary_to_json_string(answer_dict)
            except rospy.ServiceException as e:
                rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        else:
            rospy.loginfo(rospy.get_name() + ": The query has not been performed because none ontology has been loaded.")

        return OwlreadySelectQueryResponse(answer_json_str)
        

if __name__ == "__main__":
    ror = ROSowlready()
    rospy.spin()

