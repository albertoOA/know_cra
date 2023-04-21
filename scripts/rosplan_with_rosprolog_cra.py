#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from std_srvs.srv import Empty, EmptyResponse

from know_cra.rosplan_wrapper import ROSPlanWrapper
from know_cra.rosprolog_wrapper_for_rosplan import ROSPrologWrapperForROSPlanCRA

class ROSPlanCRA:
    def __init__(self):
        rospy.init_node("rosplan_for_cra_node", sys.argv)
        rospy.loginfo(rospy.get_name() + ": ROSPlan + rosprolog node has been initialized.")

        # services
        

        # variables
        self.plan_is_generated_ = False
        self.rpwp_ = ROSPlanWrapper()
        self.rpwprp_ = ROSPrologWrapperForROSPlanCRA()

if __name__ == "__main__":
    rpcra = ROSPlanCRA()

    # get information about the domain and problem
    rpcra.rpwp_.construct_types_and_instances_dict()
    ## print(rpcra.rpwp_.domain_types_with_instances_dict_)
    types_and_instances_triples_list = rpcra.rpwprp_.types_and_instances_dict_to_triples_list(rpcra.rpwp_.domain_types_with_instances_dict_)
    ## print(types_and_instances_triples_list)
    types_and_instances_assertion_query_text = rpcra.rpwprp_.construct_query_text_for_multiple_triples_assertion(types_and_instances_triples_list, False)
    ## print(types_and_instances_assertion_query_text)
    rpcra.rpwprp_.rosprolog_assertion_query(types_and_instances_assertion_query_text)

    rpcra.rpwp_.construct_subgoals_dict()
    ## print(rpcra.rpwp_.problem_subgoals_dict_)

    subgoals_triples_list = rpcra.rpwprp_.subgoals_dict_to_triples_list(rpcra.rpwp_.problem_subgoals_dict_)
    ## print(subgoals_triples_list)

    subgoals_assertion_query_text = rpcra.rpwprp_.construct_query_text_for_multiple_triples_assertion(subgoals_triples_list, False)
    ## print(subgoals_assertion_query_text)
    rpcra.rpwprp_.rosprolog_assertion_query(subgoals_assertion_query_text)

    rpcra.rpwp_.planning_pipeline() # generate and parse planning

    ## rpcra.rpwp_.construct_domain_operators_details_dict() # unnecessary - use construct_plan_dict
    ## print(rpcra.rpwp_.domain_operators_details_dict_) 

    rpcra.rpwp_.construct_plan_dict()
    ## print(rpcra.rpwp_.plan_dict_)

    plan_triples_list = rpcra.rpwprp_.plan_dict_to_triples_list(rpcra.rpwp_.plan_dict_)
    print(plan_triples_list)




    rospy.spin()

