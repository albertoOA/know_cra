#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import rospy

from know_cra.rosplan_cra_module import ROSPlanCRA

if __name__ == "__main__":
    rospy.init_node("rosplan_for_cra_node", sys.argv)
    rospy.loginfo(rospy.get_name() + ": ROSPlan + rosprolog node has been initialized.")
    
    rpcra = ROSPlanCRA()

    # get information about the domain and problem
    rpcra.rosplan_wrapper_.construct_types_and_instances_dict()
    ## print(rpcra.rosplan_wrapper_.domain_types_with_instances_dict_)
    types_and_instances_triples_list = rpcra.rosprolog_wrapper_for_rosplan_cra_.types_and_instances_dict_to_triples_list(rpcra.rosplan_wrapper_.domain_types_with_instances_dict_)
    ## print(types_and_instances_triples_list)
    types_and_instances_assertion_query_text = rpcra.rosprolog_wrapper_for_rosplan_cra_.construct_query_text_for_multiple_triples_assertion(types_and_instances_triples_list, True)
    ## print(types_and_instances_assertion_query_text)
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(types_and_instances_assertion_query_text)

    rpcra.rosplan_wrapper_.construct_subgoals_dict()
    ## print(rpcra.rosplan_wrapper_.problem_subgoals_dict_)

    subgoals_triples_list = rpcra.rosprolog_wrapper_for_rosplan_cra_.subgoals_dict_to_triples_list(rpcra.rosplan_wrapper_.problem_subgoals_dict_)
    ## print(subgoals_triples_list)

    subgoals_assertion_query_text = rpcra.rosprolog_wrapper_for_rosplan_cra_.construct_query_text_for_multiple_triples_assertion(subgoals_triples_list, True)
    ## print(subgoals_assertion_query_text)
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(subgoals_assertion_query_text)

    rpcra.rosplan_wrapper_.planning_pipeline() # generate and parse planning

    ## rpcra.rosplan_wrapper_.construct_domain_operators_details_dict() # unnecessary - use construct_plan_dict
    ## print(rpcra.rosplan_wrapper_.domain_operators_details_dict_) 

    rpcra.rosplan_wrapper_.construct_plan_dict()
    ## print(rpcra.rosplan_wrapper_.plan_dict_)

    plan_triples_list = rpcra.rosprolog_wrapper_for_rosplan_cra_.plan_dict_to_triples_list(rpcra.rosplan_wrapper_.plan_dict_)
    """ # for debugging
    for i in range(0, 25):
        print(plan_triples_list[i])
        print("\n")
    """
    ## print(plan_triples_list)

    plan_assertion_query_text = rpcra.rosprolog_wrapper_for_rosplan_cra_.construct_query_text_for_multiple_triples_assertion(plan_triples_list, True)
    ## print(plan_assertion_query_text)
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(plan_assertion_query_text)



    # update the ontology knowledge base to reflect that the initial plan is no longer valid
    query_string_foo_ = "kb_call(triple(Plan, ocra_common:'hasValidity', Q)), \
        kb_unproject(triple(Q, dul:'hasDataValue', _)), \
        kb_project(triple(Q, dul:'hasDataValue', 'False'))." # two ways to assert booleans: false/true or 'False'/'True'
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(query_string_foo_)

    # update the planning knowledge base with an unexpected state that triggers a plan adaptation
    if (rpcra.plan_adaptation_case_ == "unfolded_cloth"):
        rpcra.rosplan_wrapper_.add_or_remove_single_fact_planning_kb(2, 'folded', {'g':'towel-01'}) # remove
        rpcra.rosplan_wrapper_.add_or_remove_single_fact_planning_kb(0, 'unfolded', {'g':'towel-01'}) # add
    else: 
        rospy.logwarn(rospy.get_name() + ": The introduced value for 'plan_adaptation_case_' is unvalid.") 

    # generate the problem and do re-planning 
    rpcra.rosplan_wrapper_.planning_pipeline() # generate and parse planning

    # construct new plan dictionary and assert it to the ontology knowledge base
    rpcra.rosplan_wrapper_.construct_plan_dict()
    ## print(rpcra.rosplan_wrapper_.plan_dict_)
    plan_triples_list = rpcra.rosprolog_wrapper_for_rosplan_cra_.plan_dict_to_triples_list(rpcra.rosplan_wrapper_.plan_dict_)
    plan_assertion_query_text = rpcra.rosprolog_wrapper_for_rosplan_cra_.construct_query_text_for_multiple_triples_assertion(plan_triples_list, True)
    ## print(plan_assertion_query_text)
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(plan_assertion_query_text)

    # save the NEEM using in the name 'plan_adaptation_case_' and the current time
    query_string_foo_ = "ros_package_path('know_cra', P1), \
        atom_concat(P1, '/NEEMs/contrastive_plans/"+ rpcra.plan_adaptation_case_ +"_' , P2), \
        get_time(T), atom_concat(P2, T, P3), mng_dump(roslog, P3)."
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(query_string_foo_)

    rospy.spin()

