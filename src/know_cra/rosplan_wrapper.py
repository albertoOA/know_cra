#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares-Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial
import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import CompletePlan
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService, GetDomainOperatorDetailsService, \
    GetDomainTypeService, GetInstanceService, KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import roslib

class ROSPlanWrapper:
    def __init__(self):
        # Define service clients
        self._get_goals = rospy.ServiceProxy("/rosplan_knowledge_base/state/goals", GetAttributeService)
        self._get_operator_details = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operator_details", GetDomainOperatorDetailsService)
        self._get_operators = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operators", GetDomainOperatorService)
        self._get_types = rospy.ServiceProxy("/rosplan_knowledge_base/domain/types", GetDomainTypeService)
        self._get_instances = rospy.ServiceProxy("/rosplan_knowledge_base/state/instances", GetInstanceService)
        self._get_propositions = rospy.ServiceProxy("/rosplan_knowledge_base/state/propositions", GetAttributeService)
        self._update_kb_srv = rospy.ServiceProxy("/rosplan_knowledge_base/update", KnowledgeUpdateService)
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        
        # Define topic subcriptions
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._parsed_plan_subs = rospy.Subscriber("/rosplan_parsing_interface/complete_plan", CompletePlan, self.parsed_plan_cb)

        # Define varibles
        self.plan_is_received_ = False
        self.domain_types_with_instances_dict_ = dict()
        self.problem_subgoals_dict_ = dict()

        # Getting domain operators
        self._get_operators.wait_for_service()
        self.operators_ = [x.name for x in self._get_operators().operators]

        # Getting instances of garment type
        self._get_instances.wait_for_service()
        self.garments_ = self._get_instances('garment', False, False).instances

        # Getting propositions of current state
        self._get_propositions.wait_for_service()
        self.propositions_attributes_ = self._get_propositions().attributes
        ## print(self.propositions_attributes_)

        """ TO PLAY WITH IT WITHOUT CALLING THE METHODS FROM OUTSIDE
        # Generating problem and planning
        self.planning_pipeline()

        #print(self.generated_plan_string_)
        """


    def raw_plan_cb(self, msg): # it is called when a new plan is published
        rospy.loginfo(rospy.get_name() + ": Received raw plan")
        self.plan_is_received_ = True
        print(msg.data)
        self.generated_plan_string_ = msg.data
    
    def parsed_plan_cb(self, msg): # it is called when a new parsed plan is published
        rospy.loginfo(rospy.get_name() + ": Received parsed plan")
        #print(msg.plan)
        self.generated_plan_parsed_ = msg.plan

    def planning_pipeline(self): 
        # TODO UPDATE PROBLEM
        rospy.loginfo(rospy.get_name() + ": Generating problem and planning")
        self._problem_gen.wait_for_service()
        self._problem_gen()
        self._planner.wait_for_service()
        self._planner()
        self._parse_plan.wait_for_service()
        self._parse_plan()

    def construct_types_and_instances_dict(self): 
        rospy.loginfo(rospy.get_name() + ": Getting the domain types and their instances to assert them to the ontology KB")
        self._get_types.wait_for_service()
        self.domain_types_ans_ = self._get_types()
        for t in self.domain_types_ans_.types:
            self.domain_types_with_instances_dict_[t] = self._get_instances(t, False, False).instances # (booleans) include_constants: include_subtypes:

    def construct_subgoals_dict(self):
        rospy.loginfo(rospy.get_name() + ": Getting the problem goal to assert it to the ontology KB")
        self._get_goals.wait_for_service()
        self.problem_goal_ans_ = self._get_goals()
        cont = 0
        for a in self.problem_goal_ans_.attributes:
            if len(a.values) == 1: # 'object quality' goal component
                goal_component_tuple = [a.attribute_name, a.values[0].value]
            elif len(a.values) == 2: # 'object relationship' goal component
                goal_component_tuple = [a.attribute_name, a.values[0].value, a.values[1].value]
            else:
                rospy.logerr(rospy.get_name() + ": Part of the goal has an unexpected format")
                goal_component_tuple = []
            
            self.problem_subgoals_dict_['goal_component_'+str(cont)] = goal_component_tuple
            cont += 1

"""
if __name__ == "__main__":
    rospy.init_node("coherent_planning_node", sys.argv)
    rp = ROSPlanWrapper()
    rospy.spin()
"""