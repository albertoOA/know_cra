#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares-Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial
import rospy
import sys
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import CompletePlan
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService, GetDomainOperatorDetailsService, \
    GetInstanceService, KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import roslib
from rosprolog_client import PrologException, Prolog


class ROSPlanWrapper:
    def __init__(self):
        # Define service clients
        self._get_goals = rospy.ServiceProxy("/rosplan_knowledge_base/state/goals", GetAttributeService)
        self._get_operator_details = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operator_details", GetDomainOperatorDetailsService)
        self._update_kb_srv = rospy.ServiceProxy("/rosplan_knowledge_base/update", KnowledgeUpdateService)
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        self.get_operators = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operators", GetDomainOperatorService)
        self.get_instances = rospy.ServiceProxy("/rosplan_knowledge_base/state/instances", GetInstanceService)
        # Define service servers
        self._trigger_plan_srv = rospy.Service("~planning_pipeline", Empty, self.planning_pipeline)
        # Define topic subcriptions
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._parsed_plan_subs = rospy.Subscriber("/rosplan_parsing_interface/complete_plan", CompletePlan, self.parsed_plan_cb)

        # Define varibles
        self.plan_is_received_ = False
        # TODO dispatch?

        # Getting domain operators
        self.get_operators.wait_for_service()
        self.operators = [x.name for x in self.get_operators().operators]

        # Getting instances of garment type
        self.get_instances.wait_for_service()
        self.garments = self.get_instances('garment', False, False).instances

        """ TO PLAY WITH IT WITHOUT CALLING THE SERVICES FROM OUTSIDE
        # Generating problem and planning
        self.planning_pipeline("{}")

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

    def planning_pipeline(self, req): 
        # TODO UPDATE PROBLEM
        try:
            rospy.loginfo(rospy.get_name() + ": Generating problem and planning")
            self._problem_gen.call()
            self._planner.call()
            self._parse_plan.call()
        except rospy.ServiceException as e:
            rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        return EmptyResponse()

"""
if __name__ == "__main__":
    rospy.init_node("coherent_planning_node", sys.argv)
    rp = ROSPlanWrapper()
    rospy.spin()
"""