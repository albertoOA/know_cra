#!/usr/bin/env python3
# coding=utf-8
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>, King's College London
import rospy
import sys
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService, GetDomainOperatorDetailsService, \
    GetInstanceService, KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import roslib
from rosprolog_client import PrologException, Prolog


class ROSPlanExecutor:
    def __init__(self):
        # Define service clients
        self._get_goals = rospy.ServiceProxy("/rosplan_knowledge_base/state/goals", GetAttributeService)
        self._get_operator_details = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operator_details", GetDomainOperatorDetailsService)
        self._update_kb_srv = rospy.ServiceProxy("/rosplan_knowledge_base/update", KnowledgeUpdateService)
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        self._trigger_plan_srv = rospy.Service("~planning_pipeline", Empty, self.planning_pipeline)
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._plan_received = False
        # TODO dispatch?

        get_operators = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operators", GetDomainOperatorService)
        get_operators.wait_for_service()
        self.operators = [x.name for x in get_operators().operators]

        get_instances = rospy.ServiceProxy("/rosplan_knowledge_base/state/instances", GetInstanceService)
        get_instances.wait_for_service()
        self.garments = get_instances('garment', False, False).instances



    def raw_plan_cb(self, msg):
        self._plan_received = True
        print(msg.data)

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


if __name__ == "__main__":
    rospy.init_node("coherent_planning_node", sys.argv)
    rp = ROSPlanExecutor()
    rospy.spin()

