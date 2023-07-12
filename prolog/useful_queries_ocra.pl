
------- Coverage and competency questions ---

· C1 - What is a collaboration? 
(next query returns several options, one is 'Event')
subclass_of('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#Collaboration', A). 
(next one returns true)
subclass_of('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#Collaboration', 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Event').

· C2 - What is a plan adaptation?
(next query returns several options, one is 'Event') 
subclass_of('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#PlanAdaptation', A). 
(next one returns true)
subclass_of('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#PlanAdaptation', 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Event').

· Q1 - Which and how many collaborations are running now?
instance_of(I, 'http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#Collaboration').
instance_of(I, ocra:'Collaboration').
kb_call(triple(I, rdf:type, ocra:'Collaboration')).

· Q2 - Which is the plan of a collaboration?
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_filling_a_tray.owl#CollaborativelyFillingATray', ocra:executesPlan, P)).
kb_call(triple(ocra_filling_a_tray:'CollaborativelyFillingATray', ocra:executesPlan, P)).
kb_call(holds(ocra_filling_a_tray:'CollaborativelyFillingATray', ocra:executesPlan, P)).

· Q3 - Which is the goal of a collaborative plan?
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_filling_a_tray.owl#CollaborativelyFillingATray', ocra:executesPlan, P)), kb_call(triple(P, dul:hasComponent, G)).
kb_call(triple(ocra_filling_a_tray:'CollaborativelyFillingATray', ocra:executesPlan, P)), kb_call(triple(P, dul:hasComponent, G)).

· Q4 - Are these agents collaborating?
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_filling_a_tray.owl#CollaborativelyFillingATray', dul:hasParticipant, A)).
kb_call(triple(ocra_filling_a_tray:'CollaborativelyFillingATray', dul:hasParticipant, A)).

· Q5 - Where is a collaboration happening?
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_filling_a_tray.owl#CollaborativelyFillingATray', dul:hasLocation, P)).
kb_call(triple(ocra_filling_a_tray:'CollaborativelyFillingATray', dul:hasLocation, P)).

· Q6 - How is a collaboration classified (e.g.non-physical)?
kb_call(triple(ocra_filling_a_tray:'CollaborativelyFillingATray', dul:isClassifiedBy, T)).

· Q7 - Which is the risk of a collaboration?
instance_of(R, ocra:'CollaborationRisk'), kb_call(triple(R, dul:isQualityOf, ocra_filling_a_tray:'CollaborativelyFillingATray')), kb_call(triple(R, dul:hasDataValue, RiskLevel)).

· Q8 - Which and how many plan adaptations are running now?
instance_of(I, ocra:'PlanAdaptation').

· Q9 - Which is(are) the agent(s) participating in the plan adaptation?
kb_call(triple(ocra_filling_a_tray:'Full_compartment_adaptation', dul:hasParticipant, A)).

· Q10 - Why is an adaptation of an agent plan happening?
instance_of(S, dul:'Situation'), kb_call(triple(S, dul:isPostconditionOf, E)), kb_call(triple(E, dul:isPartOf, ocra_filling_a_tray:'Full_compartment_adaptation')), kb_call(triple(E, ocra:executesPlan, P1)), kb_call(triple(P1, ocra:isWorsePlanThan, P2)).
instance_of(S, dul:'Situation'), kb_call(triple(S, dul:isPostconditionOf, E)), kb_call(triple(E, dul:isPartOf, ocra_filling_a_tray:'Full_compartment_adaptation')), kb_call(triple(E, ocra:executesPlan, P1)), kb_call(triple(P1, ocra:isWorsePlanThan, P2)), kb_call(triple(S, dul:isSettingFor, Bc)), kb_call(triple(Bc, dul:hasQuality, Q)), kb_call(triple(Q, dul:hasDataValue, V)).

instance_of(S, dul:'Situation'), kb_call(triple(S, dul:isPostconditionOf, E)), kb_call(triple(E, dul:isPartOf, ocra_filling_a_tray:'Full_compartment_adaptation')), kb_call(triple(E, ocra:executesPlan, P1)), kb_call(triple(P2, ocra:isBetterPlanThan, P1)).

· Q11 - Which is the plan before and after an adaptation?
instance_of(P1, dul:'Plan'), kb_call(triple(P1, ocra:isPlanExecutedIn, E1)), kb_call(triple(E1, dul:isPartOf, ocra_filling_a_tray:'Full_compartment_adaptation')), kb_call(triple(P1, ocra:isWorsePlanThan, P2)).
instance_of(P2, dul:'Plan'), kb_call(triple(P2, ocra:isPlanExecutedIn, E2)), kb_call(triple(E2, dul:isPartOf, ocra_filling_a_tray:'Full_compartment_adaptation')), kb_call(triple(P2, ocra:isBetterPlanThan, P1)).

· Q12 - Which is the goal to be achieved by both the old and the new plan? Which is the goal of the agent(s) involved in the plan adaptation?
instance_of(P2, dul:'Plan'), kb_call(triple(P2, ocra:isPlanExecutedIn, E2)), kb_call(triple(E2, dul:isPartOf, ocra_filling_a_tray:'Full_compartment_adaptation')), kb_call(triple(P2, ocra:isBetterPlanThan, P1)), kb_call(triple(P1, dul:hasComponent, G)), kb_call(triple(G, dul:isComponentOf, P2)).
kb_call(triple(ocra_filling_a_tray:'Full_compartment_adaptation', dul:hasParticipant, A)), kb_call(triple(A, ocra:'hasGoal', G)).

------- Coverage and competency questions (end) ---






------- Semantic map queries ---

· know the current available capacity of a compartment of the tray..
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/maps/filling_a_tray.owl#compartment_20', dul:hasQuality, Q4)), kb_call(has_type(Q4, ocra_filling_a_tray:'AvailableCapacity')), kb_call(triple(Q4, dul:hasDataValue, V)).

· update the current available capacity of a compartment of the tray..
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/maps/filling_a_tray.owl#compartment_20', dul:hasQuality, Q4)), kb_call(has_type(Q4, ocra_filling_a_tray:'AvailableCapacity')), kb_unproject(triple(Q4, dul:hasDataValue, _)), kb_project(triple(Q4, dul:hasDataValue, 0)).


· ask (one by one) for all the compartments (parts of the tray)..
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/maps/filling_a_tray.owl#work_tray', dul:hasPart, C)).

· ask for all the compartments (parts of the tray)..we get a list in Clist (we also get an empty C)
findall(C, kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/maps/filling_a_tray.owl#work_tray', dul:hasPart, C)), Clist).

· know the current cartesian velocity of a physical object..
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/maps/filling_a_tray.owl#robot_end_effector', dul:hasQuality, Q4)), kb_call(has_type(Q4, ocra_filling_a_tray:'CartesianVelocity')), kb_call(triple(Q4, dul:hasDataValue, V)).


· update the current cartesian velocity of a physical object..
kb_call(triple('http://www.iri.upc.edu/groups/perception/OCRA/maps/filling_a_tray.owl#robot_end_effector', dul:hasQuality, Q4)), kb_call(has_type(Q4, ocra_filling_a_tray:'CartesianVelocity')), kb_unproject(triple(Q4, dul:hasDataValue, _)), kb_project(triple(Q4, dul:hasDataValue, '0.3 0.0 0.0 0.0 0.0 0.0')).

kb_call(triple(map:'robot_end_effector', dul:hasQuality, Q4)), kb_call(has_type(Q4, ocra_filling_a_tray:'CartesianVelocity')), kb_call(triple(Q4, dul:hasDataValue, V)).

------- Semantic map queries (end) ---






------- Queries with time interval ---

· instances of collaboration within an interval (In prolog, by default, all the true triples without interval are ALWAYS true)
kb_call(instance_of(I, 'http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#Collaboration') during [1.0, 2.0]).

· assert new instance of collaboration with an interval
kb_project(instance_of(ocra_filling_a_tray:'DoingSomething', 'http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#Collaboration') during [1.0, 2.0]).

· time interval for a given statement (instances of collaboration)  NOTE::IT DES NOT WORK WITH HOLDS, IT ALWAYS RETURNS (_,inf)
kb_call(triple(I, rdf:type, ocra:'Collaboration') during[T1, T2]).

· instances of collaboration within a given interval [0.0, 4.0]
kb_call((triple(I, rdf:type, ocra:'Collaboration') during[T1, T2], T1>=0.0, T2=<4.0)).

------- Queries with time interval (end) ---



------- Queries with negative assertions ---
· within the answer we will get a 'triple' formed by S, R, and O. Meaning that S not(R) O. 
kb_call(triple(D,Rd,owl:'NegativePropertyAssertion')), kb_call(triple(D, owl:'sourceIndividual', S)), kb_call(triple(D, owl:'assertionProperty', R)), kb_call(triple(D, owl:'targetIndividual', O)).

· same with time interval
kb_call(triple(D,Rd,owl:'NegativePropertyAssertion') during [T1, T2]), kb_call(triple(D, owl:'sourceIndividual', S) during [T1, T2]), kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', O) during [T1, T2]).


· assert the negative object property using rosprolog
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobot0', owl:'NegativePropertyAssertion')).
kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobot0', owl:'sourceIndividual', ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety')).
kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobot0', owl:'assertionProperty', dul:'hasParticipant')).
kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobot0', owl:'targetIndividual', ocra_filling_a_tray:'HumanOperator_0')).

· same with time interval
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'NegativePropertyAssertion') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'sourceIndividual', ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'assertionProperty', dul:'hasParticipant') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'targetIndividual', ocra_filling_a_tray:'HumanOperator_0') during [20.0, 30.0]).


kb_project(instance_of(map_piling_cloth:'_:DescriptionRobotY', owl:'NegativePropertyAssertion') during [1.0, 30.0]), kb_project(triple(map_piling_cloth:'_:DescriptionRobotY', owl:'sourceIndividual', map_piling_cloth:'eeeeso') during [1.0, 30.0]), kb_project(triple(map_piling_cloth:'_:DescriptionRobotY', owl:'assertionProperty', rdf:'type') during [1.0, 30.0]), kb_project(triple(map_piling_cloth:'_:DescriptionRobotY', owl:'targetIndividual', dul:'Plan') during [1.0, 30.0]).



------- NEEMs ---

· store the current knowledge into a NEEM 
ros_package_path('know_cra', P1), atom_concat(P1, '/NEEMs/contrastive_plans', P2), get_time(T), atom_concat(P2, T, P3), mng_dump(roslog, P3).

· restore the knowledge from a NEEM (timestamp needs to be known)
ros_package_path('know_cra', P1), atom_concat(P1, '/NEEMs/extra_neems', P2), atom_concat(P2, '1627968924.4820015', P3), atom_concat(P3, '/roslog', P4), mng_restore(roslog, P4).



--- Queries with inequality constraints -
· get all the entities related to an instance in which the relation is different to 'rdf:type' (the next is the opposite)
kb_call(triple(ocra_filling_a_tray:'HighRiskPlanAdaptation_0', R, E) during [T1, T2]),  dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R).
kb_call(triple(ocra_filling_a_tray:'HighRiskPlanAdaptation_0', R, E) during [T1, T2]), not(dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R)).

--- Queries with inequality constraints -


· storing a complete example of NEEM
kb_project(instance_of(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', ocra:'Collaboration') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', ocra:executesPlan, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', ocra:isPlanExecutedIn, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', dul:hasComponent, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', dul:isComponentOf, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:hasParticipant, ocra_filling_a_tray:'KinovaGen3_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', dul:isParticipantIn, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:hasParticipant, ocra_filling_a_tray:'HumanOperator_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'HumanOperator_0', dul:isParticipantIn, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.0, 20.0]).

kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', ocra:isPlanOf, ocra_filling_a_tray:'KinovaGen3_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', ocra:hasPlan, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', ocra:isPlanOf, ocra_filling_a_tray:'HumanOperator_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'HumanOperator_0', ocra:hasPlan, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [1.0, 20.0]).

kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', ocra:isGoalOf, ocra_filling_a_tray:'KinovaGen3_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', ocra:hasGoal, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', ocra:isGoalOf, ocra_filling_a_tray:'HumanOperator_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'HumanOperator_0', ocra:hasGoal, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [1.0, 20.0]).

kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:hasLocation, ocra_filling_a_tray:'PerceptionManipulationLab_0') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'PerceptionManipulationLab_0', dul:isLocationOf, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.0, 20.0]).
kb_project(instance_of(ocra_filling_a_tray:'PerceptionManipulationLab_0', ocra:'CollaborationPlace') during [1.0, 20.0]).

kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:isClassifiedBy, ocra_filling_a_tray:'CollaborationClass_0') during [1.0, 1.5]).
kb_project(triple(ocra_filling_a_tray:'CollaborationClass_0', dul:classifies, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.0, 1.5]).
kb_project(instance_of(ocra_filling_a_tray:'CollaborationClass_0', ocra:'Non-PhysicalCollaboration') during [1.0, 1.5]).
kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:isClassifiedBy, ocra_filling_a_tray:'CollaborationClass_1') during [1.5, 15.5]).
kb_project(triple(ocra_filling_a_tray:'CollaborationClass_1', dul:classifies, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.5, 15.5]).
kb_project(instance_of(ocra_filling_a_tray:'CollaborationClass_1', ocra:'IndirectlyPhysicalCollaboration') during [1.5, 15.5]).
kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:isClassifiedBy, ocra_filling_a_tray:'CollaborationClass_2') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborationClass_2', dul:classifies, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [15.5, 20.0]).
kb_project(instance_of(ocra_filling_a_tray:'CollaborationClass_2', ocra:'DirectlyPhysicalCollaboration') during [15.5, 20.0]).

kb_project(triple(ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety', dul:hasQuality, ocra_filling_a_tray:'CurrentRiskOfCollision') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:isQualityOf, ocra_filling_a_tray:'CollaborativelyFillingATrayWithSafety') during [1.0, 20.0]).
kb_project(instance_of(ocra_filling_a_tray:'CurrentRiskOfCollision', ocra:'CollaborationRisk') during [1.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LOW_RISK') during [1.0, 3.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'MEDIUM_RISK') during [3.0, 7.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LOW_RISK') during [7.0, 10.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'MEDIUM_RISK') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'HIGH_RISK') during [15.0, 15.5]).

kb_project(instance_of(ocra_filling_a_tray:'HighRiskPlanAdaptation_0', ocra:'PlanAdaptation') during [10.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'HighRiskPlanAdaptation_0', dul:hasParticipant, ocra_filling_a_tray:'KinovaGen3_0') during [10.0, 20.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', dul:isParticipantIn, ocra_filling_a_tray:'HighRiskPlanAdaptation_0') during [10.0, 20.0]).
kb_project(instance_of(ocra_filling_a_tray:'CollisionRiskIsHigh_0', dul:'Situation') during [15.0, 15.5]).
kb_project(triple(ocra_filling_a_tray:'CollisionRiskIsHigh_0', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9') during [15.0, 15.5]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9', dul:isPreconditionOf, ocra_filling_a_tray:'CollisionRiskIsHigh_0') during [15.0, 15.5]).
kb_project(triple(ocra_filling_a_tray:'CollisionRiskIsHigh_0', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand', dul:isPostconditionOf, ocra_filling_a_tray:'CollisionRiskIsHigh_0') during [15.5, 20.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9', dul:isPartOf, ocra_filling_a_tray:'HighRiskPlanAdaptation_0') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'HighRiskPlanAdaptation_0', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand', dul:isPartOf, ocra_filling_a_tray:'HighRiskPlanAdaptation_0') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'HighRiskPlanAdaptation_0', dul:hasPart, ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9', ocra:executesPlan, ocra_filling_a_tray:'PlacingTokenOnCompartment9') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'PlacingTokenOnCompartment9', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand', ocra:executesPlan, ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).

kb_project(triple(ocra_filling_a_tray:'PlacingTokenOnCompartment9', ocra:isWorsePlanThan, ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlacingTokenOnCompartment9') during [15.5, 20.0]).


kb_project(triple(ocra_filling_a_tray:'PlacingTokenOnCompartment9', dul:hasComponent, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', dul:isComponentOf, ocra_filling_a_tray:'PlacingTokenOnCompartment9') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand', dul:hasComponent, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', dul:isComponentOf, ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).

kb_project(triple(ocra_filling_a_tray:'PlacingTokenOnCompartment9', ocra:isPlanOf, ocra_filling_a_tray:'KinovaGen3_0') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', ocra:hasPlan, ocra_filling_a_tray:'PlacingTokenOnCompartment9') during [10.0, 15.0]).
kb_project(triple(ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand', ocra:isPlanOf, ocra_filling_a_tray:'KinovaGen3_0') during [15.5, 20.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', ocra:hasPlan, ocra_filling_a_tray:'StoppingAndRemainingCompliantUntilHumanCommand') during [15.5, 20.0]).






kb_project(instance_of(ocra_filling_a_tray:'Event_0', dul:'Event') during [20.0, 30.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'NegativePropertyAssertion') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'sourceIndividual', ocra_filling_a_tray:'Event_0') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'assertionProperty', dul:'hasParticipant') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'targetIndividual', ocra_filling_a_tray:'HumanOperator_0') during [20.0, 30.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'NegativePropertyAssertion') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'sourceIndividual', ocra_filling_a_tray:'HumanOperator_0') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'assertionProperty', dul:'isParticipantIn') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'targetIndividual', ocra_filling_a_tray:'Event_0') during [20.0, 30.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'NegativePropertyAssertion') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'sourceIndividual', ocra_filling_a_tray:'Event_0') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'assertionProperty', rdf:'type') during [20.0, 30.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'targetIndividual', ocra:'Collaboration') during [20.0, 30.0]).

kb_project(triple(ocra_filling_a_tray:'Event_0', ocra:executesPlan, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', dul:hasComponent, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', dul:isComponentOf, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'Event_0', dul:hasParticipant, ocra_filling_a_tray:'KinovaGen3_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', dul:isParticipantIn, ocra_filling_a_tray:'Event_0') during [20.0, 30.0]).

kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', ocra:isPlanOf, ocra_filling_a_tray:'KinovaGen3_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', ocra:hasPlan, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'FillingTheTrayWithTokens', ocra:isPlanOf, ocra_filling_a_tray:'HumanOperator_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'HumanOperator_0', ocra:hasPlan, ocra_filling_a_tray:'FillingTheTrayWithTokens') during [20.0, 30.0]).

kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', ocra:isGoalOf, ocra_filling_a_tray:'KinovaGen3_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'KinovaGen3_0', ocra:hasGoal, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'FullTrayUnderSafetyConditions', ocra:isGoalOf, ocra_filling_a_tray:'HumanOperator_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'HumanOperator_0', ocra:hasGoal, ocra_filling_a_tray:'FullTrayUnderSafetyConditions') during [20.0, 30.0]).

kb_project(triple(ocra_filling_a_tray:'Event_0', dul:hasLocation, ocra_filling_a_tray:'PerceptionManipulationLab_0') during [20.0, 30.0]).
kb_project(triple(ocra_filling_a_tray:'PerceptionManipulationLab_0', dul:isLocationOf, ocra_filling_a_tray:'Event_0') during [20.0, 30.0]).
kb_project(instance_of(ocra_filling_a_tray:'PerceptionManipulationLab_0', ocra:'CollaborationPlace') during [20.0, 30.0]).

NOTE THAT THERE ARE SOME RELATIONSHIPS THAT CANNOT BE APPLIED, BECAUSE IT IS NOT A COLLABORATION (E.G., RISK, COLL. TYPE, ETC.)




· examples to remove
kb_unproject(triple(ocra_filling_a_tray:'CollisionRiskIsHigh_0', dul:isPostConditionOf, ocra_filling_a_tray:'ExecutionOfPlacingTokenOnCompartment9')).
kb_unproject(triple(ocra_filling_a_tray:'ExecutionOfStoppingAndRemainingCompliantUntilHumanCommand', dul:isPostConditionOf, ocra_filling_a_tray:'CollisionRiskIsHigh_0')).

------- NEEMs (end) ---





------- COHERENT  ---

· we get the instance of the region (if the name is not self-explanatory, we cannot understand which is the level of bend)
kb_call(triple(S, rdf:type, dul:'Situation')), kb_call(triple(S, dul:includesEvent, A)), kb_call(triple(S, dul:includesObject, G)), kb_call(triple(S, dul:isSettingFor, B)), kb_call(triple(B, dul:isQualityOf, G)), kb_call(triple(B, dul:hasRegion, R)).

· we get the type of region (HIGH, MEDIUM, LOW), which helps in case the name of the instance is not self-explanatory, but we also get that the entity is a NamedIndividual, which might be undesired (need to post-process the answer)
kb_call(triple(S, rdf:type, dul:'Situation')), kb_call(triple(S, dul:includesEvent, A)), kb_call(triple(S, dul:includesObject, G)), kb_call(triple(S, dul:isSettingFor, B)), kb_call(triple(B, dul:isQualityOf, G)), kb_call(triple(B, dul:hasRegion, R)), kb_call(triple(R, rdf:type, RT)).

------- COHERENT (end) ---
