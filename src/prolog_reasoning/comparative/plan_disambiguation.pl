:- module(plan_disambiguation,
    [ compare_all_existing_plans_in_pairs/0,
	  compare_two_plans(r,r), 
      compare_two_plans_based_on_cost(r,r),
	  compare_two_plans_based_on_number_of_tasks(r,r),
	  compare_two_plans_based_on_makespan(r,r)
    ]).

/** <module> Predicates for comparative reasoning with plans

@author Alberto Olivaes-Alarcos
@license BSD
*/

%% compare_all_existing_plans_in_pairs() 
% 
% Compares all instances of a plan in pairs (e.g., based on their qualities and their roles) to decide which one is better. 
% The logic of this predicate is application dependent. 
%
%
compare_all_existing_plans_in_pairs() :-
	instance_of(Ia, dul:'Plan'), instance_of(Ib, dul:'Plan'), dif(Ia, Ib) -> compare_two_plans(Ia, Ib); false.  

%% compare_two_plans(?Pa, ?Pb) 
% 
% Compares two plans (e.g., based on their qualities and their roles) to decide which one is better. 
% The logic of this predicate is application dependent. 
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans(Pa, Pb) :-
	compare_two_plans_based_on_cost(Pa, Pb),
	compare_two_plans_based_on_number_of_tasks(Pa, Pb),
	compare_two_plans_based_on_makespan(Pa, Pb). 


%% compare_two_plans_based_on_cost(?Pa, ?Pb) 
% 
% Compares the cost of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_cost(Pa, Pb) :-
	triple(Pa, ocra_common:'hasCost', Xa), 
	triple(Pb, ocra_common:'hasCost', Xb), 
	triple(Xa, dul:'hasDataValue', Va), 
	triple(Xb, dul:'hasDataValue', Vb), 
	(ground(Xa), ground(Xb)) ->
	((Va < Vb) -> kb_project(triple(Xa, ocra_common:'hasBetterQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasWorseQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isMoreExpensivePlanThan', Pa)), kb_project(triple(Pa, ocra_common:'isCheaperPlanThan', Pb)) ; 
	kb_project(triple(Xb, ocra_common:'hasBetterQualityValueThan', Xa)), kb_project(triple(Xa, ocra_common:'hasWorseQualityValueThan', Xb)), kb_project(triple(Pa, ocra_common:'isMoreExpensivePlanThan', Pb)), kb_project(triple(Pb, ocra_common:'isCheaperPlanThan', Pa))) ; false. 


%% compare_two_plans_based_on_number_of_tasks(?Pa, ?Pb) 
% 
% Compares the number of tasks of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_number_of_tasks(Pa, Pb) :-
	triple(Pa, ocra_common:'hasNumberOfTasks', Xa), 
	triple(Pb, ocra_common:'hasNumberOfTasks', Xb), 
	triple(Xa, dul:'hasDataValue', Va), 
	triple(Xb, dul:'hasDataValue', Vb), 
	(ground(Xa), ground(Xb)) ->
	((Va < Vb) -> kb_project(triple(Xa, ocra_common:'hasBetterQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasWorseQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isPlanWithMoreTasksThan', Pa)), kb_project(triple(Pa, ocra_common:'isPlanWithLessTasksThan', Pb)) ; 
	kb_project(triple(Xb, ocra_common:'hasBetterQualityValueThan', Xa)), kb_project(triple(Xa, ocra_common:'hasWorseQualityValueThan', Xb)), kb_project(triple(Pa, ocra_common:'isPlanWithMoreTasksThan', Pb)), kb_project(triple(Pb, ocra_common:'isPlanWithLessTasksThan', Pa))) ; false. 


%% compare_two_plans_based_on_makespan(?Pa, ?Pb) 
% 
% Compares the makespan of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_makespan(Pa, Pb) :-
	triple(Pa, ocra_common:'hasExpectedMakespan', Xa), 
	triple(Pb, ocra_common:'hasExpectedMakespan', Xb), 
	triple(Xa, dul:'hasDataValue', Va), 
	triple(Xb, dul:'hasDataValue', Vb), 
	(ground(Xa), ground(Xb)) ->
	((Va < Vb) -> kb_project(triple(Xa, ocra_common:'hasBetterQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasWorseQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isSlowerPlanThan', Pa)), kb_project(triple(Pa, ocra_common:'isFasterPlanThan', Pb)) ; 
	kb_project(triple(Xb, ocra_common:'hasBetterQualityValueThan', Xa)), kb_project(triple(Xa, ocra_common:'hasWorseQualityValueThan', Xb)), kb_project(triple(Pa, ocra_common:'isSlowerPlanThan', Pb)), kb_project(triple(Pb, ocra_common:'isFasterPlanThan', Pa))) ; false. 

