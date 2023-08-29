:- module(plan_disambiguation,
    [ compare_all_existing_plans_in_pairs/0,
	  compare_two_plans(r,r), 
      compare_two_plans_based_on_cost(r,r)
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
	compare_two_plans_based_on_cost(Pa, Pb). 


%% compare_two_plans_based_on_cost(?Pa, ?Pb) 
% 
% Compares the cost of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_cost(Pa, Pb) :-
	triple(Pa, ocra_common:'hasCost', Ca), 
	triple(Pb, ocra_common:'hasCost', Cb), 
	triple(Ca, dul:'hasDataValue', Va), 
	triple(Cb, dul:'hasDataValue', Vb), 
	(ground(Ca), ground(Cb)) ->
	((Va < Vb) -> kb_project(triple(Ca, ocra_common:'hasBetterQualityValueThan', Cb)), kb_project(triple(Cb, ocra_common:'hasWorseQualityValueThan', Ca)), kb_project(triple(Pb, ocra_common:'isMoreExpensivePlanThan', Pa)), kb_project(triple(Pa, ocra_common:'isCheaperPlanThan', Pb)) ; 
	kb_project(triple(Cb, ocra_common:'hasBetterQualityValueThan', Ca)), kb_project(triple(Ca, ocra_common:'hasWorseQualityValueThan', Cb)), kb_project(triple(Pa, ocra_common:'isMoreExpensivePlanThan', Pb)), kb_project(triple(Pb, ocra_common:'isCheaperPlanThan', Pa))) ; false. 

