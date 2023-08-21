:- module(plan_disambiguation,
    [ compare_pairs_of_instances_of(r),
	  compare_two_plans(r,r), 
      compare_two_plans_based_on_cost(r,r)
    ]).

/** <module> Predicates for comparative reasoning with plans

@author Alberto Olivaes-Alarcos
@license BSD
*/

%% compare_pairs_of_instances_of(?C) 
% 
% Compares all instances of an ontological class in pairs (e.g., based on their qualities and their roles) to decide which one is better. 
% The logic of this predicate is applicataion dependent. 
%
% @param C         Ontological class
%
compare_pairs_of_instances_of(C) :-
	instance_of(Ia, C), instance_of(Ib, C), dif(Ia, Ib), 
	(C == "dul:'Plan'") -> compare_two_plans(Ia, Ib) ; false. % TODO : It is not working, the others yes. 

%% compare_two_plans(?Pa, ?Pb) 
% 
% Compares two plans (e.g., based on their qualities and their roles) to decide which one is better. 
% The logic of this predicate is applicataion dependent. 
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans(Pa, Pb) :-
	compare_two_plans_based_on_cost(Pa, Pb). 


%% (?Pa, ?Pb) 
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

