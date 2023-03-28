; Alberto Olivares-Alarcos
(define (domain collaborative-cloth-piling)

	(:requirements :strips :typing :durative-actions :action-costs)


	(:types 
		garment pile type agent - object
		robot human - agent
		agent
		object
	)

	(:predicates
		; garment is (not) grasped
		; verb = be (grasped)
		; subject = the ?g
		; prep = by ?a
		(grasped ?g - garment ?a - agent)
		
		; garment is available to be grasped
		; verb = be (available to be grasped)
		; subject = the ?g
		(graspable ?g - garment)

		; Agent ?a is free to manipulate
		; verb = be (free to manipulate)
		; subject = ?a
		(free-to-manipulate ?a - agent)

		; Garment ?g is on the pile ?p
		; verb = be
		; subject = the ?g
		; prep = on the pile ?p !
		(on-pile ?g - garment ?p - pile)

		; Garment ?g is piled
		; verb = be (piled)
		; subject = the ?g
		(piled ?g - garment)

		; ?g is of type ?t
		; verb = be
		; subject = the ?g
		; prep = of type ?t !
		(type ?g - object ?t - type)

		; Garment ?g is supported (on a surface)
		; verb = be (supported)
		; subject = the ?g
		(supported ?g - garment) 

		; Garment ?g is lifted (not on a surface)
		; verb = be (lifted)
		; subject = the ?g
		(lifted ?g - garment) 
		
		; Garment ?g is folded
		; verb = be (folded)
		; subject = the ?g
		(folded ?g - garment)

		; Garment ?g is unfolded
		; verb = be (unfolded)
		; subject = the ?g
		(unfolded ?g - garment)

		(different ?r - agent ?h - agent)
	)

	(:functions
		(grasp-time ?a - agent)
	)


	; Grasping a folded garment (to pile it)
	; verb = grasp / take / grab
	; subject = ?a
	; direct-object = the ?g
	(:durative-action grasp-folded-garment
		:parameters (?g - garment ?a - agent)
		:duration (= ?duration (grasp-time ?a))
		:condition (and 
			(at start (free-to-manipulate ?a))
			(at start (folded ?g))
			(at start (graspable ?g))
		)
		:effect (and
			(at start (not (free-to-manipulate ?a)))
			(at start (not (graspable ?g)))
			(at end (grasped ?g ?a))
		)
	)

	; Grasping an unfolded garment (to fold it)
	; verb = grasp / take / grab
	; subject = ?a
	; direct-object = the ?g
	(:durative-action grasp-unfolded-garment
		:parameters (?g - garment ?h - human)
		:duration (= ?duration 100)
		:condition (and 
			(at start (free-to-manipulate ?h))
			(at start (unfolded ?g))
			(at start (graspable ?g))
		)
		:effect (and
			(at start (not (free-to-manipulate ?h)))
			(at start (not (graspable ?g)))
			(at end (grasped ?g ?h))
		)
	)

	; Lifting a grasped garment
	; verb = lift 
	; subject = ?a
	; direct-object = the ?g
	(:durative-action lift ; aka removetablecontact
		:parameters (?g - garment ?a - agent)
		:duration (= ?duration 100)
		:condition (and
			(at start (grasped ?g ?a))
			(at start (supported ?g))
		)
		:effect (and
			(at end (not (supported ?g)))
			(at end (lifted ?g))
		)
	)

	; Piling a garment
	; verb = pile / stack / place
	; subject = ?a
	; direct-object = the ?g
	; prep = on the ?p !
	; prep = of type ?t
	(:durative-action pile-garment ; aka transfer
		:parameters (?g - garment ?p - pile ?t - type ?a - agent)
		:duration (= ?duration (grasp-time ?a))
		:condition (and 
			(at start (grasped ?g ?a))
			(at start (type ?g ?t))
			;; (at start (type ?p ?t)) ; if we want to divide in different piles based on the type of garment
			(at start (lifted ?g))
			(at start (folded ?g))
		)
		:effect (and
			(at start (not (grasped ?g ?a)))
			(at end (graspable ?g))
			(at end (free-to-manipulate ?a))
			(at end (piled ?g))
			(at end (on-pile ?g ?p))
		)
	)

	; Folding a garment
	; verb = fold
	; subject = ?h
	; direct-object = the ?g
	(:durative-action fold-garment
		:parameters (?g - garment ?h - human)
		:duration (= ?duration 100)
		:condition (and 
			(at start (unfolded ?g))
			(at start (lifted ?g))
			(at start (grasped ?g ?h))
		)
		:effect (and
			(at end (free-to-manipulate ?h))
			(at end (not (unfolded ?g)))
			(at end (not (lifted ?g)))
			(at end (not (grasped ?g ?h)))
			(at end (graspable ?g))
			(at end (folded ?g))
			(at end (supported ?g))
		)
	)
)