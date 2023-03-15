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
		; garment is grasped
		; verb = be (grasped)
		; subject = the ?g
		; prep = by ?a
		(grasped ?g - garment ?a - agent)
		; (not-grasped ?g - garment)

		; Agent ?a is free to grasp
		; verb = be (free to grasp)
		; subject = ?a
		(free-to-grasp ?a - agent)

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

		(holding ?g - garment ?r - gripper)
		(supported ?g - garment) ; garment is on a surface
		(lifted ?g - garment) ; garment is not on a surface
	)


	; Grasping a garment
	; verb = grasp / take / grab
	; subject = ?a
	; direct-object = the ?g
	(:durative-action grasp-garment
		:parameters (?g - garment ?a - agent)
		:duration (= ?duration 100)
		:condition (and 
			(at start (free-to-grasp ?a)))
		:effect (and
			(at start (not (free-to-grasp ?a)))
			(at end (grasped ?g ?a))
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
			(at start (supported ?g)))
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
	(:durative-action pile_garment ; aka transfer
		:parameters (?g - garment ?p - pile ?t - type ?a - agent)
		:duration (= ?duration 100)
		:condition (and 
			(at start (grasped ?g ?a))
			(at start (type ?g ?t))
			;; (at start (type ?p ?t)) ; if we want to divide in different piles based on the type of garment
			(at start (lifted ?g)))
		:effect (and
			(at start (not (grasped ?g ?a)))
			(at end (free-to-grasp ?a))
			(at end (piled ?g))
			(at end (on-pile ?g ?p))
			)
	)
)