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

		(different ?r - agent ?h - agent)
	)


	; Grasps a garment
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

)