; Alberto Olivares-Alarcos
(define (domain collaborative-cloth-piling)

	(:requirements :strips :typing :durative-actions :action-costs)


	(:types 
		robot human - agent
		garment pile agent - physical-object

		garment-type - concept
		concept - social-object

		social-object physical-object - object
		object - entity
		entity
	)

	(:predicates
		; object is grasped by an agent
		; verb = be (grasped)
		; subject = the ?g
		; prep = by ?a
		(grasped-by ?o - object ?a - agent)
		
		; object is available to be grasped
		; verb = be (available to be grasped)
		; subject = the ?o
		(graspable ?o - object)

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

		; Garment ?g is supported (on a surface and not on a pile)
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



		; Predicate to clasify entities using ontological concepts in the DUL ontology
		; ?e is classified as concept ?c
		; verb = be classified
		; subject = the ?e
		; prep = by concept ?c 
		;; (is-classified-by ?e - entity ?c - concept)

		; Predicate to qualify entities using ontological qualities in the DUL ontology
		; ?e has quality ?q
		; verb = have
		; subject = the ?e
		; object = ?q 
		;; (has-quality ?e - entity ?q - quality) ;; define quality as Entity
	)

	(:functions
		(manipulate-folded-time ?a - agent)
		(place-unfolded-time ?a - agent)

		(current-number-of-garments-on-pile ?p - pile)
		(target-number-of-garments-on-pile ?p - pile)
	)


	; Grasping a folded garment (to pile it)
	; verb = grasp / take / grab
	; subject = ?a
	; direct-object = the ?g
	(:durative-action grasp-folded-garment
		:parameters (?g - garment ?a - agent)
		:duration (= ?duration (manipulate-folded-time ?a))
		:condition (and 
			(at start (free-to-manipulate ?a))
			(at start (folded ?g))
			(at start (graspable ?g))
			(at start (supported ?g))
		)
		:effect (and
			(at start (not (free-to-manipulate ?a)))
			(at start (not (graspable ?g)))
			(at end (grasped-by ?g ?a))
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
			(at end (grasped-by ?g ?h))
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
			(at start (grasped-by ?g ?a))
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
	; prep = of garment-type ?t
	(:durative-action pile-garment ; aka transfer
		:parameters (?g - garment ?p - pile ?t - garment-type ?a - agent)
		:duration (= ?duration (manipulate-folded-time ?a))
		:condition (and 
			(at start (grasped-by ?g ?a))
			;; (at start (is-classified-by ?g ?t))
			;; (at start (is-classified-by ?p ?t)) ; if we want to divide in different piles based on the type of garment
			(at start (lifted ?g))
			(at start (folded ?g))
		)
		:effect (and
			(at start (not (grasped-by ?g ?a)))
			(at end (graspable ?g))
			(at end (free-to-manipulate ?a))
			(at end (piled ?g))
			(at end (on-pile ?g ?p))
			(at end (increase (current-number-of-garments-on-pile ?p) 1))
		)
	)

	; Handing an unfolded garment
	; verb = hand / give / pass
	; subject = ?r
	; direct-object = the ?g
	; indirect-object = to ?h
	(:durative-action hand-unfolded-garment ; aka transfer
		:parameters (?g - garment ?r - robot ?h - human)
		:duration (= ?duration 50)
		:condition (and 
			(at start (grasped-by ?g ?r))
			(at start (free-to-manipulate ?h))
			(at start (lifted ?g))
			(at start (unfolded ?g))
		)
		:effect (and
			(at start (not (grasped-by ?g ?r)))
			(at start (not (free-to-manipulate ?h)))
			(at start (not (graspable ?g)))
			(at end (grasped-by ?g ?h))
			(at end (free-to-manipulate ?r))
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
			(at start (grasped-by ?g ?h))
		)
		:effect (and
			(at end (free-to-manipulate ?h))
			(at end (not (unfolded ?g)))
			(at end (not (lifted ?g)))
			(at end (not (grasped-by ?g ?h)))
			(at end (graspable ?g))
			(at end (folded ?g))
			(at end (supported ?g))
		)
	)

	; Placing an unfolded garment
	; verb = place / lay / set
	; subject = ?a
	; direct-object = the ?g
	(:durative-action place-unfolded-garment
		:parameters (?g - garment ?a - agent)
		:duration (= ?duration (place-unfolded-time ?a))
		:condition (and 
			(at start (unfolded ?g))
			(at start (lifted ?g))
			(at start (grasped-by ?g ?a))
		)
		:effect (and
			(at end (free-to-manipulate ?a))
			(at end (not (lifted ?g)))
			(at end (not (grasped-by ?g ?a)))
			(at end (graspable ?g))
			(at end (supported ?g))
		)
	)

	; Grasping a pile of garments
	; verb = grasp / take / grab
	; subject = ?h
	; direct-object = the ?p
	(:durative-action grasp-pile-of-garments
		:parameters (?p - pile ?h - human)
		:duration (= ?duration 100)
		:condition (and 
			(at start (free-to-manipulate ?h))
			(at start (= (current-number-of-garments-on-pile ?p) (target-number-of-garments-on-pile ?p)))
			(at start (graspable ?p))
		)
		:effect (and
			(at start (not (free-to-manipulate ?h)))
			(at start (not (graspable ?p)))
			(at end (grasped-by ?p ?h))
		)
	)
)
