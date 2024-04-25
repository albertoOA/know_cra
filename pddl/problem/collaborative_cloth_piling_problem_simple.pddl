(define (problem piling)
	(:domain collaborative-cloth-piling)
	(:objects
		towel-01 towel-02 dish-towel-01  - garment
		towel dish-towel - garment-type
		pile-01 - pile
		;; pile-02 - pile ; if we want to divide in different piles based on the type of garment
		human-01 - human
		robot-01 - robot
	)

	(:init
		(free-to-manipulate human-01)
		(free-to-manipulate robot-01)

		(different human-01 robot-01)
		(different robot-01 human-01)

		(= (manipulate-folded-time human-01) 200)
		(= (manipulate-folded-time robot-01) 100)

		(= (current-number-of-garments-on-pile pile-01) 0)
		(= (target-number-of-garments-on-pile pile-01) 3)

		;; (is-classified-by towel-01 towel)
		;; (is-classified-by towel-02 towel)
		;; (is-classified-by dish-towel-01 dish-towel)

		(folded towel-01)
		(unfolded towel-02)
		(unfolded dish-towel-01)

		(graspable towel-01)
		(graspable towel-02)
		(graspable dish-towel-01)

		(graspable pile-01)

		(supported towel-01)
		(supported towel-02)
		(supported dish-towel-01)

		;; (is-classified-by pile1 towel) ; if we want to divide in different piles based on the type of garment
		;; (is-classified-by pile2 dish-towel) ; if we want to divide in different piles based on the type of garment
	)

	(:goal
		(and 
			(piled towel-01)
			(piled towel-02)
			(piled dish-towel-01)
			(on-pile towel-01 pile-01)
			(on-pile towel-02 pile-01)
			(on-pile dish-towel-01 pile-01)
			(grasped-by pile-01 human-01)
		)
	)
)
