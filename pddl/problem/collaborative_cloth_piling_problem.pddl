(define (problem piling)
	(:domain collaborative-cloth-piling)
	(:objects
		towel1 towel2 rag1  - garment
		towel rag - type
		pile1 - pile
		;; pile2 - pile ; if we want to divide in different piles based on the type of garment
		indigo - human
		kinova - robot
	)

	(:init
		(free-to-manipulate indigo)
		(free-to-manipulate kinova)

		(different indigo kinova)
		(different kinova indigo)

		(= (grasp-time indigo) 200)
		(= (grasp-time kinova) 100)

		(type towel1 towel)
		(type towel2 towel)
		(type rag1 rag)

		(folded towel1)
		(unfolded towel2)
		(unfolded rag1)

		(graspable towel1)
		(graspable towel2)
		(graspable rag1)

		(supported towel1)
		(supported towel2)
		(supported rag1)

		;; (type pile1 towel) ; if we want to divide in different piles based on the type of garment
		;; (type pile2 rag) ; if we want to divide in different piles based on the type of garment
	)

	(:goal
		(and 
			(piled towel1)
			(piled towel2)
			(piled rag1)
		)
	)
)