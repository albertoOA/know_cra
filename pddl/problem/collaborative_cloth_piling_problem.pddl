(define (problem piling)
	(:domain collaborative-cloth-piling)
	(:objects
		towel1 rag1  - garment
		towel rag - type
		pile1 - pile
		;; pile2 - pile ; if we want to divide in different piles based on the type of garment
		gerard - human
		tiago - robot
	)

	(:init
		(free-to-grasp gerard)
		(free-to-grasp tiago)

		(type towel1 towel)
		(type rag1 rag)

		(supported towel1)
		(supported rag1)

		;; (type pile1 towel) ; if we want to divide in different piles based on the type of garment
		;; (type pile2 rag) ; if we want to divide in different piles based on the type of garment
	)

	(:goal
		(and 
			(piled towel1)
			(piled rag1)
		)
	)
)