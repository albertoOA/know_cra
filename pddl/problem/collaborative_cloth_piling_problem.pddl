(define (problem piling)
	(:domain collaborative-cloth-piling)
	(:objects
		towel-01 towel-02 rag-01  - garment
		towel rag - garment-type
		pile-01 - pile
		;; pile-02 - pile ; if we want to divide in different piles based on the type of garment
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

		(type towel-01 towel)
		(type towel-02 towel)
		(type rag-01 rag)

		(folded towel-01)
		(unfolded towel-02)
		(unfolded rag-01)

		(graspable towel-01)
		(graspable towel-02)
		(graspable rag-01)

		(supported towel-01)
		(supported towel-02)
		(supported rag-01)

		;; (type pile1 towel) ; if we want to divide in different piles based on the type of garment
		;; (type pile2 rag) ; if we want to divide in different piles based on the type of garment
	)

	(:goal
		(and 
			(piled towel-01)
			(piled towel-02)
			(piled rag-01)
		)
	)
)