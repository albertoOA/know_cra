(define (problem piling)
	(:domain collaborative-cloth-piling)
	(:objects
		shirt1  - garment
		gerard - human
		tiago - robot
	)

	(:init
		(free-to-grasp gerard)
		(free-to-grasp tiago)

		(different gerard tiago)
		(different tiago gerard)
	)

	(:goal
		(and 
			(grasped shirt1 gerard)
		)
	)
)