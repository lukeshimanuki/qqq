(define (domain mover)
	(:requirements :strips :equality)
	(:predicates
		(Pickable ?o)
		(EmptyArm)
		(BaseConf ?q)
		(Sampled ?q)
		(PlaceConf ?o ?p ?q ?region)
		(AtPose ?o ?p)
		(AtConf ?q)
		(InRegion ?o ?r)
		(PoseInRegion ?p ?r)
		(Region ?r)
		(IntendedRegion ?region)
		(Pose ?p)
		(Pick ?o ?p ?q ?s ?g) ; s is sampled baseconf near q
		(Place ?o ?p ?q ?g ?r)
		(Picked ?o ?g)
		(PlacedAt ?p)
		(Grasp ?g)

		(GoalObject ?o)
		(NonGoalObject ?o)
		(GoalRegion ?r)
		(NonGoalRegion ?r)
		(QInRegion ?q ?r)
		(Q ?q)

		(InGoal ?o)
		(Near ?q1 ?q2) ; prm_q, sampled_q

		(Edge ?q1 ?q2)
		(Reach ?q1 ?q2)
		(UnsafeMove2 ?q)
		(UnsafeMove ?q1 ?q2)
		(UnsafeCarry ?q1 ?q2 ?o ?g)
		(UnsafePlace ?p)
		(BlocksMove ?p ?q)
		(BlocksPlace ?p1 ?p2)
		(BlocksMoveS ?p ?q)
		(CollidesMove ?o ?p ?q1 ?q2)
		(CollidesCarry ?o ?p ?q1 ?q2 ?held_obj ?g)
		(Moved ?o)

		(FrontPlace ?q ?p)
		(FrontPick ?q ?p)

		(PaP ?o ?r ?pickt ?placet ?gc ?s ?t)
		(AtState ?s)
		(State ?s)
		(PaP ?o ?r ?params ?s ?t)
	)
	;(:functions (Distance ?q1 ?q2))

	;(:action vaporize
	; :parameters (?o ?p)
	; :precondition (and
	;	 (Pickable ?o)
	;	(Pose ?p)
	;	(AtPose ?o ?p)
	;	 ;(Picked ?o)
	; )
	; :effect (and
	;	(not (AtPose ?o ?p))
	;	 ;(not (Picked ?o))
	;))

	; move from q1 to q2, pick, move back to q1
	(
	:action pick
	:parameters (?o ?p ?q ?s ?g ?r)
	:precondition (and
		(Pickable ?o)
		(Pose ?p)
		(Q ?q)
		(BaseConf ?q)
		(Q ?s)
		(Sampled ?s)
		(Grasp ?g)
		(Pick ?o ?p ?q ?s ?g)
		(Region ?r)

		(EmptyArm)
		(InRegion ?o ?r)
		(AtPose ?o ?p)
		(AtConf ?q)

		(not (UnsafeMove ?q ?s))
		(not (UnsafeCarry ?s ?q ?o ?g))
	)
	:effect (and
		(Picked ?o ?g)
		(not (EmptyArm))
		(not (InRegion ?o ?r))
		(not (AtPose ?o ?p))
	))

	(
	:action place
	:parameters (?o ?p ?q ?s ?g ?r)
	:precondition (and
		(Pickable ?o)
		(Pose ?p)
		(Q ?q)
		(BaseConf ?q)
		(Q ?s)
		(Sampled ?s)
		(Grasp ?g)
		(Pick ?o ?p ?q ?s ?g)
		(Region ?r)
		(NonGoalRegion ?r)
		(QInRegion ?q ?r)

		(AtConf ?q)
		(Picked ?o ?g)

		(not (UnsafeCarry ?q ?s ?o ?g))
		(not (UnsafeMove ?s ?q))
	)
	:effect (and
		(AtPose ?o ?p)
		(EmptyArm)
		(not (Picked ?o ?g))
		(InRegion ?o ?r)
		(Moved ?o)
	))

	(
	:action placegoal
	:parameters (?o ?p ?q ?s ?g ?r)
	:precondition (and
		(Pickable ?o)
		(GoalObject ?o)
		(Pose ?p)
		(Q ?q)
		(BaseConf ?q)
		(Q ?s)
		(Sampled ?s)
		(Grasp ?g)
		(Pick ?o ?p ?q ?s ?g)
		(Region ?r)
		(GoalRegion ?r)
		(QInRegion ?q ?r)

		(AtConf ?q)
		(Picked ?o ?g)

		(not (UnsafeCarry ?q ?s ?o ?g))
		(not (UnsafeMove ?s ?q))
	)
	:effect (and
		(AtPose ?o ?p)
		(EmptyArm)
		(not (Picked ?o ?g))
		(InRegion ?o ?r)
		(Moved ?o)
	))

	(
	:action move
	:parameters (?q1 ?q2)
	:precondition (and
		(Q ?q1)
		(BaseConf ?q1)
		(Q ?q2)
		(BaseConf ?q2)
		(Edge ?q1 ?q2)

		(AtConf ?q1)
		(EmptyArm)

		(not (UnsafeMove ?q1 ?q2))
	)
	:effect (and
		(AtConf ?q2)
		(not (AtConf ?q1))
	))

	(
	:action carry
	:parameters (?q1 ?q2 ?o ?g)
	:precondition (and
		(Q ?q1)
		(BaseConf ?q1)
		(Q ?q2)
		(BaseConf ?q2)
		(Edge ?q1 ?q2)
		(Pickable ?o)
		(Grasp ?g)

		(AtConf ?q1)
		(Picked ?o ?g)

		(not (UnsafeCarry ?q1 ?q2 ?o ?g))
	)
	:effect (and
		(AtConf ?q2)
		(not (AtConf ?q1))
	))

	;(
	;:action move
	;:parameters (?q1 ?q2)
	;:precondition (and
	;	(BaseConf ?q1)
	;	(BaseConf ?q2)

	;	(EmptyArm)
	;	(AtConf ?q1)
	;	;(not (UnsafeConf ?q2))
	;	;(Edge ?q1 ?q2)
	;	(not (UnsafeMove ?q1 ?q2))
	;)
	;:effect (and
	;	(AtConf ?q2)
	;	(not (AtConf ?q1))
	;))

	;(
	;:action carry
	;:parameters (?q1 ?q2 ?o ?g ?gc ?pickp ?pickq)
	;:precondition (and
	;	(BaseConf ?q1)
	;	(BaseConf ?q2)

	;	(Pickable ?o)
	;	(Pose ?pickp)
	;	(BaseConf ?pickq)
	;	(Grasp ?o ?g ?gc)

	;	(not (EmptyArm))
	;	(Picked ?o ?g ?gc ?pickp ?pickq)
	;	(AtConf ?q1)
	;	;(not (UnsafeConfCarry ?q2 ?o ?g ?gc ?pickp ?pickq))
	;	;(Edge ?q1 ?q2)
	;	;(not (UnsafeCarry ?q1 ?q2 ?o ?g ?gc ?pickp ?pickq))
	;)
	;:effect (and
	;	(AtConf ?q2)
	;	(not (AtConf ?q1))
	;))

	;(
	;:action teleport
	;:parameters (?q1 ?q2)
	;:precondition (and
	;	(BaseConf ?q1)
	;	(BaseConf ?q2)

	;	(AtConf ?q1)
	;)
	;:effect (and
	;	(AtConf ?q2)
	;	(not (AtConf ?q1))
	;))

	;(:derived (InRegion ?o ?r) (exists (?p) (and
	;	(Pickable ?o)
	;	(Region ?r)
	;	(Pose ?p)
	;	(AtPose ?o ?p)
	;	(PoseInRegion ?p ?r)
	;	;(Moved)
	;)))

	;(:derived (Reachable ?q) (or
	;	 (and (BaseConf ?q) (AtConf ?q))
	;	 (exists (?q2) (and (Edge ?q2 ?q) (not (UnsafeMove ?q2)) (Reachable ?q2)))
	;))

	;(:derived (UnsafeMove ?q) (exists (?p) (and
	;	(BlocksMove ?p ?q)
	;	(PlacedAt ?p)
	;)))

	;(:derived (UnsafePlace ?pose) (exists (?p) (and
	;	(BlocksPlace ?pose ?p)
	;	(PlacedAt ?p)
	;)))

	;(:derived (UnsafeMove ?q1 ?q2) (exists (?o ?p) (and
	;	(AtPose ?o ?p)
	;	(CollidesMove ?q1 ?q2 ?o ?p)
	;)))

	(:derived (UnsafeMove ?q1 ?q2) (exists (?o ?p) (and
		(AtPose ?o ?p)
		(Q ?q1)
		(Q ?q2)
		(CollidesMove ?o ?p ?q1 ?q2)
	)))

	(:derived (UnsafeCarry ?q1 ?q2 ?oo ?g) (exists (?o ?p) (and
		(AtPose ?o ?p)
		(Q ?q1)
		(Q ?q2)
		(CollidesCarry ?o ?p ?q1 ?q2 ?oo ?g)
	)))
)

