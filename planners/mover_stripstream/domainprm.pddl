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
		(Pick ?o ?p ?q ?g)
		(Place ?o ?p ?q ?g ?r)
		(Picked ?o ?p ?q ?g)
		(PlacedAt ?p)
		(Grasp ?g)

		(GoalObject ?o)
		(NonGoalObject ?o)
		(GoalRegion ?r)
		(NonGoalRegion ?r)

		(InGoal ?o)
		(Near ?q1 ?q2) ; prm_q, sampled_q

		(Edge ?q1 ?q2)
		(Reach ?q1 ?q2)
		(UnsafeMove2 ?q)
		(UnsafeMove ?q1 ?q2)
		(UnsafeCarry ?q1 ?q2 ?o ?g ?gc ?pickp ?pickq)
		(UnsafePlace ?p)
		(BlocksMove ?p ?q)
		(BlocksPlace ?p1 ?p2)
		(BlocksMoveS ?p ?q)
		(CollidesMove ?q1 ?q2 ?o ?p)
		(CollidesCarry ?q1 ?q2 ?held_obj ?g ?gc ?pickp ?pickq ?o ?p)
		(Moved)

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
	:parameters (?q1 ?o ?p ?q2 ?g ?r)
	:precondition (and
		(BaseConf ?q1)
		(Pickable ?o)
		(Pose ?p)
		(Sampled ?q2)
		(Grasp ?g)
		(Pick ?o ?p ?q2 ?g)
		(Region ?r)

		(EmptyArm)
		(InRegion ?o ?r)
		(AtPose ?o ?p)
		(AtConf ?q1)

		(Near ?q1 ?q2)
		;(not (UnsafeCarry ?q1 ?o ?p ?q2 ?g))
		;(not (UnsafeCarry ?q2 ?o ?p ?q2 ?g))
	)
	:effect (and
		(Picked ?o ?p ?q2 ?g)
		(not (EmptyArm))
		(not (InRegion ?o ?r))
		(not (AtPose ?o ?p))
	))

	(
	:action place
	:parameters (?q ?o ?pickp ?placep ?pickq ?placeq ?g ?r)
	:precondition (and
		(BaseConf ?q)
		(Pickable ?o)
		;(NonGoalObject ?o)
		(Pose ?pickp)
		(Pose ?placep)
		(Sampled ?pickq)
		(Sampled ?placeq)
		(Grasp ?g)
		(Region ?r)
		(NonGoalRegion ?r)
		(Place ?o ?placep ?placeq ?g ?r)

		(AtConf ?q)
		(Picked ?o ?pickp ?pickq ?g)

		;(Near ?q ?placeq)
		;(not (UnsafeCarry ?placeq ?o ?pickp ?pickq ?g))
	)
	:effect (and
		(AtPose ?o ?placep)
		(EmptyArm)
		(not (Picked ?o ?pickp ?pickq ?g))
		;(not (InGoal ?o))
		(InRegion ?o ?r)
	))

	(
	:action placegoal
	:parameters (?q ?o ?pickp ?placep ?pickq ?placeq ?g ?r)
	:precondition (and
		(BaseConf ?q)
		(Pickable ?o)
		(GoalObject ?o)
		(Pose ?pickp)
		(Pose ?placep)
		(Sampled ?pickq)
		(Sampled ?placeq)
		(Grasp ?g)
		(Region ?r)
		(GoalRegion ?r)
		(Place ?o ?placep ?placeq ?g ?r)

		(AtConf ?q)
		(Picked ?o ?pickp ?pickq ?g)

		;(Near ?q ?placeq)
		;(not (UnsafeCarry ?placeq ?o ?pickp ?pickq ?g))
	)
	:effect (and
		(AtPose ?o ?placep)
		(EmptyArm)
		(not (Picked ?o ?pickp ?pickq ?g))
		;(InGoal ?o)
		(InRegion ?o ?r)
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

	(
	:action teleport
	:parameters (?q1 ?q2)
	:precondition (and
		(BaseConf ?q1)
		(BaseConf ?q2)

		(AtConf ?q1)
	)
	:effect (and
		(AtConf ?q2)
		(not (AtConf ?q1))
	))

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

	;(:derived (UnsafeCarry ?q1 ?q2 ?oo ?g ?gc ?pickp ?pickq) (exists (?o ?p) (and
	;	(AtPose ?o ?p)
	;	(CollidesCarry ?q1 ?q2 ?oo ?g ?gc ?pickp ?pickq ?o ?p)
	;)))
)

