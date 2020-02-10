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
		(Pick ?o ?p ?q ?g ?gc)
		(Place ?o ?p ?q ?g ?gc)
		(Picked ?o ?g ?gc ?p ?q)
		(PlacedAt ?p)
		(Grasp ?o ?g ?gc)

		(GoalObject ?o)
		(NonGoalRegion ?r)

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

	(
	:action pap
	:parameters (?o ?r ?prev_r ?params ?s ?t)
	:precondition (and
		(Pickable ?o)
		(GoalObject ?o)
		(Region ?r)
		(Region ?prev_r)
		(State ?s)
		(State ?t)
		(PaP ?o ?r ?params ?s ?t)

		(InRegion ?o ?prev_r)
		(AtState ?s)
	)
	:effect (and
		(not (InRegion ?o ?prev_r))
		(not (AtState ?s))
		(AtState ?t)
		(InRegion ?o ?r)
	))

	(
	:action nongoalpap
	:parameters (?o ?r ?prev_r ?params ?s ?t)
	:precondition (and
		(Pickable ?o)
		(Region ?r)
		(NonGoalRegion ?r)
		(Region ?prev_r)
		(State ?s)
		(State ?t)
		(PaP ?o ?r ?params ?s ?t)

		(InRegion ?o ?prev_r)
		(AtState ?s)
	)
	:effect (and
		(not (InRegion ?o ?prev_r))
		(not (AtState ?s))
		(AtState ?t)
		(InRegion ?o ?r)
	))

	;(
	;:action pick
	;:parameters (?o ?p ?q ?g ?gc)
	;:precondition (and
	;	(Pickable ?o)
	;	(Pose ?p)
	;	(BaseConf ?q)
	;	(Grasp ?o ?g ?gc)
	;	(Pick ?o ?p ?q ?g ?gc)

	;	(EmptyArm)
	;	(AtPose ?o ?p)
	;	(AtConf ?q)

	;	;(not (UnsafePick ?o ?p ?q ?g))
	;)
	;:effect (and
	;	(Picked ?o ?g ?gc ?p ?q)
	;	(not (EmptyArm))
	;	(not (AtPose ?o ?p))
	;))

	;(
	;:action place
	;:parameters (?o ?pickp ?placep ?pickq ?placeq ?g ?gc ?r)
	;:precondition (and
	;	(Pickable ?o)
	;	(Pose ?pickp)
	;	(Pose ?placep)
	;	(BaseConf ?pickq)
	;	(BaseConf ?placeq)
	;	(Grasp  ?o ?g ?gc)
	;	(Place ?o ?placep ?placeq ?g ?gc)
	;	(Region ?r)

	;	(AtConf ?placeq)
	;	(Picked ?o ?g ?gc ?pickp ?pickq)

	;	;(not (UnsafePlace ?o ?p ?q ?g))
	;)
	;:effect (and
	;	(AtPose ?o ?placep)
	;	(EmptyArm)
	;	(not (Picked ?o ?g ?gc ?pickp ?pickq))
	;	;(Moved)
	;))

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
	;	(AtConf ?q1)
	;	(BaseConf ?q1)
	;	(BaseConf ?q2)
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

	;(:derived (UnsafeCarry ?q1 ?q2 ?oo ?g ?gc ?pickp ?pickq) (exists (?o ?p) (and
	;	(AtPose ?o ?p)
	;	(CollidesCarry ?q1 ?q2 ?oo ?g ?gc ?pickp ?pickq ?o ?p)
	;)))
)

