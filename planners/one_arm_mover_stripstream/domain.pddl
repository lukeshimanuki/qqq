(define (domain mover)
    (:requirements :strips :equality)
    (:predicates (Pickable ?o)
                 (EmptyArm)
                 (BaseConf ?q)
                 (Sampled ?q)
                 (PlaceConf ?o ?p ?q ?region)
                 (AtPose ?o ?p)
                 (AtConf ?q)
                 (InRegion ?o ?region)
				 (PoseInRegion ?q ?region)
                 (Region ?region)
                 (IntendedRegion ?region)
                 (Pose ?p)
                 (Picked ?o)
                 (PlacedAt ?p)

				 (Edge ?q1 ?q2)
				 (Reach ?q1 ?q2)
				 (UnsafeMove ?q)
				 (UnsafePlace ?p)
				 (BlocksMove ?p ?q)
				 (BlocksPlace ?p1 ?p2)
				 (BlocksMoveS ?p ?q)

				 (FrontPlace ?q ?p)
				 (FrontPick ?q ?p)
                 )
    (:functions (Distance ?q1 ?q2))

	;(:action vaporize
	; :parameters (?o ?p)
	; :precondition (and
	;    (Pickable ?o)
	;	(Pose ?p)
	;	(AtPose ?o ?p)
    ;    ;(Picked ?o)
	; )
	; :effect (and
	;	(not (AtPose ?o ?p))
    ;    ;(not (Picked ?o))
	;))

    (:action pick
     :parameters (?obj ?pose ?conf)
     :precondition (and
		(Pickable ?obj)
		(FrontPick ?conf ?pose)
		; (FrontPlace ?conf ?pose)

        (EmptyArm)
        (AtPose ?obj ?pose)
        (AtConf ?conf)
     )
     :effect (and
        (Picked ?obj)
        (not (EmptyArm))
        (not (PlacedAt ?pose))
        (not (AtPose ?obj ?pose))
        ; (increase (total-cost) 1)
    ))

    (:action place
     :parameters (?obj ?pose ?conf)
     :precondition (and
		(Pickable ?obj)
		(FrontPick ?conf ?pose)
		; (FrontPlace ?conf ?pose)

        (AtConf ?conf)
        (Picked ?obj)
        (not (PlacedAt ?pose))
		; (not (UnsafePlace ?pose))
     )
     :effect (and
        (AtPose ?obj ?pose)
        (EmptyArm)
        (PlacedAt ?pose)
        (not (Picked ?obj))
        ; (increase (total-cost) 1)
    ))

	(:action move
	 :parameters (?q1 ?q2)
	 :precondition (and
		(Edge ?q1 ?q2)
		(AtConf ?q1)
		(not (UnsafeMove ?q2))
	 )
	 :effect (and
	    (AtConf ?q2)
		(not (AtConf ?q1))
        ; (increase (total-cost) (Distance ?q1 ?q2))
	))

    ;(:action teleport
	; :parameters (?q1 ?q2)
	; :precondition (and
	;	(and (Conf ?q1 ?q2) (Conf ?q1 ?q2))
	;	(AtConf ?q1)
	;	(not (UnsafeMove ?q2))
	; )
	; :effect (and
	;    (AtConf ?q2)
	;	(not (AtConf ?q1))
    ;    ; (increase (total-cost) (Distance ?q1 ?q2))
	;))

	(:derived (InRegion ?obj ?region) (exists (?pose) (and
	    (Pickable ?obj)
		; (PoseInRegion ?conf ?region)
		(PoseInRegion ?pose ?region)
	    (AtPose ?obj ?pose)
	)))

	;(:derived (Reachable ?q) (or
    ;    (and (BaseConf ?q) (AtConf ?q))
    ;    (exists (?q2) (and (Edge ?q2 ?q) (not (UnsafeMove ?q2)) (Reachable ?q2)))
	;))

	(:derived (UnsafeMove ?q) (exists (?p) (and
		(BlocksMove ?p ?q)
	    (PlacedAt ?p)
	)))

	(:derived (UnsafePlace ?pose) (exists (?p) (and
		(BlocksPlace ?pose ?p)
	    (PlacedAt ?p)
	)))
)

