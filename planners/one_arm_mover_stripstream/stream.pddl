(define (stream namo)

	;(:stream gen-edge
	;:inputs (?q1)
	;:domain (Sampled ?q1)
	;:outputs (?q2)
	;:certified (and
	;	(BaseConf ?q2)
	;	(Edge ?q1 ?q2)
	;	(Edge ?q2 ?q1)
	;))

	;(:stream test-edge
	;:inputs (?q1 ?q2)
	;:domain (and (Sampled ?q1) (BaseConf ?q2))
	;:certified (and
	;	(Edge ?q1 ?q2)
	;	(Edge ?q2 ?q1)
	;))

	;(:stream gen-pick
	;:inputs (?o)
	;:domain (and
	;	(Pickable ?o)
	;)
	;:outputs (?q ?g)
	;:certified (and
	;	(BaseConf ?q)
	;	(Sampled ?q)
	;	(Grasp ?g)
	;	(GraspTransform ?o ?g ?q)
	;))

	;(:stream gen-place
	;:inputs (?o ?r)
	;:domain (and
	;	(Pickable ?o)
	;	(Region ?r)
	;)
	;:outputs (?q ?p)
	;:certified (and
	;	(BaseConf ?q)
	;	(Sampled ?q)
	;	(Pose ?o ?p)
	;	(PlaceConf ?o ?p ?q ?r)
	;))

	;(:stream front-place
	; :inputs (?q)
	; :domain (and (BaseConf ?q))
	; :outputs (?p)
	; :certified (and
	;	(Pose ?p)
	;	(FrontPlace ?q ?p)
	;	;(PoseInRegion ?p ?r)
	;))

	;(:predicate (FrontPick ?q ?p) (and
	;	(BaseConf ?q)
	;	(Pose ?p)
	;))

	;(:function (Distance ?q1 ?q2) (Edge ?q1 ?q2))

	(:predicate (BlocksMove ?p ?q) (and
	    (Pose ?p)
		(BaseConf ?q)
	))

	(:predicate (BlocksPlace ?p1 ?p2) (and
		(Pose ?p1)
		(Pose ?p2)
	))
)
