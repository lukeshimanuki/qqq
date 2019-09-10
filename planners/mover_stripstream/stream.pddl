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

	(
	:stream gen-pick
	:inputs (?o ?p)
	:domain (and
		(Pickable ?o)
		(Pose ?p)
	)
	:outputs (?q ?g ?gc)
	:certified (and
		(BaseConf ?q)
		(Grasp ?o ?g ?gc)
		(Pick ?o ?p ?q ?g ?gc)
	))

	(
	:stream gen-place
	:inputs (?o ?pickp ?pickq ?g ?gc ?r)
	:domain (and
		(Pickable ?o)
		(Region ?r)
		(Grasp ?o ?g ?gc)
		(Pose ?pickp)
		(BaseConf ?pickq)
	)
	:outputs (?placeq ?placep)
	:certified (and
		(BaseConf ?placeq)
		(Pose ?placep)
		(Place ?o ?placep ?placeq ?g ?gc)
		(PoseInRegion ?placep ?r)
	))

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

	;(:predicate (BlocksMove ?p ?q) (and
	;    (Pose ?p)
	;	(BaseConf ?q)
	;))

	;(:predicate (BlocksPlace ?p1 ?p2) (and
	;	(Pose ?p1)
	;	(Pose ?p2)
	;))
)
