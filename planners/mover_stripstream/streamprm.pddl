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

	;(
	;:stream gen-pap
	;:inputs (?o ?r ?s)
	;:domain (and
	;	(Pickable ?o)
	;	(Region ?r)
	;	(State ?s)
	;)
	;:outputs (?params ?t)
	;:certified (and
	;	(State ?t)
	;	(PaP ?o ?r ?params ?s ?t)
	;))

	(
	:stream transform-pick
	:inputs (?o ?g ?q)
	:domain (and
		(Pickable ?o)
		(Grasp ?g)
		(BaseConf ?q)
	)
	:outputs (?p ?s)
	:certified (and
		(Pose ?p)
		(Sampled ?s)
		(Pick ?o ?p ?q ?s ?g)
	))

	;(
	;:stream gen-pick
	;:inputs (?o ?p)
	;:domain (and
	;	(Pickable ?o)
	;	(Pose ?p)
	;	;(BaseConf ?prm_q)
	;)
	;:outputs (?q ?g)
	;:certified (and
	;	(Sampled ?q)
	;	(Grasp ?g)
	;	(Pick ?o ?p ?q ?g)
	;	;(Near ?prm_q ?q)
	;))

	;(
	;:stream gen-place
	;:inputs (?o ?pickp ?pickq ?g ?r)
	;:domain (and
	;	(Pickable ?o)
	;	(Pose ?pickp)
	;	(Sampled ?pickq)
	;	(Grasp ?g)
	;	(Pick ?o ?pickp ?pickq ?g)
	;	(Region ?r)
	;	;(BaseConf ?prm_q)
	;)
	;:outputs (?placeq ?placep)
	;:certified (and
	;	(Sampled ?placeq)
	;	(Pose ?placep)
	;	(Place ?o ?placep ?placeq ?g ?r)
	;	(NearPose ?placeq ?placep)
	;))

	;(
	;:stream gen-conf
	;:inputs (?r)
	;:domain (and
	;	(Region ?r)
	;)
	;:outputs (?q)
	;:certified (and
	;	(BaseConf ?q)
	;	(ConfInRegion ?q ?r)
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

	;(:predicate (NotNear ?q1 ?q2) (and
	;	(BaseConf ?q1)
	;	(Sampled ?q2)
	;))

	;(:predicate (BlocksMove ?p ?q) (and
	;    (Pose ?p)
	;	(BaseConf ?q)
	;))

	;(:predicate (BlocksPlace ?p1 ?p2) (and
	;	(Pose ?p1)
	;	(Pose ?p2)
	;))

	;(:predicate (CollidesMove ?q1 ?q2 ?o ?p) (and
	;	(BaseConf ?q1)
	;	(BaseConf ?q2)
	;	(Pickable ?o)
	;	(Pose ?p)
	;))

	;(:predicate (CollidesCarry ?q1 ?q2 ?oo ?g ?gc ?pickp ?pickq ?o ?p) (and
	;	(BaseConf ?q1)
	;	(BaseConf ?q2)
	;	(Pickable ?oo)
	;	(Grasp ?o ?g ?gc)
	;	(Pose ?pickp)
	;	(BaseConf ?pickq)
	;	(Pickable ?o)
	;	(Pose ?p)
	;))
)
