; author: vektor dewanto
; ver: Oct 2, 2012

; This is for the pick-and-place task
; r for ROBOT
; l for LOCATION
; o for OBJECT

(in-package :shop2-user)

(defun plan-found-hook (state which plan cost depth)
  (
    with-open-file (str "plan.shop2"
                     :direction :output
                     :if-exists :append
                     :if-does-not-exist :create)
    (format str "~A ~%" plan)
  )
)

(defdomain tidy-up 
  (
    (:operator (!transit ?r ?l1 ?l2) 
      ( (in ?r ?l1) (empty ?r) ) 
      ( (in ?r ?l1) ) 
      ( (in ?r ?l2) )
    )
    (:operator (!grasp ?r ?o ?l) 
      ( (in ?r ?l) (empty ?r) (in ?o ?l) ) 
      ( (empty ?r) (in ?o ?l) ) 
      ( (holding ?r ?o) )
    )
    (:operator (!transfer ?r ?o ?l1 ?l2) 
      ( (in ?r ?l1) (holding ?r ?o) ) 
      ( (in ?r ?l1) ) 
      ( (in ?r ?l2) )
    )
    (:operator (!ungrasp ?r ?o ?l) 
      ( (in ?r ?l) (holding ?r ?o) ) 
      ( (holding ?r ?o)  ) 
      ( (in ?o ?l) (empty ?r) )
    )
    
    (:method (pick-and-place ?l1 ?l2)
      ( (in ?o ?l1) )
      (:ordered (!transit ?r ?l0 ?l1) (!grasp ?r ?o ?l1) (!transfer ?r ?o ?l1 ?l2) (!ungrasp ?r ?o ?l2) )
    )
    (:method (tidy ?messy-spot ?tidy-spot ?r1 ?home1 ?r2 ?home2)
      ( (in ?o ?messy-spot) )
      (:ordered (pick-and-place ?messy-spot ?tidy-spot) (tidy ?messy-spot ?tidy-spot ?r1 ?home1 ?r2 ?home2) )
      ( )
      ( (!transit ?r1 ?tidy-spot ?home1) (!transit ?r2 ?tidy-spot ?home2) );
    )
  )
)
(defproblem after-party-problem tidy-up
  ( 
    ;the initial messy-spot config
    (in can1 messy-spot); (in can2 messy-spot) (in can3 messy-spot) (in can4 messy-spot) (in can5 messy-spot) (in can6 messy-spot) (in can7 messy-spot)
    
    ;the initial tidy-spot config
    ()
    
    ;the initial robot's state
    (empty rarm) (in rarm rhome) (empty larm) (in larm lhome)
  ) 
  ( (tidy messy-spot tidy-spot rarm rhome larm lhome) )
)

;(shop-trace :methods)
;(shop-trace :all)

(find-plans 'after-party-problem :verbose :long-plans :which :all)
;(find-plans 'after-party-problem :verbose :long-plans :which :first :plan-tree :true)
