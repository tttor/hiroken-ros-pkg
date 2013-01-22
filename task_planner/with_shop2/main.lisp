; author: vektor dewanto
; ver: Jan 2013; Oct, 2012

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

(defdomain tidying-up 
  (
    (:operator (!transit ?r ?l1 ?l2) 
      ( (in ?r ?l1) (empty ?r) ) 
      ( (in ?r ?l1) ) 
      ( (in ?r ?l2) )
    )
    (:operator (!grasp ?o ?l ?r) 
      ( (in ?o ?l) (in ?r ?l) (empty ?r) ) 
      ( (empty ?r) (in ?o ?l) ) 
      ( (holding ?r ?o) )
    )
    (:operator (!transfer ?r ?o ?l1 ?l2) 
      ( (in ?r ?l1) (holding ?r ?o) ) 
      ( (in ?r ?l1) ) 
      ( (in ?r ?l2) )
    )
    (:operator (!ungrasp ?o ?l ?r) 
      ( (in ?r ?l) (holding ?r ?o) ) 
      ( (holding ?r ?o)  ) 
      ( (in ?o ?l) (empty ?r) )
    )
    ;=====================================================================================================
    ; The method "pick" is for picking an object o at object location ol with a robot r at its initial location rl
    ; precond: the object o is at ol AND the robot r is at its initial location rl AND the robot r must not hold anything
    ; actions: (1) transitting the robot r from its initial position rl to object location ol, then
    ;          (2) grasping the object o, which is in ol, with r
    (:method (pick ?o ?ol ?r ?rl)
      ( (in ?o ?ol) (in ?r ?rl) (empty ?r) )
      (:ordered (!transit ?r ?rl ?ol) (!grasp ?o ?ol ?r) )
    )
    
    ; The method "place" is for placing an object o at a placing location pl with a robot r holding the object o at its location rl
    ; precond: the robot r is holding an object ?o AND it is in its initial location rl
    ; actions: (1) transfering the robot r that holds an object o from the robot initial location rl to the placing location pl, then
    ;          (2) ungrasping the object o (of the robot r) at the placing location pl
    (:method (place ?o ?pl ?r ?rl)
      ( (holding ?r ?o) (in ?r ?rl) )
      (:ordered (!transfer ?r ?o ?rl ?pl) (!ungrasp ?o ?pl ?r) )
    )

    (:method (gohome ?r ?wl ?rhl)
      ( (in ?r ?wl) )
      (:ordered (!transit ?r ?wl ?rhl) )
    )
        
    ; The method "pick-and-place" is for picking an object o from a picking location pickl to a placing location placel with a robot r that is in its location rl
    ; precond: the object o is in the picking location pickl AND the robot r is empty AND the robot r is at its initial location rl
    ; actions: (1) picking the object o from the picking location ?pickl with a robot r at its initial location rl
    ;          (2) placing the object o at the placin location placel with a robot r at its location, which is then pickl
    (:method (pick-and-place ?o ?pickl ?placel ?r)
      ( (in ?o ?pickl) (empty ?r) )
      (:ordered (pick ?o ?pickl ?r ?rl) (place ?o ?placel ?r ?pickl) )
    )
    
    ; The method "tidyup" is for moving all objects in the messy spot messy-spot to the specified tidy spot tidy-spot with a single robot r at its location rl
    (:method (tidyup ?messy-spot ?tidy-spot ?r ?rhl)
      ( (in ?o ?messy-spot) )
      (:ordered (pick-and-place ?o ?messy-spot ?tidy-spot ?r) (tidyup ?messy-spot ?tidy-spot ?r ?rhl) )
      ( )
      (:ordered (gohome ?r ?tidy-spot ?rhl) )
    )

    (:method (tidyup-2-monoton ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl)
      ( (in ?o ?messy-spot) )
      (:ordered (tidyup ?messy-spot ?tidy-spot ?r ?rhl) )
    )
    
    (:method (pick-and-place-2 ?o ?pickl ?placel ?ar ?pr ?rhl)
      ( (in ?o ?pickl) (empty ?ar) (in ?pr ?placel) )
      (:ordered (gohome ?pr ?placel ?rhl) (pick ?o ?pickl ?ar ?arl) (place ?o ?placel ?ar ?pickl) )
      
      ( (in ?o ?pickl) (empty ?ar) )
      (:ordered (pick ?o ?pickl ?ar ?arl) (place ?o ?placel ?ar ?pickl) )
    )
    ;=====================================================================================================
    ; This method almost succeed in that it produces not-monoton plan in pick-and-place
    ; However, it misses to move passive arm go home, if it is in the tidy-config
    (:method (tidyup-2-almost ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl)
      ( (in ?o ?messy-spot) )
      (:ordered (pick-and-place ?o ?messy-spot ?tidy-spot ?r) (tidyup-2-almost ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl) )
      
      ( in ?r1 ?tidy-spot)
      (:ordered (gohome ?r1 ?tidy-spot ?rhl) )
      ( in ?r2 ?tidy-spot)
      (:ordered (gohome ?r2 ?tidy-spot ?rhl) )
    )
    
    ; How to differentiate between (1) directly go to messy-spot from tidy-spot and (2) change arm, then go to home from tidy-spot for the passive arm  
    ; How to differentiate between (1) pick from tidy-spot and (2) pick from home
    ; How to say (not r) if (pick-and-place ?o ?messy-spot ?tidy-spot ?r)
    
    (:method (tidyup-2 ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl)
      ( (in ?o ?messy-spot) (in ?r1 ?tidy-spot) )
      (:ordered (gohome ?r1 ?tidy-spot ?rhl) (tidyup-2 ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl) )
      ( (in ?o ?messy-spot) (in ?r2 ?tidy-spot) )
      (:ordered (gohome ?r2 ?tidy-spot ?rhl) (tidyup-2 ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl) )

      ( (in ?o ?messy-spot) )
      (:ordered (pick-and-place ?o ?messy-spot ?tidy-spot ?r) (tidyup-2 ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl) )

      ( in ?r1 ?tidy-spot)
      (:ordered (gohome ?r1 ?tidy-spot ?rhl) )
      ( in ?r2 ?tidy-spot)
      (:ordered (gohome ?r2 ?tidy-spot ?rhl) )
    )
    
    
    ; This method (with pick-and-place-2) almost succeeds in that 
    ; This remove the possibility of having the same always works for more than 2 object
    ; NOT OKAY, for more than 2 object, This is buggy
    (:method (tidyup-2-X ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl)
      ( (in ?o ?messy-spot) )
      (:ordered (pick-and-place-2 ?o ?messy-spot ?tidy-spot ?r ?r ?rhl) (tidyup-2-X ?messy-spot ?tidy-spot ?r1 ?r2 ?rhl) )
      ()
      ()
    )
    ;=====================================================================================================
  )
)

(defproblem after-party-problem tidying-up
  ( 
    ;the initial messy-spot config
    (in can1 messy-spot)
    (in can2 messy-spot) 
    (in can3 messy-spot)
    (in can4 messy-spot)
    (in can5 messy-spot)
    ;(in can6 messy-spot)
    
    ;the initial tidy-spot config
    ()
    
    ;the initial robots' state
    (empty rarm) (in rarm home)
    (empty larm) (in larm home)
  ) 
  
  ( (tidyup-2 messy-spot tidy-spot rarm larm home) )

; ( (tidyup-2-almost messy-spot tidy-spot rarm larm home) )
 
;  ( (pick-and-place-2 can1 messy-spot tidy-spot rarm larm home) )


; ( (tidyup-2-monoton messy-spot tidy-spot rarm larm home) )
    
;  ( (tidyup messy-spot tidy-spot rarm home) )

;  ( (pick-and-place can1 messy-spot tidy-spot rarm) )
)

;(shop-trace :methods)
;(shop-trace :all)

(find-plans 'after-party-problem :verbose :long-plans :which :all)
;(find-plans 'after-party-problem :verbose :long-plans :which :first)

;(find-plans 'after-party-problem :verbose :long-plans :which :first :plan-tree :true)
