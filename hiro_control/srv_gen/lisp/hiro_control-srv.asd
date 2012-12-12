
(cl:in-package :asdf)

(defsystem "hiro_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "CommitMovingArm" :depends-on ("_package_CommitMovingArm"))
    (:file "_package_CommitMovingArm" :depends-on ("_package"))
    (:file "MoveArm" :depends-on ("_package_MoveArm"))
    (:file "_package_MoveArm" :depends-on ("_package"))
    (:file "ControlArm" :depends-on ("_package_ControlArm"))
    (:file "_package_ControlArm" :depends-on ("_package"))
    (:file "ControlHand" :depends-on ("_package_ControlHand"))
    (:file "_package_ControlHand" :depends-on ("_package"))
    (:file "GotoGoal" :depends-on ("_package_GotoGoal"))
    (:file "_package_GotoGoal" :depends-on ("_package"))
  ))