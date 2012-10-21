
(cl:in-package :asdf)

(defsystem "task_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :arm_navigation_msgs-msg
)
  :components ((:file "_package")
    (:file "PlanTask" :depends-on ("_package_PlanTask"))
    (:file "_package_PlanTask" :depends-on ("_package"))
  ))