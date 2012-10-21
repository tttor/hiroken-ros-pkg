
(cl:in-package :asdf)

(defsystem "grasp_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :arm_navigation_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "PlanGrasp" :depends-on ("_package_PlanGrasp"))
    (:file "_package_PlanGrasp" :depends-on ("_package"))
  ))