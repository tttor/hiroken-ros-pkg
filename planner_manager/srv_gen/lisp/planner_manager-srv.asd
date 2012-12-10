
(cl:in-package :asdf)

(defsystem "planner_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Plan" :depends-on ("_package_Plan"))
    (:file "_package_Plan" :depends-on ("_package"))
  ))