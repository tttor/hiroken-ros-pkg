
(cl:in-package :asdf)

(defsystem "nn_machine-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nn_machine-msg
)
  :components ((:file "_package")
    (:file "RunNet" :depends-on ("_package_RunNet"))
    (:file "_package_RunNet" :depends-on ("_package"))
    (:file "TrainNet" :depends-on ("_package_TrainNet"))
    (:file "_package_TrainNet" :depends-on ("_package"))
  ))