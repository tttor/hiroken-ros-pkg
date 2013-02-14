
(cl:in-package :asdf)

(defsystem "ws_anal-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EvalWS" :depends-on ("_package_EvalWS"))
    (:file "_package_EvalWS" :depends-on ("_package"))
  ))