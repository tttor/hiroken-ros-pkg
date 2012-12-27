
(cl:in-package :asdf)

(defsystem "hiro_sensor-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "See" :depends-on ("_package_See"))
    (:file "_package_See" :depends-on ("_package"))
  ))