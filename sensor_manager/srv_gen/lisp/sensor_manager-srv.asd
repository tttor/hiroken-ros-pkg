
(cl:in-package :asdf)

(defsystem "sensor_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Sense" :depends-on ("_package_Sense"))
    (:file "_package_Sense" :depends-on ("_package"))
  ))