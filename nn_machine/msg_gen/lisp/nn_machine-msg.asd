
(cl:in-package :asdf)

(defsystem "nn_machine-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Feature" :depends-on ("_package_Feature"))
    (:file "_package_Feature" :depends-on ("_package"))
  ))