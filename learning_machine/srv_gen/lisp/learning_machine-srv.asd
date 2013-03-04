
(cl:in-package :asdf)

(defsystem "learning_machine-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CreateMetadata" :depends-on ("_package_CreateMetadata"))
    (:file "_package_CreateMetadata" :depends-on ("_package"))
    (:file "Test" :depends-on ("_package_Test"))
    (:file "_package_Test" :depends-on ("_package"))
    (:file "Train" :depends-on ("_package_Train"))
    (:file "_package_Train" :depends-on ("_package"))
  ))