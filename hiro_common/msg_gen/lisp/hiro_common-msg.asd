
(cl:in-package :asdf)

(defsystem "hiro_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PathQuality" :depends-on ("_package_PathQuality"))
    (:file "_package_PathQuality" :depends-on ("_package"))
  ))