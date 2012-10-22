
(cl:in-package :asdf)

(defsystem "hiro_common-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "BenchmarkPath" :depends-on ("_package_BenchmarkPath"))
    (:file "_package_BenchmarkPath" :depends-on ("_package"))
  ))