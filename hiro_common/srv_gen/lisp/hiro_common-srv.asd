
(cl:in-package :asdf)

(defsystem "hiro_common-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "GetManipulability" :depends-on ("_package_GetManipulability"))
    (:file "_package_GetManipulability" :depends-on ("_package"))
    (:file "BenchmarkPath" :depends-on ("_package_BenchmarkPath"))
    (:file "_package_BenchmarkPath" :depends-on ("_package"))
  ))