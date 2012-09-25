
(cl:in-package :asdf)

(defsystem "laser_to_wall-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WallScan" :depends-on ("_package_WallScan"))
    (:file "_package_WallScan" :depends-on ("_package"))
  ))