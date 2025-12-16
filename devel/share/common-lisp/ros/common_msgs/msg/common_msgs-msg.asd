
(cl:in-package :asdf)

(defsystem "common_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Float64Stamped" :depends-on ("_package_Float64Stamped"))
    (:file "_package_Float64Stamped" :depends-on ("_package"))
  ))