
(cl:in-package :asdf)

(defsystem "panthera_locomotion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Custom_msg" :depends-on ("_package_Custom_msg"))
    (:file "_package_Custom_msg" :depends-on ("_package"))
    (:file "Custom_msg" :depends-on ("_package_Custom_msg"))
    (:file "_package_Custom_msg" :depends-on ("_package"))
    (:file "Motor_health" :depends-on ("_package_Motor_health"))
    (:file "_package_Motor_health" :depends-on ("_package"))
    (:file "Motor_health" :depends-on ("_package_Motor_health"))
    (:file "_package_Motor_health" :depends-on ("_package"))
  ))