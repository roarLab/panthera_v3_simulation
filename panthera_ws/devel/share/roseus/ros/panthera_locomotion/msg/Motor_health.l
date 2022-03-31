;; Auto-generated. Do not edit!


(when (boundp 'panthera_locomotion::Motor_health)
  (if (not (find-package "PANTHERA_LOCOMOTION"))
    (make-package "PANTHERA_LOCOMOTION"))
  (shadow 'Motor_health (find-package "PANTHERA_LOCOMOTION")))
(unless (find-package "PANTHERA_LOCOMOTION::MOTOR_HEALTH")
  (make-package "PANTHERA_LOCOMOTION::MOTOR_HEALTH"))

(in-package "ROS")
;;//! \htmlinclude Motor_health.msg.html


(defclass panthera_locomotion::Motor_health
  :super ros::object
  :slots (_fuse_condition ))

(defmethod panthera_locomotion::Motor_health
  (:init
   (&key
    ((:fuse_condition __fuse_condition) 0.0)
    )
   (send-super :init)
   (setq _fuse_condition (float __fuse_condition))
   self)
  (:fuse_condition
   (&optional __fuse_condition)
   (if __fuse_condition (setq _fuse_condition __fuse_condition)) _fuse_condition)
  (:serialization-length
   ()
   (+
    ;; float64 _fuse_condition
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _fuse_condition
       (sys::poke _fuse_condition (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _fuse_condition
     (setq _fuse_condition (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get panthera_locomotion::Motor_health :md5sum-) "4b811d7770caee98dbe252ed8c9b8311")
(setf (get panthera_locomotion::Motor_health :datatype-) "panthera_locomotion/Motor_health")
(setf (get panthera_locomotion::Motor_health :definition-)
      "float64 fuse_condition

")



(provide :panthera_locomotion/Motor_health "4b811d7770caee98dbe252ed8c9b8311")


