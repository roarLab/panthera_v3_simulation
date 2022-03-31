;; Auto-generated. Do not edit!


(when (boundp 'panthera_locomotion::Custom_msg)
  (if (not (find-package "PANTHERA_LOCOMOTION"))
    (make-package "PANTHERA_LOCOMOTION"))
  (shadow 'Custom_msg (find-package "PANTHERA_LOCOMOTION")))
(unless (find-package "PANTHERA_LOCOMOTION::CUSTOM_MSG")
  (make-package "PANTHERA_LOCOMOTION::CUSTOM_MSG"))

(in-package "ROS")
;;//! \htmlinclude Custom_msg.msg.html


(defclass panthera_locomotion::Custom_msg
  :super ros::object
  :slots (_current _speed _target_speed _pid_speed ))

(defmethod panthera_locomotion::Custom_msg
  (:init
   (&key
    ((:current __current) 0.0)
    ((:speed __speed) 0.0)
    ((:target_speed __target_speed) 0.0)
    ((:pid_speed __pid_speed) 0.0)
    )
   (send-super :init)
   (setq _current (float __current))
   (setq _speed (float __speed))
   (setq _target_speed (float __target_speed))
   (setq _pid_speed (float __pid_speed))
   self)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:target_speed
   (&optional __target_speed)
   (if __target_speed (setq _target_speed __target_speed)) _target_speed)
  (:pid_speed
   (&optional __pid_speed)
   (if __pid_speed (setq _pid_speed __pid_speed)) _pid_speed)
  (:serialization-length
   ()
   (+
    ;; float64 _current
    8
    ;; float64 _speed
    8
    ;; float64 _target_speed
    8
    ;; float64 _pid_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_speed
       (sys::poke _target_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pid_speed
       (sys::poke _pid_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_speed
     (setq _target_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pid_speed
     (setq _pid_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get panthera_locomotion::Custom_msg :md5sum-) "70d45acb8d6001240e5e0b4d267d7330")
(setf (get panthera_locomotion::Custom_msg :datatype-) "panthera_locomotion/Custom_msg")
(setf (get panthera_locomotion::Custom_msg :definition-)
      "float64 current
float64 speed
float64 target_speed
float64 pid_speed
")



(provide :panthera_locomotion/Custom_msg "70d45acb8d6001240e5e0b4d267d7330")


