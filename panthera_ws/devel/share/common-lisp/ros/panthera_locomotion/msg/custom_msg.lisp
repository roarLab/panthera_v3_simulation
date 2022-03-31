; Auto-generated. Do not edit!


(cl:in-package panthera_locomotion-msg)


;//! \htmlinclude Custom_msg.msg.html

(cl:defclass <Custom_msg> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (target_speed
    :reader target_speed
    :initarg :target_speed
    :type cl:float
    :initform 0.0)
   (pid_speed
    :reader pid_speed
    :initarg :pid_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass Custom_msg (<Custom_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Custom_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Custom_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panthera_locomotion-msg:<Custom_msg> is deprecated: use panthera_locomotion-msg:Custom_msg instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <Custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-msg:current-val is deprecated.  Use panthera_locomotion-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-msg:speed-val is deprecated.  Use panthera_locomotion-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'target_speed-val :lambda-list '(m))
(cl:defmethod target_speed-val ((m <Custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-msg:target_speed-val is deprecated.  Use panthera_locomotion-msg:target_speed instead.")
  (target_speed m))

(cl:ensure-generic-function 'pid_speed-val :lambda-list '(m))
(cl:defmethod pid_speed-val ((m <Custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-msg:pid_speed-val is deprecated.  Use panthera_locomotion-msg:pid_speed instead.")
  (pid_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Custom_msg>) ostream)
  "Serializes a message object of type '<Custom_msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pid_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Custom_msg>) istream)
  "Deserializes a message object of type '<Custom_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Custom_msg>)))
  "Returns string type for a message object of type '<Custom_msg>"
  "panthera_locomotion/Custom_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Custom_msg)))
  "Returns string type for a message object of type 'Custom_msg"
  "panthera_locomotion/Custom_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Custom_msg>)))
  "Returns md5sum for a message object of type '<Custom_msg>"
  "70d45acb8d6001240e5e0b4d267d7330")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Custom_msg)))
  "Returns md5sum for a message object of type 'Custom_msg"
  "70d45acb8d6001240e5e0b4d267d7330")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Custom_msg>)))
  "Returns full string definition for message of type '<Custom_msg>"
  (cl:format cl:nil "float64 current~%float64 speed~%float64 target_speed~%float64 pid_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Custom_msg)))
  "Returns full string definition for message of type 'Custom_msg"
  (cl:format cl:nil "float64 current~%float64 speed~%float64 target_speed~%float64 pid_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Custom_msg>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Custom_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Custom_msg
    (cl:cons ':current (current msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':target_speed (target_speed msg))
    (cl:cons ':pid_speed (pid_speed msg))
))
