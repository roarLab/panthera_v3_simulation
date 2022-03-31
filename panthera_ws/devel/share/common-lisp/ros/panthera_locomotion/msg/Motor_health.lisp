; Auto-generated. Do not edit!


(cl:in-package panthera_locomotion-msg)


;//! \htmlinclude Motor_health.msg.html

(cl:defclass <Motor_health> (roslisp-msg-protocol:ros-message)
  ((fuse_condition
    :reader fuse_condition
    :initarg :fuse_condition
    :type cl:float
    :initform 0.0))
)

(cl:defclass Motor_health (<Motor_health>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Motor_health>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Motor_health)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panthera_locomotion-msg:<Motor_health> is deprecated: use panthera_locomotion-msg:Motor_health instead.")))

(cl:ensure-generic-function 'fuse_condition-val :lambda-list '(m))
(cl:defmethod fuse_condition-val ((m <Motor_health>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panthera_locomotion-msg:fuse_condition-val is deprecated.  Use panthera_locomotion-msg:fuse_condition instead.")
  (fuse_condition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Motor_health>) ostream)
  "Serializes a message object of type '<Motor_health>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fuse_condition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Motor_health>) istream)
  "Deserializes a message object of type '<Motor_health>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fuse_condition) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Motor_health>)))
  "Returns string type for a message object of type '<Motor_health>"
  "panthera_locomotion/Motor_health")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Motor_health)))
  "Returns string type for a message object of type 'Motor_health"
  "panthera_locomotion/Motor_health")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Motor_health>)))
  "Returns md5sum for a message object of type '<Motor_health>"
  "4b811d7770caee98dbe252ed8c9b8311")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Motor_health)))
  "Returns md5sum for a message object of type 'Motor_health"
  "4b811d7770caee98dbe252ed8c9b8311")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Motor_health>)))
  "Returns full string definition for message of type '<Motor_health>"
  (cl:format cl:nil "float64 fuse_condition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Motor_health)))
  "Returns full string definition for message of type 'Motor_health"
  (cl:format cl:nil "float64 fuse_condition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Motor_health>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Motor_health>))
  "Converts a ROS message object to a list"
  (cl:list 'Motor_health
    (cl:cons ':fuse_condition (fuse_condition msg))
))
