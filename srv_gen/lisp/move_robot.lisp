; Auto-generated. Do not edit!


(cl:in-package ptam-srv)


;//! \htmlinclude move_robot-request.msg.html

(cl:defclass <move_robot-request> (roslisp-msg-protocol:ros-message)
  ((start_x
    :reader start_x
    :initarg :start_x
    :type cl:float
    :initform 0.0)
   (start_y
    :reader start_y
    :initarg :start_y
    :type cl:float
    :initform 0.0)
   (start_theta
    :reader start_theta
    :initarg :start_theta
    :type cl:float
    :initform 0.0)
   (goal_x
    :reader goal_x
    :initarg :goal_x
    :type cl:float
    :initform 0.0)
   (goal_y
    :reader goal_y
    :initarg :goal_y
    :type cl:float
    :initform 0.0)
   (goal_theta
    :reader goal_theta
    :initarg :goal_theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass move_robot-request (<move_robot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_robot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_robot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<move_robot-request> is deprecated: use ptam-srv:move_robot-request instead.")))

(cl:ensure-generic-function 'start_x-val :lambda-list '(m))
(cl:defmethod start_x-val ((m <move_robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:start_x-val is deprecated.  Use ptam-srv:start_x instead.")
  (start_x m))

(cl:ensure-generic-function 'start_y-val :lambda-list '(m))
(cl:defmethod start_y-val ((m <move_robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:start_y-val is deprecated.  Use ptam-srv:start_y instead.")
  (start_y m))

(cl:ensure-generic-function 'start_theta-val :lambda-list '(m))
(cl:defmethod start_theta-val ((m <move_robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:start_theta-val is deprecated.  Use ptam-srv:start_theta instead.")
  (start_theta m))

(cl:ensure-generic-function 'goal_x-val :lambda-list '(m))
(cl:defmethod goal_x-val ((m <move_robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:goal_x-val is deprecated.  Use ptam-srv:goal_x instead.")
  (goal_x m))

(cl:ensure-generic-function 'goal_y-val :lambda-list '(m))
(cl:defmethod goal_y-val ((m <move_robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:goal_y-val is deprecated.  Use ptam-srv:goal_y instead.")
  (goal_y m))

(cl:ensure-generic-function 'goal_theta-val :lambda-list '(m))
(cl:defmethod goal_theta-val ((m <move_robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:goal_theta-val is deprecated.  Use ptam-srv:goal_theta instead.")
  (goal_theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_robot-request>) ostream)
  "Serializes a message object of type '<move_robot-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_robot-request>) istream)
  "Deserializes a message object of type '<move_robot-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_theta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_theta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_robot-request>)))
  "Returns string type for a service object of type '<move_robot-request>"
  "ptam/move_robotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_robot-request)))
  "Returns string type for a service object of type 'move_robot-request"
  "ptam/move_robotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_robot-request>)))
  "Returns md5sum for a message object of type '<move_robot-request>"
  "533042a64ee768c9dd424ef588119ef4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_robot-request)))
  "Returns md5sum for a message object of type 'move_robot-request"
  "533042a64ee768c9dd424ef588119ef4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_robot-request>)))
  "Returns full string definition for message of type '<move_robot-request>"
  (cl:format cl:nil "float64 start_x~%float64 start_y~%float64 start_theta~%float64 goal_x~%float64 goal_y~%float64 goal_theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_robot-request)))
  "Returns full string definition for message of type 'move_robot-request"
  (cl:format cl:nil "float64 start_x~%float64 start_y~%float64 start_theta~%float64 goal_x~%float64 goal_y~%float64 goal_theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_robot-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_robot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'move_robot-request
    (cl:cons ':start_x (start_x msg))
    (cl:cons ':start_y (start_y msg))
    (cl:cons ':start_theta (start_theta msg))
    (cl:cons ':goal_x (goal_x msg))
    (cl:cons ':goal_y (goal_y msg))
    (cl:cons ':goal_theta (goal_theta msg))
))
;//! \htmlinclude move_robot-response.msg.html

(cl:defclass <move_robot-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass move_robot-response (<move_robot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_robot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_robot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<move_robot-response> is deprecated: use ptam-srv:move_robot-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_robot-response>) ostream)
  "Serializes a message object of type '<move_robot-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_robot-response>) istream)
  "Deserializes a message object of type '<move_robot-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_robot-response>)))
  "Returns string type for a service object of type '<move_robot-response>"
  "ptam/move_robotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_robot-response)))
  "Returns string type for a service object of type 'move_robot-response"
  "ptam/move_robotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_robot-response>)))
  "Returns md5sum for a message object of type '<move_robot-response>"
  "533042a64ee768c9dd424ef588119ef4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_robot-response)))
  "Returns md5sum for a message object of type 'move_robot-response"
  "533042a64ee768c9dd424ef588119ef4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_robot-response>)))
  "Returns full string definition for message of type '<move_robot-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_robot-response)))
  "Returns full string definition for message of type 'move_robot-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_robot-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_robot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'move_robot-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'move_robot)))
  'move_robot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'move_robot)))
  'move_robot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_robot)))
  "Returns string type for a service object of type '<move_robot>"
  "ptam/move_robot")