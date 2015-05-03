; Auto-generated. Do not edit!


(cl:in-package ptam-srv)


;//! \htmlinclude trajectory_checker-request.msg.html

(cl:defclass <trajectory_checker-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass trajectory_checker-request (<trajectory_checker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_checker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_checker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<trajectory_checker-request> is deprecated: use ptam-srv:trajectory_checker-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_checker-request>) ostream)
  "Serializes a message object of type '<trajectory_checker-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_checker-request>) istream)
  "Deserializes a message object of type '<trajectory_checker-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_checker-request>)))
  "Returns string type for a service object of type '<trajectory_checker-request>"
  "ptam/trajectory_checkerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_checker-request)))
  "Returns string type for a service object of type 'trajectory_checker-request"
  "ptam/trajectory_checkerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_checker-request>)))
  "Returns md5sum for a message object of type '<trajectory_checker-request>"
  "c9ec9792357c4b84a3c9a32c04bf74ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_checker-request)))
  "Returns md5sum for a message object of type 'trajectory_checker-request"
  "c9ec9792357c4b84a3c9a32c04bf74ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_checker-request>)))
  "Returns full string definition for message of type '<trajectory_checker-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_checker-request)))
  "Returns full string definition for message of type 'trajectory_checker-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_checker-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_checker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_checker-request
))
;//! \htmlinclude trajectory_checker-response.msg.html

(cl:defclass <trajectory_checker-response> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type cl:integer
    :initform 0)
   (possible
    :reader possible
    :initarg :possible
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass trajectory_checker-response (<trajectory_checker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_checker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_checker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<trajectory_checker-response> is deprecated: use ptam-srv:trajectory_checker-response instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <trajectory_checker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:points-val is deprecated.  Use ptam-srv:points instead.")
  (points m))

(cl:ensure-generic-function 'possible-val :lambda-list '(m))
(cl:defmethod possible-val ((m <trajectory_checker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:possible-val is deprecated.  Use ptam-srv:possible instead.")
  (possible m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_checker-response>) ostream)
  "Serializes a message object of type '<trajectory_checker-response>"
  (cl:let* ((signed (cl:slot-value msg 'points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'possible) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_checker-response>) istream)
  "Deserializes a message object of type '<trajectory_checker-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'possible) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_checker-response>)))
  "Returns string type for a service object of type '<trajectory_checker-response>"
  "ptam/trajectory_checkerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_checker-response)))
  "Returns string type for a service object of type 'trajectory_checker-response"
  "ptam/trajectory_checkerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_checker-response>)))
  "Returns md5sum for a message object of type '<trajectory_checker-response>"
  "c9ec9792357c4b84a3c9a32c04bf74ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_checker-response)))
  "Returns md5sum for a message object of type 'trajectory_checker-response"
  "c9ec9792357c4b84a3c9a32c04bf74ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_checker-response>)))
  "Returns full string definition for message of type '<trajectory_checker-response>"
  (cl:format cl:nil "int32 points~%bool possible~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_checker-response)))
  "Returns full string definition for message of type 'trajectory_checker-response"
  (cl:format cl:nil "int32 points~%bool possible~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_checker-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_checker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_checker-response
    (cl:cons ':points (points msg))
    (cl:cons ':possible (possible msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'trajectory_checker)))
  'trajectory_checker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'trajectory_checker)))
  'trajectory_checker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_checker)))
  "Returns string type for a service object of type '<trajectory_checker>"
  "ptam/trajectory_checker")