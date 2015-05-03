; Auto-generated. Do not edit!


(cl:in-package ptam-msg)


;//! \htmlinclude points_visible.msg.html

(cl:defclass <points_visible> (roslisp-msg-protocol:ros-message)
  ((points_visible
    :reader points_visible
    :initarg :points_visible
    :type cl:integer
    :initform 0))
)

(cl:defclass points_visible (<points_visible>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <points_visible>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'points_visible)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-msg:<points_visible> is deprecated: use ptam-msg:points_visible instead.")))

(cl:ensure-generic-function 'points_visible-val :lambda-list '(m))
(cl:defmethod points_visible-val ((m <points_visible>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:points_visible-val is deprecated.  Use ptam-msg:points_visible instead.")
  (points_visible m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <points_visible>) ostream)
  "Serializes a message object of type '<points_visible>"
  (cl:let* ((signed (cl:slot-value msg 'points_visible)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <points_visible>) istream)
  "Deserializes a message object of type '<points_visible>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'points_visible) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<points_visible>)))
  "Returns string type for a message object of type '<points_visible>"
  "ptam/points_visible")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'points_visible)))
  "Returns string type for a message object of type 'points_visible"
  "ptam/points_visible")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<points_visible>)))
  "Returns md5sum for a message object of type '<points_visible>"
  "42add71f8f440e7303bc4883921935ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'points_visible)))
  "Returns md5sum for a message object of type 'points_visible"
  "42add71f8f440e7303bc4883921935ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<points_visible>)))
  "Returns full string definition for message of type '<points_visible>"
  (cl:format cl:nil "int32 points_visible~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'points_visible)))
  "Returns full string definition for message of type 'points_visible"
  (cl:format cl:nil "int32 points_visible~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <points_visible>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <points_visible>))
  "Converts a ROS message object to a list"
  (cl:list 'points_visible
    (cl:cons ':points_visible (points_visible msg))
))
