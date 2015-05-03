; Auto-generated. Do not edit!


(cl:in-package ptam-msg)


;//! \htmlinclude point.msg.html

(cl:defclass <point> (roslisp-msg-protocol:ros-message)
  ((c_i
    :reader c_i
    :initarg :c_i
    :type cl:integer
    :initform 0)
   (dist
    :reader dist
    :initarg :dist
    :type cl:float
    :initform 0.0))
)

(cl:defclass point (<point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-msg:<point> is deprecated: use ptam-msg:point instead.")))

(cl:ensure-generic-function 'c_i-val :lambda-list '(m))
(cl:defmethod c_i-val ((m <point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:c_i-val is deprecated.  Use ptam-msg:c_i instead.")
  (c_i m))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:dist-val is deprecated.  Use ptam-msg:dist instead.")
  (dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <point>) ostream)
  "Serializes a message object of type '<point>"
  (cl:let* ((signed (cl:slot-value msg 'c_i)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <point>) istream)
  "Deserializes a message object of type '<point>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'c_i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<point>)))
  "Returns string type for a message object of type '<point>"
  "ptam/point")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'point)))
  "Returns string type for a message object of type 'point"
  "ptam/point")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<point>)))
  "Returns md5sum for a message object of type '<point>"
  "8d908333811b10088914f4a926f9411e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'point)))
  "Returns md5sum for a message object of type 'point"
  "8d908333811b10088914f4a926f9411e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<point>)))
  "Returns full string definition for message of type '<point>"
  (cl:format cl:nil "int32 c_i~%float32 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'point)))
  "Returns full string definition for message of type 'point"
  (cl:format cl:nil "int32 c_i~%float32 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <point>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <point>))
  "Converts a ROS message object to a list"
  (cl:list 'point
    (cl:cons ':c_i (c_i msg))
    (cl:cons ':dist (dist msg))
))
