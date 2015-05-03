; Auto-generated. Do not edit!


(cl:in-package ptam-msg)


;//! \htmlinclude map_info.msg.html

(cl:defclass <map_info> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass map_info (<map_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <map_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'map_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-msg:<map_info> is deprecated: use ptam-msg:map_info instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <map_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:data-val is deprecated.  Use ptam-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <map_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:index-val is deprecated.  Use ptam-msg:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <map_info>) ostream)
  "Serializes a message object of type '<map_info>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <map_info>) istream)
  "Deserializes a message object of type '<map_info>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<map_info>)))
  "Returns string type for a message object of type '<map_info>"
  "ptam/map_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'map_info)))
  "Returns string type for a message object of type 'map_info"
  "ptam/map_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<map_info>)))
  "Returns md5sum for a message object of type '<map_info>"
  "f14d254c9aa1e06009f4a43da7832db0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'map_info)))
  "Returns md5sum for a message object of type 'map_info"
  "f14d254c9aa1e06009f4a43da7832db0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<map_info>)))
  "Returns full string definition for message of type '<map_info>"
  (cl:format cl:nil "int8[] data~%int8 index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'map_info)))
  "Returns full string definition for message of type 'map_info"
  (cl:format cl:nil "int8[] data~%int8 index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <map_info>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <map_info>))
  "Converts a ROS message object to a list"
  (cl:list 'map_info
    (cl:cons ':data (data msg))
    (cl:cons ':index (index msg))
))
