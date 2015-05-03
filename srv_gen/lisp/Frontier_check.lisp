; Auto-generated. Do not edit!


(cl:in-package ptam-srv)


;//! \htmlinclude Frontier_check-request.msg.html

(cl:defclass <Frontier_check-request> (roslisp-msg-protocol:ros-message)
  ((Frontiers
    :reader Frontiers
    :initarg :Frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray)))
)

(cl:defclass Frontier_check-request (<Frontier_check-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Frontier_check-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Frontier_check-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<Frontier_check-request> is deprecated: use ptam-srv:Frontier_check-request instead.")))

(cl:ensure-generic-function 'Frontiers-val :lambda-list '(m))
(cl:defmethod Frontiers-val ((m <Frontier_check-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:Frontiers-val is deprecated.  Use ptam-srv:Frontiers instead.")
  (Frontiers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Frontier_check-request>) ostream)
  "Serializes a message object of type '<Frontier_check-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Frontiers) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Frontier_check-request>) istream)
  "Deserializes a message object of type '<Frontier_check-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Frontiers) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Frontier_check-request>)))
  "Returns string type for a service object of type '<Frontier_check-request>"
  "ptam/Frontier_checkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Frontier_check-request)))
  "Returns string type for a service object of type 'Frontier_check-request"
  "ptam/Frontier_checkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Frontier_check-request>)))
  "Returns md5sum for a message object of type '<Frontier_check-request>"
  "a52d0c0173389c6c5ef2ae39380ff751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Frontier_check-request)))
  "Returns md5sum for a message object of type 'Frontier_check-request"
  "a52d0c0173389c6c5ef2ae39380ff751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Frontier_check-request>)))
  "Returns full string definition for message of type '<Frontier_check-request>"
  (cl:format cl:nil "geometry_msgs/PoseArray Frontiers~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Frontier_check-request)))
  "Returns full string definition for message of type 'Frontier_check-request"
  (cl:format cl:nil "geometry_msgs/PoseArray Frontiers~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Frontier_check-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Frontiers))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Frontier_check-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Frontier_check-request
    (cl:cons ':Frontiers (Frontiers msg))
))
;//! \htmlinclude Frontier_check-response.msg.html

(cl:defclass <Frontier_check-response> (roslisp-msg-protocol:ros-message)
  ((total_points
    :reader total_points
    :initarg :total_points
    :type cl:integer
    :initform 0)
   (points_visible
    :reader points_visible
    :initarg :points_visible
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Frontier_check-response (<Frontier_check-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Frontier_check-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Frontier_check-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<Frontier_check-response> is deprecated: use ptam-srv:Frontier_check-response instead.")))

(cl:ensure-generic-function 'total_points-val :lambda-list '(m))
(cl:defmethod total_points-val ((m <Frontier_check-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:total_points-val is deprecated.  Use ptam-srv:total_points instead.")
  (total_points m))

(cl:ensure-generic-function 'points_visible-val :lambda-list '(m))
(cl:defmethod points_visible-val ((m <Frontier_check-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:points_visible-val is deprecated.  Use ptam-srv:points_visible instead.")
  (points_visible m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Frontier_check-response>) ostream)
  "Serializes a message object of type '<Frontier_check-response>"
  (cl:let* ((signed (cl:slot-value msg 'total_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points_visible))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'points_visible))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Frontier_check-response>) istream)
  "Deserializes a message object of type '<Frontier_check-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points_visible) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points_visible)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Frontier_check-response>)))
  "Returns string type for a service object of type '<Frontier_check-response>"
  "ptam/Frontier_checkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Frontier_check-response)))
  "Returns string type for a service object of type 'Frontier_check-response"
  "ptam/Frontier_checkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Frontier_check-response>)))
  "Returns md5sum for a message object of type '<Frontier_check-response>"
  "a52d0c0173389c6c5ef2ae39380ff751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Frontier_check-response)))
  "Returns md5sum for a message object of type 'Frontier_check-response"
  "a52d0c0173389c6c5ef2ae39380ff751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Frontier_check-response>)))
  "Returns full string definition for message of type '<Frontier_check-response>"
  (cl:format cl:nil "int32 total_points~%int32[] points_visible~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Frontier_check-response)))
  "Returns full string definition for message of type 'Frontier_check-response"
  (cl:format cl:nil "int32 total_points~%int32[] points_visible~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Frontier_check-response>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points_visible) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Frontier_check-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Frontier_check-response
    (cl:cons ':total_points (total_points msg))
    (cl:cons ':points_visible (points_visible msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Frontier_check)))
  'Frontier_check-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Frontier_check)))
  'Frontier_check-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Frontier_check)))
  "Returns string type for a service object of type '<Frontier_check>"
  "ptam/Frontier_check")