; Auto-generated. Do not edit!


(cl:in-package ptam-srv)


;//! \htmlinclude cluster-request.msg.html

(cl:defclass <cluster-request> (roslisp-msg-protocol:ros-message)
  ((frontiers
    :reader frontiers
    :initarg :frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (threshold
    :reader threshold
    :initarg :threshold
    :type cl:float
    :initform 0.0))
)

(cl:defclass cluster-request (<cluster-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cluster-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cluster-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<cluster-request> is deprecated: use ptam-srv:cluster-request instead.")))

(cl:ensure-generic-function 'frontiers-val :lambda-list '(m))
(cl:defmethod frontiers-val ((m <cluster-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:frontiers-val is deprecated.  Use ptam-srv:frontiers instead.")
  (frontiers m))

(cl:ensure-generic-function 'threshold-val :lambda-list '(m))
(cl:defmethod threshold-val ((m <cluster-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:threshold-val is deprecated.  Use ptam-srv:threshold instead.")
  (threshold m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cluster-request>) ostream)
  "Serializes a message object of type '<cluster-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'frontiers) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cluster-request>) istream)
  "Deserializes a message object of type '<cluster-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'frontiers) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cluster-request>)))
  "Returns string type for a service object of type '<cluster-request>"
  "ptam/clusterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cluster-request)))
  "Returns string type for a service object of type 'cluster-request"
  "ptam/clusterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cluster-request>)))
  "Returns md5sum for a message object of type '<cluster-request>"
  "84e0be09b741237cbf3f2e85a040f8f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cluster-request)))
  "Returns md5sum for a message object of type 'cluster-request"
  "84e0be09b741237cbf3f2e85a040f8f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cluster-request>)))
  "Returns full string definition for message of type '<cluster-request>"
  (cl:format cl:nil "geometry_msgs/PoseArray frontiers~%float32 threshold~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cluster-request)))
  "Returns full string definition for message of type 'cluster-request"
  (cl:format cl:nil "geometry_msgs/PoseArray frontiers~%float32 threshold~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cluster-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'frontiers))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cluster-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cluster-request
    (cl:cons ':frontiers (frontiers msg))
    (cl:cons ':threshold (threshold msg))
))
;//! \htmlinclude cluster-response.msg.html

(cl:defclass <cluster-response> (roslisp-msg-protocol:ros-message)
  ((cluster_center
    :reader cluster_center
    :initarg :cluster_center
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (clustered_frontiers
    :reader clustered_frontiers
    :initarg :clustered_frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (point_info
    :reader point_info
    :initarg :point_info
    :type (cl:vector ptam-msg:point)
   :initform (cl:make-array 0 :element-type 'ptam-msg:point :initial-element (cl:make-instance 'ptam-msg:point))))
)

(cl:defclass cluster-response (<cluster-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cluster-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cluster-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<cluster-response> is deprecated: use ptam-srv:cluster-response instead.")))

(cl:ensure-generic-function 'cluster_center-val :lambda-list '(m))
(cl:defmethod cluster_center-val ((m <cluster-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:cluster_center-val is deprecated.  Use ptam-srv:cluster_center instead.")
  (cluster_center m))

(cl:ensure-generic-function 'clustered_frontiers-val :lambda-list '(m))
(cl:defmethod clustered_frontiers-val ((m <cluster-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:clustered_frontiers-val is deprecated.  Use ptam-srv:clustered_frontiers instead.")
  (clustered_frontiers m))

(cl:ensure-generic-function 'point_info-val :lambda-list '(m))
(cl:defmethod point_info-val ((m <cluster-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:point_info-val is deprecated.  Use ptam-srv:point_info instead.")
  (point_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cluster-response>) ostream)
  "Serializes a message object of type '<cluster-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cluster_center))))
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
   (cl:slot-value msg 'cluster_center))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'clustered_frontiers) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'point_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cluster-response>) istream)
  "Deserializes a message object of type '<cluster-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cluster_center) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cluster_center)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'clustered_frontiers) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ptam-msg:point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cluster-response>)))
  "Returns string type for a service object of type '<cluster-response>"
  "ptam/clusterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cluster-response)))
  "Returns string type for a service object of type 'cluster-response"
  "ptam/clusterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cluster-response>)))
  "Returns md5sum for a message object of type '<cluster-response>"
  "84e0be09b741237cbf3f2e85a040f8f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cluster-response)))
  "Returns md5sum for a message object of type 'cluster-response"
  "84e0be09b741237cbf3f2e85a040f8f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cluster-response>)))
  "Returns full string definition for message of type '<cluster-response>"
  (cl:format cl:nil "int32[] cluster_center~%geometry_msgs/PoseArray clustered_frontiers~%point[] point_info~%~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: ptam/point~%int32 c_i~%float32 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cluster-response)))
  "Returns full string definition for message of type 'cluster-response"
  (cl:format cl:nil "int32[] cluster_center~%geometry_msgs/PoseArray clustered_frontiers~%point[] point_info~%~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: ptam/point~%int32 c_i~%float32 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cluster-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cluster_center) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'clustered_frontiers))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cluster-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cluster-response
    (cl:cons ':cluster_center (cluster_center msg))
    (cl:cons ':clustered_frontiers (clustered_frontiers msg))
    (cl:cons ':point_info (point_info msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cluster)))
  'cluster-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cluster)))
  'cluster-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cluster)))
  "Returns string type for a service object of type '<cluster>"
  "ptam/cluster")