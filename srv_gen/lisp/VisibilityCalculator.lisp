; Auto-generated. Do not edit!


(cl:in-package ptam-srv)


;//! \htmlinclude VisibilityCalculator-request.msg.html

(cl:defclass <VisibilityCalculator-request> (roslisp-msg-protocol:ros-message)
  ((ugv_frontiers
    :reader ugv_frontiers
    :initarg :ugv_frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray)))
)

(cl:defclass VisibilityCalculator-request (<VisibilityCalculator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisibilityCalculator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisibilityCalculator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<VisibilityCalculator-request> is deprecated: use ptam-srv:VisibilityCalculator-request instead.")))

(cl:ensure-generic-function 'ugv_frontiers-val :lambda-list '(m))
(cl:defmethod ugv_frontiers-val ((m <VisibilityCalculator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:ugv_frontiers-val is deprecated.  Use ptam-srv:ugv_frontiers instead.")
  (ugv_frontiers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisibilityCalculator-request>) ostream)
  "Serializes a message object of type '<VisibilityCalculator-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ugv_frontiers) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisibilityCalculator-request>) istream)
  "Deserializes a message object of type '<VisibilityCalculator-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ugv_frontiers) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisibilityCalculator-request>)))
  "Returns string type for a service object of type '<VisibilityCalculator-request>"
  "ptam/VisibilityCalculatorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisibilityCalculator-request)))
  "Returns string type for a service object of type 'VisibilityCalculator-request"
  "ptam/VisibilityCalculatorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisibilityCalculator-request>)))
  "Returns md5sum for a message object of type '<VisibilityCalculator-request>"
  "3a44e6eb897dd41c3de9eb553ef7c91f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisibilityCalculator-request)))
  "Returns md5sum for a message object of type 'VisibilityCalculator-request"
  "3a44e6eb897dd41c3de9eb553ef7c91f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisibilityCalculator-request>)))
  "Returns full string definition for message of type '<VisibilityCalculator-request>"
  (cl:format cl:nil "geometry_msgs/PoseArray ugv_frontiers~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisibilityCalculator-request)))
  "Returns full string definition for message of type 'VisibilityCalculator-request"
  (cl:format cl:nil "geometry_msgs/PoseArray ugv_frontiers~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisibilityCalculator-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ugv_frontiers))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisibilityCalculator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VisibilityCalculator-request
    (cl:cons ':ugv_frontiers (ugv_frontiers msg))
))
;//! \htmlinclude VisibilityCalculator-response.msg.html

(cl:defclass <VisibilityCalculator-response> (roslisp-msg-protocol:ros-message)
  ((ugv_frontier_visibility
    :reader ugv_frontier_visibility
    :initarg :ugv_frontier_visibility
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass VisibilityCalculator-response (<VisibilityCalculator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisibilityCalculator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisibilityCalculator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<VisibilityCalculator-response> is deprecated: use ptam-srv:VisibilityCalculator-response instead.")))

(cl:ensure-generic-function 'ugv_frontier_visibility-val :lambda-list '(m))
(cl:defmethod ugv_frontier_visibility-val ((m <VisibilityCalculator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:ugv_frontier_visibility-val is deprecated.  Use ptam-srv:ugv_frontier_visibility instead.")
  (ugv_frontier_visibility m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisibilityCalculator-response>) ostream)
  "Serializes a message object of type '<VisibilityCalculator-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ugv_frontier_visibility))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'ugv_frontier_visibility))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisibilityCalculator-response>) istream)
  "Deserializes a message object of type '<VisibilityCalculator-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ugv_frontier_visibility) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ugv_frontier_visibility)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisibilityCalculator-response>)))
  "Returns string type for a service object of type '<VisibilityCalculator-response>"
  "ptam/VisibilityCalculatorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisibilityCalculator-response)))
  "Returns string type for a service object of type 'VisibilityCalculator-response"
  "ptam/VisibilityCalculatorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisibilityCalculator-response>)))
  "Returns md5sum for a message object of type '<VisibilityCalculator-response>"
  "3a44e6eb897dd41c3de9eb553ef7c91f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisibilityCalculator-response)))
  "Returns md5sum for a message object of type 'VisibilityCalculator-response"
  "3a44e6eb897dd41c3de9eb553ef7c91f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisibilityCalculator-response>)))
  "Returns full string definition for message of type '<VisibilityCalculator-response>"
  (cl:format cl:nil "float32[] ugv_frontier_visibility~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisibilityCalculator-response)))
  "Returns full string definition for message of type 'VisibilityCalculator-response"
  (cl:format cl:nil "float32[] ugv_frontier_visibility~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisibilityCalculator-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ugv_frontier_visibility) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisibilityCalculator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VisibilityCalculator-response
    (cl:cons ':ugv_frontier_visibility (ugv_frontier_visibility msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VisibilityCalculator)))
  'VisibilityCalculator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VisibilityCalculator)))
  'VisibilityCalculator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisibilityCalculator)))
  "Returns string type for a service object of type '<VisibilityCalculator>"
  "ptam/VisibilityCalculator")