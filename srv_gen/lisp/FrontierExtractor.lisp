; Auto-generated. Do not edit!


(cl:in-package ptam-srv)


;//! \htmlinclude FrontierExtractor-request.msg.html

(cl:defclass <FrontierExtractor-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FrontierExtractor-request (<FrontierExtractor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrontierExtractor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrontierExtractor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<FrontierExtractor-request> is deprecated: use ptam-srv:FrontierExtractor-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrontierExtractor-request>) ostream)
  "Serializes a message object of type '<FrontierExtractor-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrontierExtractor-request>) istream)
  "Deserializes a message object of type '<FrontierExtractor-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrontierExtractor-request>)))
  "Returns string type for a service object of type '<FrontierExtractor-request>"
  "ptam/FrontierExtractorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrontierExtractor-request)))
  "Returns string type for a service object of type 'FrontierExtractor-request"
  "ptam/FrontierExtractorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrontierExtractor-request>)))
  "Returns md5sum for a message object of type '<FrontierExtractor-request>"
  "3d9ac009080ac5dccf76d53c837f311b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrontierExtractor-request)))
  "Returns md5sum for a message object of type 'FrontierExtractor-request"
  "3d9ac009080ac5dccf76d53c837f311b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrontierExtractor-request>)))
  "Returns full string definition for message of type '<FrontierExtractor-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrontierExtractor-request)))
  "Returns full string definition for message of type 'FrontierExtractor-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrontierExtractor-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrontierExtractor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FrontierExtractor-request
))
;//! \htmlinclude FrontierExtractor-response.msg.html

(cl:defclass <FrontierExtractor-response> (roslisp-msg-protocol:ros-message)
  ((ugv_frontiers
    :reader ugv_frontiers
    :initarg :ugv_frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (uav_frontiers
    :reader uav_frontiers
    :initarg :uav_frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (passive_uav_frontiers
    :reader passive_uav_frontiers
    :initarg :passive_uav_frontiers
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray)))
)

(cl:defclass FrontierExtractor-response (<FrontierExtractor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrontierExtractor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrontierExtractor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-srv:<FrontierExtractor-response> is deprecated: use ptam-srv:FrontierExtractor-response instead.")))

(cl:ensure-generic-function 'ugv_frontiers-val :lambda-list '(m))
(cl:defmethod ugv_frontiers-val ((m <FrontierExtractor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:ugv_frontiers-val is deprecated.  Use ptam-srv:ugv_frontiers instead.")
  (ugv_frontiers m))

(cl:ensure-generic-function 'uav_frontiers-val :lambda-list '(m))
(cl:defmethod uav_frontiers-val ((m <FrontierExtractor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:uav_frontiers-val is deprecated.  Use ptam-srv:uav_frontiers instead.")
  (uav_frontiers m))

(cl:ensure-generic-function 'passive_uav_frontiers-val :lambda-list '(m))
(cl:defmethod passive_uav_frontiers-val ((m <FrontierExtractor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-srv:passive_uav_frontiers-val is deprecated.  Use ptam-srv:passive_uav_frontiers instead.")
  (passive_uav_frontiers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrontierExtractor-response>) ostream)
  "Serializes a message object of type '<FrontierExtractor-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ugv_frontiers) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'uav_frontiers) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'passive_uav_frontiers) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrontierExtractor-response>) istream)
  "Deserializes a message object of type '<FrontierExtractor-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ugv_frontiers) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'uav_frontiers) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'passive_uav_frontiers) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrontierExtractor-response>)))
  "Returns string type for a service object of type '<FrontierExtractor-response>"
  "ptam/FrontierExtractorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrontierExtractor-response)))
  "Returns string type for a service object of type 'FrontierExtractor-response"
  "ptam/FrontierExtractorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrontierExtractor-response>)))
  "Returns md5sum for a message object of type '<FrontierExtractor-response>"
  "3d9ac009080ac5dccf76d53c837f311b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrontierExtractor-response)))
  "Returns md5sum for a message object of type 'FrontierExtractor-response"
  "3d9ac009080ac5dccf76d53c837f311b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrontierExtractor-response>)))
  "Returns full string definition for message of type '<FrontierExtractor-response>"
  (cl:format cl:nil "geometry_msgs/PoseArray ugv_frontiers~%geometry_msgs/PoseArray uav_frontiers~%geometry_msgs/PoseArray passive_uav_frontiers~%~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrontierExtractor-response)))
  "Returns full string definition for message of type 'FrontierExtractor-response"
  (cl:format cl:nil "geometry_msgs/PoseArray ugv_frontiers~%geometry_msgs/PoseArray uav_frontiers~%geometry_msgs/PoseArray passive_uav_frontiers~%~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrontierExtractor-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ugv_frontiers))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'uav_frontiers))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'passive_uav_frontiers))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrontierExtractor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FrontierExtractor-response
    (cl:cons ':ugv_frontiers (ugv_frontiers msg))
    (cl:cons ':uav_frontiers (uav_frontiers msg))
    (cl:cons ':passive_uav_frontiers (passive_uav_frontiers msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FrontierExtractor)))
  'FrontierExtractor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FrontierExtractor)))
  'FrontierExtractor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrontierExtractor)))
  "Returns string type for a service object of type '<FrontierExtractor>"
  "ptam/FrontierExtractor")