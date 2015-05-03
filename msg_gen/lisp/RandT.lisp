; Auto-generated. Do not edit!


(cl:in-package ptam-msg)


;//! \htmlinclude RandT.msg.html

(cl:defclass <RandT> (roslisp-msg-protocol:ros-message)
  ((translation
    :reader translation
    :initarg :translation
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (rotation
    :reader rotation
    :initarg :rotation
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RandT (<RandT>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RandT>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RandT)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptam-msg:<RandT> is deprecated: use ptam-msg:RandT instead.")))

(cl:ensure-generic-function 'translation-val :lambda-list '(m))
(cl:defmethod translation-val ((m <RandT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:translation-val is deprecated.  Use ptam-msg:translation instead.")
  (translation m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <RandT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptam-msg:rotation-val is deprecated.  Use ptam-msg:rotation instead.")
  (rotation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RandT>) ostream)
  "Serializes a message object of type '<RandT>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'translation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'translation))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'rotation))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RandT>) istream)
  "Deserializes a message object of type '<RandT>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'translation) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'translation)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rotation) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rotation)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RandT>)))
  "Returns string type for a message object of type '<RandT>"
  "ptam/RandT")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RandT)))
  "Returns string type for a message object of type 'RandT"
  "ptam/RandT")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RandT>)))
  "Returns md5sum for a message object of type '<RandT>"
  "a5b80608aa0b7f5b05ca5427bd9f1fd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RandT)))
  "Returns md5sum for a message object of type 'RandT"
  "a5b80608aa0b7f5b05ca5427bd9f1fd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RandT>)))
  "Returns full string definition for message of type '<RandT>"
  (cl:format cl:nil "float32[] translation~%float32[] rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RandT)))
  "Returns full string definition for message of type 'RandT"
  (cl:format cl:nil "float32[] translation~%float32[] rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RandT>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'translation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rotation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RandT>))
  "Converts a ROS message object to a list"
  (cl:list 'RandT
    (cl:cons ':translation (translation msg))
    (cl:cons ':rotation (rotation msg))
))
