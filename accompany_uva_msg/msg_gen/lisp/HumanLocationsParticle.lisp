; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude HumanLocationsParticle.msg.html

(cl:defclass <HumanLocationsParticle> (roslisp-msg-protocol:ros-message)
  ((locations
    :reader locations
    :initarg :locations
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass HumanLocationsParticle (<HumanLocationsParticle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanLocationsParticle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanLocationsParticle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<HumanLocationsParticle> is deprecated: use accompany_uva_msg-msg:HumanLocationsParticle instead.")))

(cl:ensure-generic-function 'locations-val :lambda-list '(m))
(cl:defmethod locations-val ((m <HumanLocationsParticle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:locations-val is deprecated.  Use accompany_uva_msg-msg:locations instead.")
  (locations m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <HumanLocationsParticle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:weight-val is deprecated.  Use accompany_uva_msg-msg:weight instead.")
  (weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanLocationsParticle>) ostream)
  "Serializes a message object of type '<HumanLocationsParticle>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'locations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'locations))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanLocationsParticle>) istream)
  "Deserializes a message object of type '<HumanLocationsParticle>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'locations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'locations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanLocationsParticle>)))
  "Returns string type for a message object of type '<HumanLocationsParticle>"
  "accompany_uva_msg/HumanLocationsParticle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanLocationsParticle)))
  "Returns string type for a message object of type 'HumanLocationsParticle"
  "accompany_uva_msg/HumanLocationsParticle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanLocationsParticle>)))
  "Returns md5sum for a message object of type '<HumanLocationsParticle>"
  "349a944ee281c735fe6b84bd25a7f2b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanLocationsParticle)))
  "Returns md5sum for a message object of type 'HumanLocationsParticle"
  "349a944ee281c735fe6b84bd25a7f2b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanLocationsParticle>)))
  "Returns full string definition for message of type '<HumanLocationsParticle>"
  (cl:format cl:nil "geometry_msgs/PointStamped[] locations~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanLocationsParticle)))
  "Returns full string definition for message of type 'HumanLocationsParticle"
  (cl:format cl:nil "geometry_msgs/PointStamped[] locations~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanLocationsParticle>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'locations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanLocationsParticle>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanLocationsParticle
    (cl:cons ':locations (locations msg))
    (cl:cons ':weight (weight msg))
))
