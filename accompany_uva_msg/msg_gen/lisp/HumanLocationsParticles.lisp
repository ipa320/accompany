; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude HumanLocationsParticles.msg.html

(cl:defclass <HumanLocationsParticles> (roslisp-msg-protocol:ros-message)
  ((particles
    :reader particles
    :initarg :particles
    :type (cl:vector accompany_uva_msg-msg:HumanLocationsParticle)
   :initform (cl:make-array 0 :element-type 'accompany_uva_msg-msg:HumanLocationsParticle :initial-element (cl:make-instance 'accompany_uva_msg-msg:HumanLocationsParticle))))
)

(cl:defclass HumanLocationsParticles (<HumanLocationsParticles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanLocationsParticles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanLocationsParticles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<HumanLocationsParticles> is deprecated: use accompany_uva_msg-msg:HumanLocationsParticles instead.")))

(cl:ensure-generic-function 'particles-val :lambda-list '(m))
(cl:defmethod particles-val ((m <HumanLocationsParticles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:particles-val is deprecated.  Use accompany_uva_msg-msg:particles instead.")
  (particles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanLocationsParticles>) ostream)
  "Serializes a message object of type '<HumanLocationsParticles>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'particles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'particles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanLocationsParticles>) istream)
  "Deserializes a message object of type '<HumanLocationsParticles>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'particles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'particles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'accompany_uva_msg-msg:HumanLocationsParticle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanLocationsParticles>)))
  "Returns string type for a message object of type '<HumanLocationsParticles>"
  "accompany_uva_msg/HumanLocationsParticles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanLocationsParticles)))
  "Returns string type for a message object of type 'HumanLocationsParticles"
  "accompany_uva_msg/HumanLocationsParticles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanLocationsParticles>)))
  "Returns md5sum for a message object of type '<HumanLocationsParticles>"
  "6e27cbdc9a5378b383474fed9b399ba3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanLocationsParticles)))
  "Returns md5sum for a message object of type 'HumanLocationsParticles"
  "6e27cbdc9a5378b383474fed9b399ba3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanLocationsParticles>)))
  "Returns full string definition for message of type '<HumanLocationsParticles>"
  (cl:format cl:nil "accompany_uva_msg/HumanLocationsParticle[] particles~%~%================================================================================~%MSG: accompany_uva_msg/HumanLocationsParticle~%geometry_msgs/PointStamped[] locations~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanLocationsParticles)))
  "Returns full string definition for message of type 'HumanLocationsParticles"
  (cl:format cl:nil "accompany_uva_msg/HumanLocationsParticle[] particles~%~%================================================================================~%MSG: accompany_uva_msg/HumanLocationsParticle~%geometry_msgs/PointStamped[] locations~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanLocationsParticles>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'particles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanLocationsParticles>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanLocationsParticles
    (cl:cons ':particles (particles msg))
))
