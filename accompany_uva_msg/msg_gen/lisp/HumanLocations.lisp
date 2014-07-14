; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude HumanLocations.msg.html

(cl:defclass <HumanLocations> (roslisp-msg-protocol:ros-message)
  ((locations
    :reader locations
    :initarg :locations
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped))))
)

(cl:defclass HumanLocations (<HumanLocations>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanLocations>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanLocations)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<HumanLocations> is deprecated: use accompany_uva_msg-msg:HumanLocations instead.")))

(cl:ensure-generic-function 'locations-val :lambda-list '(m))
(cl:defmethod locations-val ((m <HumanLocations>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:locations-val is deprecated.  Use accompany_uva_msg-msg:locations instead.")
  (locations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanLocations>) ostream)
  "Serializes a message object of type '<HumanLocations>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'locations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'locations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanLocations>) istream)
  "Deserializes a message object of type '<HumanLocations>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanLocations>)))
  "Returns string type for a message object of type '<HumanLocations>"
  "accompany_uva_msg/HumanLocations")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanLocations)))
  "Returns string type for a message object of type 'HumanLocations"
  "accompany_uva_msg/HumanLocations")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanLocations>)))
  "Returns md5sum for a message object of type '<HumanLocations>"
  "048149b61e4930bb4c889d2870736e2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanLocations)))
  "Returns md5sum for a message object of type 'HumanLocations"
  "048149b61e4930bb4c889d2870736e2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanLocations>)))
  "Returns full string definition for message of type '<HumanLocations>"
  (cl:format cl:nil "geometry_msgs/PointStamped[] locations~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanLocations)))
  "Returns full string definition for message of type 'HumanLocations"
  (cl:format cl:nil "geometry_msgs/PointStamped[] locations~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanLocations>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'locations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanLocations>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanLocations
    (cl:cons ':locations (locations msg))
))
