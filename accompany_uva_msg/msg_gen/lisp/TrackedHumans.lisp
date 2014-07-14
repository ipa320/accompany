; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude TrackedHumans.msg.html

(cl:defclass <TrackedHumans> (roslisp-msg-protocol:ros-message)
  ((trackedHumans
    :reader trackedHumans
    :initarg :trackedHumans
    :type (cl:vector accompany_uva_msg-msg:TrackedHuman)
   :initform (cl:make-array 0 :element-type 'accompany_uva_msg-msg:TrackedHuman :initial-element (cl:make-instance 'accompany_uva_msg-msg:TrackedHuman))))
)

(cl:defclass TrackedHumans (<TrackedHumans>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedHumans>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedHumans)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<TrackedHumans> is deprecated: use accompany_uva_msg-msg:TrackedHumans instead.")))

(cl:ensure-generic-function 'trackedHumans-val :lambda-list '(m))
(cl:defmethod trackedHumans-val ((m <TrackedHumans>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:trackedHumans-val is deprecated.  Use accompany_uva_msg-msg:trackedHumans instead.")
  (trackedHumans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedHumans>) ostream)
  "Serializes a message object of type '<TrackedHumans>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trackedHumans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trackedHumans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedHumans>) istream)
  "Deserializes a message object of type '<TrackedHumans>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trackedHumans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trackedHumans)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'accompany_uva_msg-msg:TrackedHuman))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedHumans>)))
  "Returns string type for a message object of type '<TrackedHumans>"
  "accompany_uva_msg/TrackedHumans")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedHumans)))
  "Returns string type for a message object of type 'TrackedHumans"
  "accompany_uva_msg/TrackedHumans")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedHumans>)))
  "Returns md5sum for a message object of type '<TrackedHumans>"
  "a6569de3725dc368e6c9805c323491a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedHumans)))
  "Returns md5sum for a message object of type 'TrackedHumans"
  "a6569de3725dc368e6c9805c323491a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedHumans>)))
  "Returns full string definition for message of type '<TrackedHumans>"
  (cl:format cl:nil "accompany_uva_msg/TrackedHuman[] trackedHumans~%~%================================================================================~%MSG: accompany_uva_msg/TrackedHuman~%geometry_msgs/PointStamped location~%geometry_msgs/Vector3Stamped speed~%int32 id~%string identity~%time firstSeen~%time lastSeen~%int32 specialFlag~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3Stamped~%# This represents a Vector3 with reference coordinate frame and timestamp~%Header header~%Vector3 vector~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedHumans)))
  "Returns full string definition for message of type 'TrackedHumans"
  (cl:format cl:nil "accompany_uva_msg/TrackedHuman[] trackedHumans~%~%================================================================================~%MSG: accompany_uva_msg/TrackedHuman~%geometry_msgs/PointStamped location~%geometry_msgs/Vector3Stamped speed~%int32 id~%string identity~%time firstSeen~%time lastSeen~%int32 specialFlag~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3Stamped~%# This represents a Vector3 with reference coordinate frame and timestamp~%Header header~%Vector3 vector~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedHumans>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trackedHumans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedHumans>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedHumans
    (cl:cons ':trackedHumans (trackedHumans msg))
))
