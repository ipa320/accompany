; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude HumanDetection.msg.html

(cl:defclass <HumanDetection> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (appearance
    :reader appearance
    :initarg :appearance
    :type accompany_uva_msg-msg:Appearance
    :initform (cl:make-instance 'accompany_uva_msg-msg:Appearance)))
)

(cl:defclass HumanDetection (<HumanDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<HumanDetection> is deprecated: use accompany_uva_msg-msg:HumanDetection instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <HumanDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:location-val is deprecated.  Use accompany_uva_msg-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'appearance-val :lambda-list '(m))
(cl:defmethod appearance-val ((m <HumanDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:appearance-val is deprecated.  Use accompany_uva_msg-msg:appearance instead.")
  (appearance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanDetection>) ostream)
  "Serializes a message object of type '<HumanDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'appearance) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanDetection>) istream)
  "Deserializes a message object of type '<HumanDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'appearance) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanDetection>)))
  "Returns string type for a message object of type '<HumanDetection>"
  "accompany_uva_msg/HumanDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanDetection)))
  "Returns string type for a message object of type 'HumanDetection"
  "accompany_uva_msg/HumanDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanDetection>)))
  "Returns md5sum for a message object of type '<HumanDetection>"
  "2f5bd75e42cea1d99d835a2e15ac1849")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanDetection)))
  "Returns md5sum for a message object of type 'HumanDetection"
  "2f5bd75e42cea1d99d835a2e15ac1849")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanDetection>)))
  "Returns full string definition for message of type '<HumanDetection>"
  (cl:format cl:nil "geometry_msgs/PointStamped location~%Appearance appearance~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: accompany_uva_msg/Appearance~%int32      sumTemplatePixelSize~%float64    sumPixelWeights~%float64[]  histogram~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanDetection)))
  "Returns full string definition for message of type 'HumanDetection"
  (cl:format cl:nil "geometry_msgs/PointStamped location~%Appearance appearance~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: accompany_uva_msg/Appearance~%int32      sumTemplatePixelSize~%float64    sumPixelWeights~%float64[]  histogram~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'appearance))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanDetection
    (cl:cons ':location (location msg))
    (cl:cons ':appearance (appearance msg))
))
