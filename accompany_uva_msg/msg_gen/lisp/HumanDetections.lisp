; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude HumanDetections.msg.html

(cl:defclass <HumanDetections> (roslisp-msg-protocol:ros-message)
  ((detections
    :reader detections
    :initarg :detections
    :type (cl:vector accompany_uva_msg-msg:HumanDetection)
   :initform (cl:make-array 0 :element-type 'accompany_uva_msg-msg:HumanDetection :initial-element (cl:make-instance 'accompany_uva_msg-msg:HumanDetection))))
)

(cl:defclass HumanDetections (<HumanDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<HumanDetections> is deprecated: use accompany_uva_msg-msg:HumanDetections instead.")))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <HumanDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:detections-val is deprecated.  Use accompany_uva_msg-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanDetections>) ostream)
  "Serializes a message object of type '<HumanDetections>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanDetections>) istream)
  "Deserializes a message object of type '<HumanDetections>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'accompany_uva_msg-msg:HumanDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanDetections>)))
  "Returns string type for a message object of type '<HumanDetections>"
  "accompany_uva_msg/HumanDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanDetections)))
  "Returns string type for a message object of type 'HumanDetections"
  "accompany_uva_msg/HumanDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanDetections>)))
  "Returns md5sum for a message object of type '<HumanDetections>"
  "5465ff2088728580f9bb2ff7bab44360")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanDetections)))
  "Returns md5sum for a message object of type 'HumanDetections"
  "5465ff2088728580f9bb2ff7bab44360")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanDetections>)))
  "Returns full string definition for message of type '<HumanDetections>"
  (cl:format cl:nil "HumanDetection[] detections~%~%================================================================================~%MSG: accompany_uva_msg/HumanDetection~%geometry_msgs/PointStamped location~%Appearance appearance~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: accompany_uva_msg/Appearance~%int32      sumTemplatePixelSize~%float64    sumPixelWeights~%float64[]  histogram~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanDetections)))
  "Returns full string definition for message of type 'HumanDetections"
  (cl:format cl:nil "HumanDetection[] detections~%~%================================================================================~%MSG: accompany_uva_msg/HumanDetection~%geometry_msgs/PointStamped location~%Appearance appearance~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: accompany_uva_msg/Appearance~%int32      sumTemplatePixelSize~%float64    sumPixelWeights~%float64[]  histogram~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanDetections>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanDetections
    (cl:cons ':detections (detections msg))
))
