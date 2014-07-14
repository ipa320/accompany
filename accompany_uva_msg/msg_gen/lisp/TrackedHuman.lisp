; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude TrackedHuman.msg.html

(cl:defclass <TrackedHuman> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (speed
    :reader speed
    :initarg :speed
    :type geometry_msgs-msg:Vector3Stamped
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3Stamped))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (identity
    :reader identity
    :initarg :identity
    :type cl:string
    :initform "")
   (firstSeen
    :reader firstSeen
    :initarg :firstSeen
    :type cl:real
    :initform 0)
   (lastSeen
    :reader lastSeen
    :initarg :lastSeen
    :type cl:real
    :initform 0)
   (specialFlag
    :reader specialFlag
    :initarg :specialFlag
    :type cl:integer
    :initform 0))
)

(cl:defclass TrackedHuman (<TrackedHuman>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedHuman>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedHuman)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<TrackedHuman> is deprecated: use accompany_uva_msg-msg:TrackedHuman instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:location-val is deprecated.  Use accompany_uva_msg-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:speed-val is deprecated.  Use accompany_uva_msg-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:id-val is deprecated.  Use accompany_uva_msg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'identity-val :lambda-list '(m))
(cl:defmethod identity-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:identity-val is deprecated.  Use accompany_uva_msg-msg:identity instead.")
  (identity m))

(cl:ensure-generic-function 'firstSeen-val :lambda-list '(m))
(cl:defmethod firstSeen-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:firstSeen-val is deprecated.  Use accompany_uva_msg-msg:firstSeen instead.")
  (firstSeen m))

(cl:ensure-generic-function 'lastSeen-val :lambda-list '(m))
(cl:defmethod lastSeen-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:lastSeen-val is deprecated.  Use accompany_uva_msg-msg:lastSeen instead.")
  (lastSeen m))

(cl:ensure-generic-function 'specialFlag-val :lambda-list '(m))
(cl:defmethod specialFlag-val ((m <TrackedHuman>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:specialFlag-val is deprecated.  Use accompany_uva_msg-msg:specialFlag instead.")
  (specialFlag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedHuman>) ostream)
  "Serializes a message object of type '<TrackedHuman>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'speed) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'identity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'identity))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'firstSeen)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'firstSeen) (cl:floor (cl:slot-value msg 'firstSeen)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'lastSeen)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'lastSeen) (cl:floor (cl:slot-value msg 'lastSeen)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'specialFlag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedHuman>) istream)
  "Deserializes a message object of type '<TrackedHuman>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'speed) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'identity) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'identity) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'firstSeen) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lastSeen) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'specialFlag) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedHuman>)))
  "Returns string type for a message object of type '<TrackedHuman>"
  "accompany_uva_msg/TrackedHuman")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedHuman)))
  "Returns string type for a message object of type 'TrackedHuman"
  "accompany_uva_msg/TrackedHuman")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedHuman>)))
  "Returns md5sum for a message object of type '<TrackedHuman>"
  "831337243a4eece3e6abc55b17ef99e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedHuman)))
  "Returns md5sum for a message object of type 'TrackedHuman"
  "831337243a4eece3e6abc55b17ef99e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedHuman>)))
  "Returns full string definition for message of type '<TrackedHuman>"
  (cl:format cl:nil "geometry_msgs/PointStamped location~%geometry_msgs/Vector3Stamped speed~%int32 id~%string identity~%time firstSeen~%time lastSeen~%int32 specialFlag~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3Stamped~%# This represents a Vector3 with reference coordinate frame and timestamp~%Header header~%Vector3 vector~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedHuman)))
  "Returns full string definition for message of type 'TrackedHuman"
  (cl:format cl:nil "geometry_msgs/PointStamped location~%geometry_msgs/Vector3Stamped speed~%int32 id~%string identity~%time firstSeen~%time lastSeen~%int32 specialFlag~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3Stamped~%# This represents a Vector3 with reference coordinate frame and timestamp~%Header header~%Vector3 vector~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedHuman>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'speed))
     4
     4 (cl:length (cl:slot-value msg 'identity))
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedHuman>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedHuman
    (cl:cons ':location (location msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':id (id msg))
    (cl:cons ':identity (identity msg))
    (cl:cons ':firstSeen (firstSeen msg))
    (cl:cons ':lastSeen (lastSeen msg))
    (cl:cons ':specialFlag (specialFlag msg))
))
