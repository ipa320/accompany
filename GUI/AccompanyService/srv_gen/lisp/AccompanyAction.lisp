; Auto-generated. Do not edit!


(cl:in-package AccompanyService-srv)


;//! \htmlinclude AccompanyAction-request.msg.html

(cl:defclass <AccompanyAction-request> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type cl:string
    :initform "")
   (uid
    :reader uid
    :initarg :uid
    :type cl:integer
    :initform 0))
)

(cl:defclass AccompanyAction-request (<AccompanyAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AccompanyAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AccompanyAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AccompanyService-srv:<AccompanyAction-request> is deprecated: use AccompanyService-srv:AccompanyAction-request instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <AccompanyAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:action-val is deprecated.  Use AccompanyService-srv:action instead.")
  (action m))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <AccompanyAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:uid-val is deprecated.  Use AccompanyService-srv:uid instead.")
  (uid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AccompanyAction-request>) ostream)
  "Serializes a message object of type '<AccompanyAction-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action))
  (cl:let* ((signed (cl:slot-value msg 'uid)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AccompanyAction-request>) istream)
  "Deserializes a message object of type '<AccompanyAction-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uid) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AccompanyAction-request>)))
  "Returns string type for a service object of type '<AccompanyAction-request>"
  "AccompanyService/AccompanyActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AccompanyAction-request)))
  "Returns string type for a service object of type 'AccompanyAction-request"
  "AccompanyService/AccompanyActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AccompanyAction-request>)))
  "Returns md5sum for a message object of type '<AccompanyAction-request>"
  "76241cf4ea70a5becc8169ed10e2126a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AccompanyAction-request)))
  "Returns md5sum for a message object of type 'AccompanyAction-request"
  "76241cf4ea70a5becc8169ed10e2126a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AccompanyAction-request>)))
  "Returns full string definition for message of type '<AccompanyAction-request>"
  (cl:format cl:nil "string action~%int64 uid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AccompanyAction-request)))
  "Returns full string definition for message of type 'AccompanyAction-request"
  (cl:format cl:nil "string action~%int64 uid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AccompanyAction-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AccompanyAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AccompanyAction-request
    (cl:cons ':action (action msg))
    (cl:cons ':uid (uid msg))
))
;//! \htmlinclude AccompanyAction-response.msg.html

(cl:defclass <AccompanyAction-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass AccompanyAction-response (<AccompanyAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AccompanyAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AccompanyAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AccompanyService-srv:<AccompanyAction-response> is deprecated: use AccompanyService-srv:AccompanyAction-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <AccompanyAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:result-val is deprecated.  Use AccompanyService-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AccompanyAction-response>) ostream)
  "Serializes a message object of type '<AccompanyAction-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AccompanyAction-response>) istream)
  "Deserializes a message object of type '<AccompanyAction-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AccompanyAction-response>)))
  "Returns string type for a service object of type '<AccompanyAction-response>"
  "AccompanyService/AccompanyActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AccompanyAction-response)))
  "Returns string type for a service object of type 'AccompanyAction-response"
  "AccompanyService/AccompanyActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AccompanyAction-response>)))
  "Returns md5sum for a message object of type '<AccompanyAction-response>"
  "76241cf4ea70a5becc8169ed10e2126a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AccompanyAction-response)))
  "Returns md5sum for a message object of type 'AccompanyAction-response"
  "76241cf4ea70a5becc8169ed10e2126a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AccompanyAction-response>)))
  "Returns full string definition for message of type '<AccompanyAction-response>"
  (cl:format cl:nil "int64 result~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AccompanyAction-response)))
  "Returns full string definition for message of type 'AccompanyAction-response"
  (cl:format cl:nil "int64 result~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AccompanyAction-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AccompanyAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AccompanyAction-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AccompanyAction)))
  'AccompanyAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AccompanyAction)))
  'AccompanyAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AccompanyAction)))
  "Returns string type for a service object of type '<AccompanyAction>"
  "AccompanyService/AccompanyAction")