; Auto-generated. Do not edit!


(cl:in-package AccompanyService-srv)


;//! \htmlinclude db_msg-request.msg.html

(cl:defclass <db_msg-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:integer
    :initform 0)
   (param
    :reader param
    :initarg :param
    :type cl:string
    :initform "")
   (sonReq
    :reader sonReq
    :initarg :sonReq
    :type cl:integer
    :initform 0))
)

(cl:defclass db_msg-request (<db_msg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <db_msg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'db_msg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AccompanyService-srv:<db_msg-request> is deprecated: use AccompanyService-srv:db_msg-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <db_msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:request-val is deprecated.  Use AccompanyService-srv:request instead.")
  (request m))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <db_msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:param-val is deprecated.  Use AccompanyService-srv:param instead.")
  (param m))

(cl:ensure-generic-function 'sonReq-val :lambda-list '(m))
(cl:defmethod sonReq-val ((m <db_msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:sonReq-val is deprecated.  Use AccompanyService-srv:sonReq instead.")
  (sonReq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <db_msg-request>) ostream)
  "Serializes a message object of type '<db_msg-request>"
  (cl:let* ((signed (cl:slot-value msg 'request)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'param))
  (cl:let* ((signed (cl:slot-value msg 'sonReq)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <db_msg-request>) istream)
  "Deserializes a message object of type '<db_msg-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'param) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sonReq) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<db_msg-request>)))
  "Returns string type for a service object of type '<db_msg-request>"
  "AccompanyService/db_msgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'db_msg-request)))
  "Returns string type for a service object of type 'db_msg-request"
  "AccompanyService/db_msgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<db_msg-request>)))
  "Returns md5sum for a message object of type '<db_msg-request>"
  "3e6bed7dbd53be1f4d5a46e4a74c1074")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'db_msg-request)))
  "Returns md5sum for a message object of type 'db_msg-request"
  "3e6bed7dbd53be1f4d5a46e4a74c1074")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<db_msg-request>)))
  "Returns full string definition for message of type '<db_msg-request>"
  (cl:format cl:nil "int64 request~%string param~%int64 sonReq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'db_msg-request)))
  "Returns full string definition for message of type 'db_msg-request"
  (cl:format cl:nil "int64 request~%string param~%int64 sonReq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <db_msg-request>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'param))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <db_msg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'db_msg-request
    (cl:cons ':request (request msg))
    (cl:cons ':param (param msg))
    (cl:cons ':sonReq (sonReq msg))
))
;//! \htmlinclude db_msg-response.msg.html

(cl:defclass <db_msg-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0)
   (sonRes
    :reader sonRes
    :initarg :sonRes
    :type cl:integer
    :initform 0)
   (answer
    :reader answer
    :initarg :answer
    :type cl:string
    :initform ""))
)

(cl:defclass db_msg-response (<db_msg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <db_msg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'db_msg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AccompanyService-srv:<db_msg-response> is deprecated: use AccompanyService-srv:db_msg-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <db_msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:code-val is deprecated.  Use AccompanyService-srv:code instead.")
  (code m))

(cl:ensure-generic-function 'sonRes-val :lambda-list '(m))
(cl:defmethod sonRes-val ((m <db_msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:sonRes-val is deprecated.  Use AccompanyService-srv:sonRes instead.")
  (sonRes m))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <db_msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AccompanyService-srv:answer-val is deprecated.  Use AccompanyService-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <db_msg-response>) ostream)
  "Serializes a message object of type '<db_msg-response>"
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sonRes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <db_msg-response>) istream)
  "Deserializes a message object of type '<db_msg-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sonRes) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<db_msg-response>)))
  "Returns string type for a service object of type '<db_msg-response>"
  "AccompanyService/db_msgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'db_msg-response)))
  "Returns string type for a service object of type 'db_msg-response"
  "AccompanyService/db_msgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<db_msg-response>)))
  "Returns md5sum for a message object of type '<db_msg-response>"
  "3e6bed7dbd53be1f4d5a46e4a74c1074")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'db_msg-response)))
  "Returns md5sum for a message object of type 'db_msg-response"
  "3e6bed7dbd53be1f4d5a46e4a74c1074")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<db_msg-response>)))
  "Returns full string definition for message of type '<db_msg-response>"
  (cl:format cl:nil "int64 code~%int64 sonRes~%string answer~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'db_msg-response)))
  "Returns full string definition for message of type 'db_msg-response"
  (cl:format cl:nil "int64 code~%int64 sonRes~%string answer~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <db_msg-response>))
  (cl:+ 0
     8
     8
     4 (cl:length (cl:slot-value msg 'answer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <db_msg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'db_msg-response
    (cl:cons ':code (code msg))
    (cl:cons ':sonRes (sonRes msg))
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'db_msg)))
  'db_msg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'db_msg)))
  'db_msg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'db_msg)))
  "Returns string type for a service object of type '<db_msg>"
  "AccompanyService/db_msg")