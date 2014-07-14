; Auto-generated. Do not edit!


(cl:in-package accompany_uva_msg-msg)


;//! \htmlinclude Appearance.msg.html

(cl:defclass <Appearance> (roslisp-msg-protocol:ros-message)
  ((sumTemplatePixelSize
    :reader sumTemplatePixelSize
    :initarg :sumTemplatePixelSize
    :type cl:integer
    :initform 0)
   (sumPixelWeights
    :reader sumPixelWeights
    :initarg :sumPixelWeights
    :type cl:float
    :initform 0.0)
   (histogram
    :reader histogram
    :initarg :histogram
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Appearance (<Appearance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Appearance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Appearance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name accompany_uva_msg-msg:<Appearance> is deprecated: use accompany_uva_msg-msg:Appearance instead.")))

(cl:ensure-generic-function 'sumTemplatePixelSize-val :lambda-list '(m))
(cl:defmethod sumTemplatePixelSize-val ((m <Appearance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:sumTemplatePixelSize-val is deprecated.  Use accompany_uva_msg-msg:sumTemplatePixelSize instead.")
  (sumTemplatePixelSize m))

(cl:ensure-generic-function 'sumPixelWeights-val :lambda-list '(m))
(cl:defmethod sumPixelWeights-val ((m <Appearance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:sumPixelWeights-val is deprecated.  Use accompany_uva_msg-msg:sumPixelWeights instead.")
  (sumPixelWeights m))

(cl:ensure-generic-function 'histogram-val :lambda-list '(m))
(cl:defmethod histogram-val ((m <Appearance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader accompany_uva_msg-msg:histogram-val is deprecated.  Use accompany_uva_msg-msg:histogram instead.")
  (histogram m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Appearance>) ostream)
  "Serializes a message object of type '<Appearance>"
  (cl:let* ((signed (cl:slot-value msg 'sumTemplatePixelSize)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sumPixelWeights))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'histogram))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'histogram))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Appearance>) istream)
  "Deserializes a message object of type '<Appearance>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sumTemplatePixelSize) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sumPixelWeights) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'histogram) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'histogram)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Appearance>)))
  "Returns string type for a message object of type '<Appearance>"
  "accompany_uva_msg/Appearance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Appearance)))
  "Returns string type for a message object of type 'Appearance"
  "accompany_uva_msg/Appearance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Appearance>)))
  "Returns md5sum for a message object of type '<Appearance>"
  "bd36eb66e6a8febafac79f820b94b7e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Appearance)))
  "Returns md5sum for a message object of type 'Appearance"
  "bd36eb66e6a8febafac79f820b94b7e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Appearance>)))
  "Returns full string definition for message of type '<Appearance>"
  (cl:format cl:nil "int32      sumTemplatePixelSize~%float64    sumPixelWeights~%float64[]  histogram~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Appearance)))
  "Returns full string definition for message of type 'Appearance"
  (cl:format cl:nil "int32      sumTemplatePixelSize~%float64    sumPixelWeights~%float64[]  histogram~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Appearance>))
  (cl:+ 0
     4
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'histogram) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Appearance>))
  "Converts a ROS message object to a list"
  (cl:list 'Appearance
    (cl:cons ':sumTemplatePixelSize (sumTemplatePixelSize msg))
    (cl:cons ':sumPixelWeights (sumPixelWeights msg))
    (cl:cons ':histogram (histogram msg))
))
