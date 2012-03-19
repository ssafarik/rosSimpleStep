; Auto-generated. Do not edit!


(cl:in-package rosSimpleStep-srv)


;//! \htmlinclude SrvCalibrate-request.msg.html

(cl:defclass <SrvCalibrate-request> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (posOrigin
    :reader posOrigin
    :initarg :posOrigin
    :type cl:float
    :initform 0.0)
   (posPark
    :reader posPark
    :initarg :posPark
    :type cl:float
    :initform 0.0)
   (findIndex
    :reader findIndex
    :initarg :findIndex
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrvCalibrate-request (<SrvCalibrate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvCalibrate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvCalibrate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosSimpleStep-srv:<SrvCalibrate-request> is deprecated: use rosSimpleStep-srv:SrvCalibrate-request instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <SrvCalibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:direction-val is deprecated.  Use rosSimpleStep-srv:direction instead.")
  (direction m))

(cl:ensure-generic-function 'posOrigin-val :lambda-list '(m))
(cl:defmethod posOrigin-val ((m <SrvCalibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:posOrigin-val is deprecated.  Use rosSimpleStep-srv:posOrigin instead.")
  (posOrigin m))

(cl:ensure-generic-function 'posPark-val :lambda-list '(m))
(cl:defmethod posPark-val ((m <SrvCalibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:posPark-val is deprecated.  Use rosSimpleStep-srv:posPark instead.")
  (posPark m))

(cl:ensure-generic-function 'findIndex-val :lambda-list '(m))
(cl:defmethod findIndex-val ((m <SrvCalibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:findIndex-val is deprecated.  Use rosSimpleStep-srv:findIndex instead.")
  (findIndex m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvCalibrate-request>) ostream)
  "Serializes a message object of type '<SrvCalibrate-request>"
  (cl:let* ((signed (cl:slot-value msg 'direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'posOrigin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'posPark))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'findIndex) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvCalibrate-request>) istream)
  "Deserializes a message object of type '<SrvCalibrate-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'posOrigin) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'posPark) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'findIndex) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvCalibrate-request>)))
  "Returns string type for a service object of type '<SrvCalibrate-request>"
  "rosSimpleStep/SrvCalibrateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvCalibrate-request)))
  "Returns string type for a service object of type 'SrvCalibrate-request"
  "rosSimpleStep/SrvCalibrateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvCalibrate-request>)))
  "Returns md5sum for a message object of type '<SrvCalibrate-request>"
  "e6b1e4f347387c9840fcdbc0affbf11b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvCalibrate-request)))
  "Returns md5sum for a message object of type 'SrvCalibrate-request"
  "e6b1e4f347387c9840fcdbc0affbf11b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvCalibrate-request>)))
  "Returns full string definition for message of type '<SrvCalibrate-request>"
  (cl:format cl:nil "int32 direction~%float32 posOrigin~%float32 posPark~%bool findIndex~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvCalibrate-request)))
  "Returns full string definition for message of type 'SrvCalibrate-request"
  (cl:format cl:nil "int32 direction~%float32 posOrigin~%float32 posPark~%bool findIndex~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvCalibrate-request>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvCalibrate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvCalibrate-request
    (cl:cons ':direction (direction msg))
    (cl:cons ':posOrigin (posOrigin msg))
    (cl:cons ':posPark (posPark msg))
    (cl:cons ':findIndex (findIndex msg))
))
;//! \htmlinclude SrvCalibrate-response.msg.html

(cl:defclass <SrvCalibrate-response> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrvCalibrate-response (<SrvCalibrate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvCalibrate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvCalibrate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosSimpleStep-srv:<SrvCalibrate-response> is deprecated: use rosSimpleStep-srv:SrvCalibrate-response instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <SrvCalibrate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:position-val is deprecated.  Use rosSimpleStep-srv:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvCalibrate-response>) ostream)
  "Serializes a message object of type '<SrvCalibrate-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvCalibrate-response>) istream)
  "Deserializes a message object of type '<SrvCalibrate-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvCalibrate-response>)))
  "Returns string type for a service object of type '<SrvCalibrate-response>"
  "rosSimpleStep/SrvCalibrateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvCalibrate-response)))
  "Returns string type for a service object of type 'SrvCalibrate-response"
  "rosSimpleStep/SrvCalibrateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvCalibrate-response>)))
  "Returns md5sum for a message object of type '<SrvCalibrate-response>"
  "e6b1e4f347387c9840fcdbc0affbf11b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvCalibrate-response)))
  "Returns md5sum for a message object of type 'SrvCalibrate-response"
  "e6b1e4f347387c9840fcdbc0affbf11b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvCalibrate-response>)))
  "Returns full string definition for message of type '<SrvCalibrate-response>"
  (cl:format cl:nil "float32 position~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvCalibrate-response)))
  "Returns full string definition for message of type 'SrvCalibrate-response"
  (cl:format cl:nil "float32 position~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvCalibrate-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvCalibrate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvCalibrate-response
    (cl:cons ':position (position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvCalibrate)))
  'SrvCalibrate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvCalibrate)))
  'SrvCalibrate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvCalibrate)))
  "Returns string type for a service object of type '<SrvCalibrate>"
  "rosSimpleStep/SrvCalibrate")