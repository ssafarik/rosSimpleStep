; Auto-generated. Do not edit!


(cl:in-package rosSimpleStep-srv)


;//! \htmlinclude SrvSetZero-request.msg.html

(cl:defclass <SrvSetZero-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrvSetZero-request (<SrvSetZero-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvSetZero-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvSetZero-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosSimpleStep-srv:<SrvSetZero-request> is deprecated: use rosSimpleStep-srv:SrvSetZero-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <SrvSetZero-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:pos-val is deprecated.  Use rosSimpleStep-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvSetZero-request>) ostream)
  "Serializes a message object of type '<SrvSetZero-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvSetZero-request>) istream)
  "Deserializes a message object of type '<SrvSetZero-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvSetZero-request>)))
  "Returns string type for a service object of type '<SrvSetZero-request>"
  "rosSimpleStep/SrvSetZeroRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvSetZero-request)))
  "Returns string type for a service object of type 'SrvSetZero-request"
  "rosSimpleStep/SrvSetZeroRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvSetZero-request>)))
  "Returns md5sum for a message object of type '<SrvSetZero-request>"
  "601132c60d9f2166de4fd581d485cb38")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvSetZero-request)))
  "Returns md5sum for a message object of type 'SrvSetZero-request"
  "601132c60d9f2166de4fd581d485cb38")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvSetZero-request>)))
  "Returns full string definition for message of type '<SrvSetZero-request>"
  (cl:format cl:nil "float64 pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvSetZero-request)))
  "Returns full string definition for message of type 'SrvSetZero-request"
  (cl:format cl:nil "float64 pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvSetZero-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvSetZero-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvSetZero-request
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude SrvSetZero-response.msg.html

(cl:defclass <SrvSetZero-response> (roslisp-msg-protocol:ros-message)
  ((bStatus
    :reader bStatus
    :initarg :bStatus
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrvSetZero-response (<SrvSetZero-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvSetZero-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvSetZero-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosSimpleStep-srv:<SrvSetZero-response> is deprecated: use rosSimpleStep-srv:SrvSetZero-response instead.")))

(cl:ensure-generic-function 'bStatus-val :lambda-list '(m))
(cl:defmethod bStatus-val ((m <SrvSetZero-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:bStatus-val is deprecated.  Use rosSimpleStep-srv:bStatus instead.")
  (bStatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvSetZero-response>) ostream)
  "Serializes a message object of type '<SrvSetZero-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bStatus) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvSetZero-response>) istream)
  "Deserializes a message object of type '<SrvSetZero-response>"
    (cl:setf (cl:slot-value msg 'bStatus) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvSetZero-response>)))
  "Returns string type for a service object of type '<SrvSetZero-response>"
  "rosSimpleStep/SrvSetZeroResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvSetZero-response)))
  "Returns string type for a service object of type 'SrvSetZero-response"
  "rosSimpleStep/SrvSetZeroResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvSetZero-response>)))
  "Returns md5sum for a message object of type '<SrvSetZero-response>"
  "601132c60d9f2166de4fd581d485cb38")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvSetZero-response)))
  "Returns md5sum for a message object of type 'SrvSetZero-response"
  "601132c60d9f2166de4fd581d485cb38")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvSetZero-response>)))
  "Returns full string definition for message of type '<SrvSetZero-response>"
  (cl:format cl:nil "bool bStatus~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvSetZero-response)))
  "Returns full string definition for message of type 'SrvSetZero-response"
  (cl:format cl:nil "bool bStatus~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvSetZero-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvSetZero-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvSetZero-response
    (cl:cons ':bStatus (bStatus msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvSetZero)))
  'SrvSetZero-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvSetZero)))
  'SrvSetZero-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvSetZero)))
  "Returns string type for a service object of type '<SrvSetZero>"
  "rosSimpleStep/SrvSetZero")