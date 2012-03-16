; Auto-generated. Do not edit!


(cl:in-package rosSimpleStep-srv)


;//! \htmlinclude SrvPark-request.msg.html

(cl:defclass <SrvPark-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SrvPark-request (<SrvPark-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvPark-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvPark-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosSimpleStep-srv:<SrvPark-request> is deprecated: use rosSimpleStep-srv:SrvPark-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvPark-request>) ostream)
  "Serializes a message object of type '<SrvPark-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvPark-request>) istream)
  "Deserializes a message object of type '<SrvPark-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvPark-request>)))
  "Returns string type for a service object of type '<SrvPark-request>"
  "rosSimpleStep/SrvParkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvPark-request)))
  "Returns string type for a service object of type 'SrvPark-request"
  "rosSimpleStep/SrvParkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvPark-request>)))
  "Returns md5sum for a message object of type '<SrvPark-request>"
  "3e18d87dd8087d83731a8e3c5780564e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvPark-request)))
  "Returns md5sum for a message object of type 'SrvPark-request"
  "3e18d87dd8087d83731a8e3c5780564e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvPark-request>)))
  "Returns full string definition for message of type '<SrvPark-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvPark-request)))
  "Returns full string definition for message of type 'SrvPark-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvPark-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvPark-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvPark-request
))
;//! \htmlinclude SrvPark-response.msg.html

(cl:defclass <SrvPark-response> (roslisp-msg-protocol:ros-message)
  ((bStatus
    :reader bStatus
    :initarg :bStatus
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrvPark-response (<SrvPark-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvPark-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvPark-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosSimpleStep-srv:<SrvPark-response> is deprecated: use rosSimpleStep-srv:SrvPark-response instead.")))

(cl:ensure-generic-function 'bStatus-val :lambda-list '(m))
(cl:defmethod bStatus-val ((m <SrvPark-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosSimpleStep-srv:bStatus-val is deprecated.  Use rosSimpleStep-srv:bStatus instead.")
  (bStatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvPark-response>) ostream)
  "Serializes a message object of type '<SrvPark-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bStatus) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvPark-response>) istream)
  "Deserializes a message object of type '<SrvPark-response>"
    (cl:setf (cl:slot-value msg 'bStatus) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvPark-response>)))
  "Returns string type for a service object of type '<SrvPark-response>"
  "rosSimpleStep/SrvParkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvPark-response)))
  "Returns string type for a service object of type 'SrvPark-response"
  "rosSimpleStep/SrvParkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvPark-response>)))
  "Returns md5sum for a message object of type '<SrvPark-response>"
  "3e18d87dd8087d83731a8e3c5780564e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvPark-response)))
  "Returns md5sum for a message object of type 'SrvPark-response"
  "3e18d87dd8087d83731a8e3c5780564e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvPark-response>)))
  "Returns full string definition for message of type '<SrvPark-response>"
  (cl:format cl:nil "bool bStatus~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvPark-response)))
  "Returns full string definition for message of type 'SrvPark-response"
  (cl:format cl:nil "bool bStatus~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvPark-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvPark-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvPark-response
    (cl:cons ':bStatus (bStatus msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvPark)))
  'SrvPark-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvPark)))
  'SrvPark-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvPark)))
  "Returns string type for a service object of type '<SrvPark>"
  "rosSimpleStep/SrvPark")