; Auto-generated. Do not edit!


(cl:in-package behavior_manager-msg)


;//! \htmlinclude DepthBehaviorResult.msg.html

(cl:defclass <DepthBehaviorResult> (roslisp-msg-protocol:ros-message)
  ((current_depth
    :reader current_depth
    :initarg :current_depth
    :type cl:float
    :initform 0.0)
   (depth_error
    :reader depth_error
    :initarg :depth_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass DepthBehaviorResult (<DepthBehaviorResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthBehaviorResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthBehaviorResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_manager-msg:<DepthBehaviorResult> is deprecated: use behavior_manager-msg:DepthBehaviorResult instead.")))

(cl:ensure-generic-function 'current_depth-val :lambda-list '(m))
(cl:defmethod current_depth-val ((m <DepthBehaviorResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-msg:current_depth-val is deprecated.  Use behavior_manager-msg:current_depth instead.")
  (current_depth m))

(cl:ensure-generic-function 'depth_error-val :lambda-list '(m))
(cl:defmethod depth_error-val ((m <DepthBehaviorResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-msg:depth_error-val is deprecated.  Use behavior_manager-msg:depth_error instead.")
  (depth_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthBehaviorResult>) ostream)
  "Serializes a message object of type '<DepthBehaviorResult>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current_depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'depth_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthBehaviorResult>) istream)
  "Deserializes a message object of type '<DepthBehaviorResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_depth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth_error) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthBehaviorResult>)))
  "Returns string type for a message object of type '<DepthBehaviorResult>"
  "behavior_manager/DepthBehaviorResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthBehaviorResult)))
  "Returns string type for a message object of type 'DepthBehaviorResult"
  "behavior_manager/DepthBehaviorResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthBehaviorResult>)))
  "Returns md5sum for a message object of type '<DepthBehaviorResult>"
  "621fe9e47aa651fd68b5c50b58c4d325")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthBehaviorResult)))
  "Returns md5sum for a message object of type 'DepthBehaviorResult"
  "621fe9e47aa651fd68b5c50b58c4d325")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthBehaviorResult>)))
  "Returns full string definition for message of type '<DepthBehaviorResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# ======= Feedback =======~%float64 current_depth    # 当前观测深度~%float64 depth_error      # current_depth - target_depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthBehaviorResult)))
  "Returns full string definition for message of type 'DepthBehaviorResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# ======= Feedback =======~%float64 current_depth    # 当前观测深度~%float64 depth_error      # current_depth - target_depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthBehaviorResult>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthBehaviorResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthBehaviorResult
    (cl:cons ':current_depth (current_depth msg))
    (cl:cons ':depth_error (depth_error msg))
))
