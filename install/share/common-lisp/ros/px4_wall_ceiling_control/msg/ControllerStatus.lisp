; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude ControllerStatus.msg.html

(cl:defclass <ControllerStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (source
    :reader source
    :initarg :source
    :type cl:fixnum
    :initform 0)
   (healthy
    :reader healthy
    :initarg :healthy
    :type cl:boolean
    :initform cl:nil)
   (input_fresh
    :reader input_fresh
    :initarg :input_fresh
    :type cl:boolean
    :initform cl:nil)
   (target_reached
    :reader target_reached
    :initarg :target_reached
    :type cl:boolean
    :initform cl:nil)
   (detail
    :reader detail
    :initarg :detail
    :type cl:string
    :initform ""))
)

(cl:defclass ControllerStatus (<ControllerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<ControllerStatus> is deprecated: use px4_wall_ceiling_control-msg:ControllerStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControllerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <ControllerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'healthy-val :lambda-list '(m))
(cl:defmethod healthy-val ((m <ControllerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:healthy-val is deprecated.  Use px4_wall_ceiling_control-msg:healthy instead.")
  (healthy m))

(cl:ensure-generic-function 'input_fresh-val :lambda-list '(m))
(cl:defmethod input_fresh-val ((m <ControllerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:input_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:input_fresh instead.")
  (input_fresh m))

(cl:ensure-generic-function 'target_reached-val :lambda-list '(m))
(cl:defmethod target_reached-val ((m <ControllerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:target_reached-val is deprecated.  Use px4_wall_ceiling_control-msg:target_reached instead.")
  (target_reached m))

(cl:ensure-generic-function 'detail-val :lambda-list '(m))
(cl:defmethod detail-val ((m <ControllerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:detail-val is deprecated.  Use px4_wall_ceiling_control-msg:detail instead.")
  (detail m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerStatus>)))
    "Constants for message type '<ControllerStatus>"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerStatus)))
    "Constants for message type 'ControllerStatus"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerStatus>) ostream)
  "Serializes a message object of type '<ControllerStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'healthy) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'input_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'target_reached) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'detail))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'detail))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerStatus>) istream)
  "Deserializes a message object of type '<ControllerStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'healthy) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'input_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'target_reached) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'detail) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'detail) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerStatus>)))
  "Returns string type for a message object of type '<ControllerStatus>"
  "px4_wall_ceiling_control/ControllerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerStatus)))
  "Returns string type for a message object of type 'ControllerStatus"
  "px4_wall_ceiling_control/ControllerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerStatus>)))
  "Returns md5sum for a message object of type '<ControllerStatus>"
  "a4190f296fb56a105a72ae34460a659f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerStatus)))
  "Returns md5sum for a message object of type 'ControllerStatus"
  "a4190f296fb56a105a72ae34460a659f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerStatus>)))
  "Returns full string definition for message of type '<ControllerStatus>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 source~%bool healthy~%bool input_fresh~%bool target_reached~%string detail~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerStatus)))
  "Returns full string definition for message of type 'ControllerStatus"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 source~%bool healthy~%bool input_fresh~%bool target_reached~%string detail~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'detail))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerStatus
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':healthy (healthy msg))
    (cl:cons ':input_fresh (input_fresh msg))
    (cl:cons ':target_reached (target_reached msg))
    (cl:cons ':detail (detail msg))
))
