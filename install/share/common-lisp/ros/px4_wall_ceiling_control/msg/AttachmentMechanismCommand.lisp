; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude AttachmentMechanismCommand.msg.html

(cl:defclass <AttachmentMechanismCommand> (roslisp-msg-protocol:ros-message)
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
   (action
    :reader action
    :initarg :action
    :type cl:fixnum
    :initform 0)
   (sequence
    :reader sequence
    :initarg :sequence
    :type cl:integer
    :initform 0)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass AttachmentMechanismCommand (<AttachmentMechanismCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttachmentMechanismCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttachmentMechanismCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<AttachmentMechanismCommand> is deprecated: use px4_wall_ceiling_control-msg:AttachmentMechanismCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttachmentMechanismCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <AttachmentMechanismCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <AttachmentMechanismCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:action-val is deprecated.  Use px4_wall_ceiling_control-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'sequence-val :lambda-list '(m))
(cl:defmethod sequence-val ((m <AttachmentMechanismCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:sequence-val is deprecated.  Use px4_wall_ceiling_control-msg:sequence instead.")
  (sequence m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <AttachmentMechanismCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AttachmentMechanismCommand>)))
    "Constants for message type '<AttachmentMechanismCommand>"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:ACTION_NONE . 0)
    (:ACTION_HOLD . 1)
    (:ACTION_RELEASE . 2)
    (:ACTION_STOP . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AttachmentMechanismCommand)))
    "Constants for message type 'AttachmentMechanismCommand"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:ACTION_NONE . 0)
    (:ACTION_HOLD . 1)
    (:ACTION_RELEASE . 2)
    (:ACTION_STOP . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttachmentMechanismCommand>) ostream)
  "Serializes a message object of type '<AttachmentMechanismCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttachmentMechanismCommand>) istream)
  "Deserializes a message object of type '<AttachmentMechanismCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttachmentMechanismCommand>)))
  "Returns string type for a message object of type '<AttachmentMechanismCommand>"
  "px4_wall_ceiling_control/AttachmentMechanismCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttachmentMechanismCommand)))
  "Returns string type for a message object of type 'AttachmentMechanismCommand"
  "px4_wall_ceiling_control/AttachmentMechanismCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttachmentMechanismCommand>)))
  "Returns md5sum for a message object of type '<AttachmentMechanismCommand>"
  "1666c47965aea3b85746e7a2abcbf85d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttachmentMechanismCommand)))
  "Returns md5sum for a message object of type 'AttachmentMechanismCommand"
  "1666c47965aea3b85746e7a2abcbf85d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttachmentMechanismCommand>)))
  "Returns full string definition for message of type '<AttachmentMechanismCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 ACTION_NONE=0~%uint8 ACTION_HOLD=1~%uint8 ACTION_RELEASE=2~%uint8 ACTION_STOP=3~%~%uint8 source~%uint8 action~%uint32 sequence~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttachmentMechanismCommand)))
  "Returns full string definition for message of type 'AttachmentMechanismCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 ACTION_NONE=0~%uint8 ACTION_HOLD=1~%uint8 ACTION_RELEASE=2~%uint8 ACTION_STOP=3~%~%uint8 source~%uint8 action~%uint32 sequence~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttachmentMechanismCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttachmentMechanismCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'AttachmentMechanismCommand
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':action (action msg))
    (cl:cons ':sequence (sequence msg))
    (cl:cons ':reason (reason msg))
))
