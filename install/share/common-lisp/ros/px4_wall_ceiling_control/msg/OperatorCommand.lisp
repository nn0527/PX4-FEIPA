; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude OperatorCommand.msg.html

(cl:defclass <OperatorCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (enable_control
    :reader enable_control
    :initarg :enable_control
    :type cl:boolean
    :initform cl:nil)
   (source
    :reader source
    :initarg :source
    :type cl:string
    :initform "")
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass OperatorCommand (<OperatorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OperatorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OperatorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<OperatorCommand> is deprecated: use px4_wall_ceiling_control-msg:OperatorCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OperatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <OperatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:mode-val is deprecated.  Use px4_wall_ceiling_control-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'enable_control-val :lambda-list '(m))
(cl:defmethod enable_control-val ((m <OperatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:enable_control-val is deprecated.  Use px4_wall_ceiling_control-msg:enable_control instead.")
  (enable_control m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <OperatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <OperatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<OperatorCommand>)))
    "Constants for message type '<OperatorCommand>"
  '((:MODE_MANUAL . 0)
    (:MODE_STANDBY . 1)
    (:MODE_CEILING . 2)
    (:MODE_WALL . 3)
    (:MODE_ABORT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'OperatorCommand)))
    "Constants for message type 'OperatorCommand"
  '((:MODE_MANUAL . 0)
    (:MODE_STANDBY . 1)
    (:MODE_CEILING . 2)
    (:MODE_WALL . 3)
    (:MODE_ABORT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OperatorCommand>) ostream)
  "Serializes a message object of type '<OperatorCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable_control) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OperatorCommand>) istream)
  "Deserializes a message object of type '<OperatorCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enable_control) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OperatorCommand>)))
  "Returns string type for a message object of type '<OperatorCommand>"
  "px4_wall_ceiling_control/OperatorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OperatorCommand)))
  "Returns string type for a message object of type 'OperatorCommand"
  "px4_wall_ceiling_control/OperatorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OperatorCommand>)))
  "Returns md5sum for a message object of type '<OperatorCommand>"
  "e26e5844feffe9808476e201fd10dcc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OperatorCommand)))
  "Returns md5sum for a message object of type 'OperatorCommand"
  "e26e5844feffe9808476e201fd10dcc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OperatorCommand>)))
  "Returns full string definition for message of type '<OperatorCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 MODE_MANUAL=0~%uint8 MODE_STANDBY=1~%uint8 MODE_CEILING=2~%uint8 MODE_WALL=3~%uint8 MODE_ABORT=4~%~%uint8 mode~%bool enable_control~%string source~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OperatorCommand)))
  "Returns full string definition for message of type 'OperatorCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 MODE_MANUAL=0~%uint8 MODE_STANDBY=1~%uint8 MODE_CEILING=2~%uint8 MODE_WALL=3~%uint8 MODE_ABORT=4~%~%uint8 mode~%bool enable_control~%string source~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OperatorCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4 (cl:length (cl:slot-value msg 'source))
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OperatorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'OperatorCommand
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':enable_control (enable_control msg))
    (cl:cons ':source (source msg))
    (cl:cons ':reason (reason msg))
))
