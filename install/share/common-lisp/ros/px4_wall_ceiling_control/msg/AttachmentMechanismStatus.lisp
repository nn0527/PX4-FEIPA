; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude AttachmentMechanismStatus.msg.html

(cl:defclass <AttachmentMechanismStatus> (roslisp-msg-protocol:ros-message)
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
   (sequence
    :reader sequence
    :initarg :sequence
    :type cl:integer
    :initform 0)
   (acknowledged
    :reader acknowledged
    :initarg :acknowledged
    :type cl:boolean
    :initform cl:nil)
   (released
    :reader released
    :initarg :released
    :type cl:boolean
    :initform cl:nil)
   (fault
    :reader fault
    :initarg :fault
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass AttachmentMechanismStatus (<AttachmentMechanismStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttachmentMechanismStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttachmentMechanismStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<AttachmentMechanismStatus> is deprecated: use px4_wall_ceiling_control-msg:AttachmentMechanismStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'sequence-val :lambda-list '(m))
(cl:defmethod sequence-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:sequence-val is deprecated.  Use px4_wall_ceiling_control-msg:sequence instead.")
  (sequence m))

(cl:ensure-generic-function 'acknowledged-val :lambda-list '(m))
(cl:defmethod acknowledged-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:acknowledged-val is deprecated.  Use px4_wall_ceiling_control-msg:acknowledged instead.")
  (acknowledged m))

(cl:ensure-generic-function 'released-val :lambda-list '(m))
(cl:defmethod released-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:released-val is deprecated.  Use px4_wall_ceiling_control-msg:released instead.")
  (released m))

(cl:ensure-generic-function 'fault-val :lambda-list '(m))
(cl:defmethod fault-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:fault-val is deprecated.  Use px4_wall_ceiling_control-msg:fault instead.")
  (fault m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <AttachmentMechanismStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttachmentMechanismStatus>) ostream)
  "Serializes a message object of type '<AttachmentMechanismStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'acknowledged) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'released) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fault) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttachmentMechanismStatus>) istream)
  "Deserializes a message object of type '<AttachmentMechanismStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acknowledged) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'released) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fault) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttachmentMechanismStatus>)))
  "Returns string type for a message object of type '<AttachmentMechanismStatus>"
  "px4_wall_ceiling_control/AttachmentMechanismStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttachmentMechanismStatus)))
  "Returns string type for a message object of type 'AttachmentMechanismStatus"
  "px4_wall_ceiling_control/AttachmentMechanismStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttachmentMechanismStatus>)))
  "Returns md5sum for a message object of type '<AttachmentMechanismStatus>"
  "e3810df96eca58328a3f90eb36d48cc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttachmentMechanismStatus)))
  "Returns md5sum for a message object of type 'AttachmentMechanismStatus"
  "e3810df96eca58328a3f90eb36d48cc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttachmentMechanismStatus>)))
  "Returns full string definition for message of type '<AttachmentMechanismStatus>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 source~%uint32 sequence~%bool acknowledged~%bool released~%bool fault~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttachmentMechanismStatus)))
  "Returns full string definition for message of type 'AttachmentMechanismStatus"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 source~%uint32 sequence~%bool acknowledged~%bool released~%bool fault~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttachmentMechanismStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttachmentMechanismStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'AttachmentMechanismStatus
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':sequence (sequence msg))
    (cl:cons ':acknowledged (acknowledged msg))
    (cl:cons ':released (released msg))
    (cl:cons ':fault (fault msg))
    (cl:cons ':reason (reason msg))
))
