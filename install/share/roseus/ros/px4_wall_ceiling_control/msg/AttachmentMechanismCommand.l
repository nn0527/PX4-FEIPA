;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::AttachmentMechanismCommand)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'AttachmentMechanismCommand (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND")
  (make-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude AttachmentMechanismCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOURCE_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*SOURCE_NONE* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*SOURCE_NONE* 0)
(intern "*SOURCE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*SOURCE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*SOURCE_CEILING* 1)
(intern "*SOURCE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*SOURCE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*SOURCE_WALL* 2)
(intern "*ACTION_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*ACTION_NONE* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_NONE* 0)
(intern "*ACTION_HOLD*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*ACTION_HOLD* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_HOLD* 1)
(intern "*ACTION_RELEASE*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*ACTION_RELEASE* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_RELEASE* 2)
(intern "*ACTION_STOP*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(shadow '*ACTION_STOP* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMCOMMAND"))
(defconstant px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_STOP* 3)

(defun px4_wall_ceiling_control::AttachmentMechanismCommand-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*SOURCE_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*SOURCE_CEILING*)
        ((= const 2) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*SOURCE_WALL*)
        ((= const 0) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_HOLD*)
        ((= const 2) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_RELEASE*)
        ((= const 3) 'px4_wall_ceiling_control::AttachmentMechanismCommand::*ACTION_STOP*)
        (t nil)))

(defclass px4_wall_ceiling_control::AttachmentMechanismCommand
  :super ros::object
  :slots (_header _source _action _sequence _reason ))

(defmethod px4_wall_ceiling_control::AttachmentMechanismCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:source __source) 0)
    ((:action __action) 0)
    ((:sequence __sequence) 0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _source (round __source))
   (setq _action (round __action))
   (setq _sequence (round __sequence))
   (setq _reason (string __reason))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:sequence
   (&optional __sequence)
   (if __sequence (setq _sequence __sequence)) _sequence)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _source
    1
    ;; uint8 _action
    1
    ;; uint32 _sequence
    4
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _source
       (write-byte _source s)
     ;; uint8 _action
       (write-byte _action s)
     ;; uint32 _sequence
       (write-long _sequence s)
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _source
     (setq _source (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _action
     (setq _action (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _sequence
     (setq _sequence (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::AttachmentMechanismCommand :md5sum-) "1666c47965aea3b85746e7a2abcbf85d")
(setf (get px4_wall_ceiling_control::AttachmentMechanismCommand :datatype-) "px4_wall_ceiling_control/AttachmentMechanismCommand")
(setf (get px4_wall_ceiling_control::AttachmentMechanismCommand :definition-)
      "std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 ACTION_NONE=0
uint8 ACTION_HOLD=1
uint8 ACTION_RELEASE=2
uint8 ACTION_STOP=3

uint8 source
uint8 action
uint32 sequence
string reason

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :px4_wall_ceiling_control/AttachmentMechanismCommand "1666c47965aea3b85746e7a2abcbf85d")


