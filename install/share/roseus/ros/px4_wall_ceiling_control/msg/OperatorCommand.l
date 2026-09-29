;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::OperatorCommand)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'OperatorCommand (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND")
  (make-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude OperatorCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MODE_MANUAL*" (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(shadow '*MODE_MANUAL* (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(defconstant px4_wall_ceiling_control::OperatorCommand::*MODE_MANUAL* 0)
(intern "*MODE_STANDBY*" (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(shadow '*MODE_STANDBY* (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(defconstant px4_wall_ceiling_control::OperatorCommand::*MODE_STANDBY* 1)
(intern "*MODE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(shadow '*MODE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(defconstant px4_wall_ceiling_control::OperatorCommand::*MODE_CEILING* 2)
(intern "*MODE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(shadow '*MODE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(defconstant px4_wall_ceiling_control::OperatorCommand::*MODE_WALL* 3)
(intern "*MODE_ABORT*" (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(shadow '*MODE_ABORT* (find-package "PX4_WALL_CEILING_CONTROL::OPERATORCOMMAND"))
(defconstant px4_wall_ceiling_control::OperatorCommand::*MODE_ABORT* 4)

(defun px4_wall_ceiling_control::OperatorCommand-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::OperatorCommand::*MODE_MANUAL*)
        ((= const 1) 'px4_wall_ceiling_control::OperatorCommand::*MODE_STANDBY*)
        ((= const 2) 'px4_wall_ceiling_control::OperatorCommand::*MODE_CEILING*)
        ((= const 3) 'px4_wall_ceiling_control::OperatorCommand::*MODE_WALL*)
        ((= const 4) 'px4_wall_ceiling_control::OperatorCommand::*MODE_ABORT*)
        (t nil)))

(defclass px4_wall_ceiling_control::OperatorCommand
  :super ros::object
  :slots (_header _mode _enable_control _source _reason ))

(defmethod px4_wall_ceiling_control::OperatorCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode __mode) 0)
    ((:enable_control __enable_control) nil)
    ((:source __source) "")
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode (round __mode))
   (setq _enable_control __enable_control)
   (setq _source (string __source))
   (setq _reason (string __reason))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:enable_control
   (&optional (__enable_control :null))
   (if (not (eq __enable_control :null)) (setq _enable_control __enable_control)) _enable_control)
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _mode
    1
    ;; bool _enable_control
    1
    ;; string _source
    4 (length _source)
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; bool _enable_control
       (if _enable_control (write-byte -1 s) (write-byte 0 s))
     ;; string _source
       (write-long (length _source) s) (princ _source s)
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _enable_control
     (setq _enable_control (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::OperatorCommand :md5sum-) "e26e5844feffe9808476e201fd10dcc5")
(setf (get px4_wall_ceiling_control::OperatorCommand :datatype-) "px4_wall_ceiling_control/OperatorCommand")
(setf (get px4_wall_ceiling_control::OperatorCommand :definition-)
      "std_msgs/Header header

uint8 MODE_MANUAL=0
uint8 MODE_STANDBY=1
uint8 MODE_CEILING=2
uint8 MODE_WALL=3
uint8 MODE_ABORT=4

uint8 mode
bool enable_control
string source
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



(provide :px4_wall_ceiling_control/OperatorCommand "e26e5844feffe9808476e201fd10dcc5")


