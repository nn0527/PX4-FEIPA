;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::SupervisorState)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'SupervisorState (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE")
  (make-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))

(in-package "ROS")
;;//! \htmlinclude SupervisorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_WAIT_LINK*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_WAIT_LINK* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_WAIT_LINK* 0)
(intern "*STATE_MANUAL*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_MANUAL* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_MANUAL* 1)
(intern "*STATE_STANDBY*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_STANDBY* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_STANDBY* 2)
(intern "*STATE_PRESTREAM*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_PRESTREAM* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_PRESTREAM* 3)
(intern "*STATE_CEILING_ACTIVE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_CEILING_ACTIVE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_CEILING_ACTIVE* 4)
(intern "*STATE_WALL_ACTIVE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_WALL_ACTIVE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_WALL_ACTIVE* 5)
(intern "*STATE_FAULT*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*STATE_FAULT* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*STATE_FAULT* 6)
(intern "*MODE_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*MODE_NONE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*MODE_NONE* 0)
(intern "*MODE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*MODE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*MODE_CEILING* 1)
(intern "*MODE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*MODE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*MODE_WALL* 2)
(intern "*FAULT_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_NONE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_NONE* 0)
(intern "*FAULT_MAVROS*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_MAVROS* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_MAVROS* 1)
(intern "*FAULT_OPERATOR*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_OPERATOR* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_OPERATOR* 2)
(intern "*FAULT_ODOMETRY*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_ODOMETRY* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_ODOMETRY* 4)
(intern "*FAULT_CEILING_RANGE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_CEILING_RANGE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_CEILING_RANGE* 8)
(intern "*FAULT_WALL_RANGE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_WALL_RANGE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_WALL_RANGE* 16)
(intern "*FAULT_UNKNOWN_MODE*" (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(shadow '*FAULT_UNKNOWN_MODE* (find-package "PX4_WALL_CEILING_CONTROL::SUPERVISORSTATE"))
(defconstant px4_wall_ceiling_control::SupervisorState::*FAULT_UNKNOWN_MODE* 32)

(defun px4_wall_ceiling_control::SupervisorState-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::SupervisorState::*STATE_WAIT_LINK*)
        ((= const 1) 'px4_wall_ceiling_control::SupervisorState::*STATE_MANUAL*)
        ((= const 2) 'px4_wall_ceiling_control::SupervisorState::*STATE_STANDBY*)
        ((= const 3) 'px4_wall_ceiling_control::SupervisorState::*STATE_PRESTREAM*)
        ((= const 4) 'px4_wall_ceiling_control::SupervisorState::*STATE_CEILING_ACTIVE*)
        ((= const 5) 'px4_wall_ceiling_control::SupervisorState::*STATE_WALL_ACTIVE*)
        ((= const 6) 'px4_wall_ceiling_control::SupervisorState::*STATE_FAULT*)
        ((= const 0) 'px4_wall_ceiling_control::SupervisorState::*MODE_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::SupervisorState::*MODE_CEILING*)
        ((= const 2) 'px4_wall_ceiling_control::SupervisorState::*MODE_WALL*)
        ((= const 0) 'px4_wall_ceiling_control::SupervisorState::*FAULT_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::SupervisorState::*FAULT_MAVROS*)
        ((= const 2) 'px4_wall_ceiling_control::SupervisorState::*FAULT_OPERATOR*)
        ((= const 4) 'px4_wall_ceiling_control::SupervisorState::*FAULT_ODOMETRY*)
        ((= const 8) 'px4_wall_ceiling_control::SupervisorState::*FAULT_CEILING_RANGE*)
        ((= const 16) 'px4_wall_ceiling_control::SupervisorState::*FAULT_WALL_RANGE*)
        ((= const 32) 'px4_wall_ceiling_control::SupervisorState::*FAULT_UNKNOWN_MODE*)
        (t nil)))

(defclass px4_wall_ceiling_control::SupervisorState
  :super ros::object
  :slots (_header _state _active_mode _setpoint_stream_allowed _motion_allowed _mavros_connected _armed _offboard _operator_fresh _odom_fresh _ceiling_range_fresh _wall_range_fresh _fault_mask _reason ))

(defmethod px4_wall_ceiling_control::SupervisorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:active_mode __active_mode) 0)
    ((:setpoint_stream_allowed __setpoint_stream_allowed) nil)
    ((:motion_allowed __motion_allowed) nil)
    ((:mavros_connected __mavros_connected) nil)
    ((:armed __armed) nil)
    ((:offboard __offboard) nil)
    ((:operator_fresh __operator_fresh) nil)
    ((:odom_fresh __odom_fresh) nil)
    ((:ceiling_range_fresh __ceiling_range_fresh) nil)
    ((:wall_range_fresh __wall_range_fresh) nil)
    ((:fault_mask __fault_mask) 0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _active_mode (round __active_mode))
   (setq _setpoint_stream_allowed __setpoint_stream_allowed)
   (setq _motion_allowed __motion_allowed)
   (setq _mavros_connected __mavros_connected)
   (setq _armed __armed)
   (setq _offboard __offboard)
   (setq _operator_fresh __operator_fresh)
   (setq _odom_fresh __odom_fresh)
   (setq _ceiling_range_fresh __ceiling_range_fresh)
   (setq _wall_range_fresh __wall_range_fresh)
   (setq _fault_mask (round __fault_mask))
   (setq _reason (string __reason))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:active_mode
   (&optional __active_mode)
   (if __active_mode (setq _active_mode __active_mode)) _active_mode)
  (:setpoint_stream_allowed
   (&optional (__setpoint_stream_allowed :null))
   (if (not (eq __setpoint_stream_allowed :null)) (setq _setpoint_stream_allowed __setpoint_stream_allowed)) _setpoint_stream_allowed)
  (:motion_allowed
   (&optional (__motion_allowed :null))
   (if (not (eq __motion_allowed :null)) (setq _motion_allowed __motion_allowed)) _motion_allowed)
  (:mavros_connected
   (&optional (__mavros_connected :null))
   (if (not (eq __mavros_connected :null)) (setq _mavros_connected __mavros_connected)) _mavros_connected)
  (:armed
   (&optional (__armed :null))
   (if (not (eq __armed :null)) (setq _armed __armed)) _armed)
  (:offboard
   (&optional (__offboard :null))
   (if (not (eq __offboard :null)) (setq _offboard __offboard)) _offboard)
  (:operator_fresh
   (&optional (__operator_fresh :null))
   (if (not (eq __operator_fresh :null)) (setq _operator_fresh __operator_fresh)) _operator_fresh)
  (:odom_fresh
   (&optional (__odom_fresh :null))
   (if (not (eq __odom_fresh :null)) (setq _odom_fresh __odom_fresh)) _odom_fresh)
  (:ceiling_range_fresh
   (&optional (__ceiling_range_fresh :null))
   (if (not (eq __ceiling_range_fresh :null)) (setq _ceiling_range_fresh __ceiling_range_fresh)) _ceiling_range_fresh)
  (:wall_range_fresh
   (&optional (__wall_range_fresh :null))
   (if (not (eq __wall_range_fresh :null)) (setq _wall_range_fresh __wall_range_fresh)) _wall_range_fresh)
  (:fault_mask
   (&optional __fault_mask)
   (if __fault_mask (setq _fault_mask __fault_mask)) _fault_mask)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _state
    1
    ;; uint8 _active_mode
    1
    ;; bool _setpoint_stream_allowed
    1
    ;; bool _motion_allowed
    1
    ;; bool _mavros_connected
    1
    ;; bool _armed
    1
    ;; bool _offboard
    1
    ;; bool _operator_fresh
    1
    ;; bool _odom_fresh
    1
    ;; bool _ceiling_range_fresh
    1
    ;; bool _wall_range_fresh
    1
    ;; uint32 _fault_mask
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
     ;; uint8 _state
       (write-byte _state s)
     ;; uint8 _active_mode
       (write-byte _active_mode s)
     ;; bool _setpoint_stream_allowed
       (if _setpoint_stream_allowed (write-byte -1 s) (write-byte 0 s))
     ;; bool _motion_allowed
       (if _motion_allowed (write-byte -1 s) (write-byte 0 s))
     ;; bool _mavros_connected
       (if _mavros_connected (write-byte -1 s) (write-byte 0 s))
     ;; bool _armed
       (if _armed (write-byte -1 s) (write-byte 0 s))
     ;; bool _offboard
       (if _offboard (write-byte -1 s) (write-byte 0 s))
     ;; bool _operator_fresh
       (if _operator_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _odom_fresh
       (if _odom_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _ceiling_range_fresh
       (if _ceiling_range_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _wall_range_fresh
       (if _wall_range_fresh (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _fault_mask
       (write-long _fault_mask s)
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _active_mode
     (setq _active_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _setpoint_stream_allowed
     (setq _setpoint_stream_allowed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _motion_allowed
     (setq _motion_allowed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _mavros_connected
     (setq _mavros_connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _armed
     (setq _armed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _offboard
     (setq _offboard (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _operator_fresh
     (setq _operator_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _odom_fresh
     (setq _odom_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ceiling_range_fresh
     (setq _ceiling_range_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _wall_range_fresh
     (setq _wall_range_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _fault_mask
     (setq _fault_mask (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::SupervisorState :md5sum-) "1ec3406c7c8a31f63daae31442cabb75")
(setf (get px4_wall_ceiling_control::SupervisorState :datatype-) "px4_wall_ceiling_control/SupervisorState")
(setf (get px4_wall_ceiling_control::SupervisorState :definition-)
      "std_msgs/Header header

uint8 STATE_WAIT_LINK=0
uint8 STATE_MANUAL=1
uint8 STATE_STANDBY=2
uint8 STATE_PRESTREAM=3
uint8 STATE_CEILING_ACTIVE=4
uint8 STATE_WALL_ACTIVE=5
uint8 STATE_FAULT=6

uint8 MODE_NONE=0
uint8 MODE_CEILING=1
uint8 MODE_WALL=2

uint32 FAULT_NONE=0
uint32 FAULT_MAVROS=1
uint32 FAULT_OPERATOR=2
uint32 FAULT_ODOMETRY=4
uint32 FAULT_CEILING_RANGE=8
uint32 FAULT_WALL_RANGE=16
uint32 FAULT_UNKNOWN_MODE=32

uint8 state
uint8 active_mode
bool setpoint_stream_allowed
bool motion_allowed
bool mavros_connected
bool armed
bool offboard
bool operator_fresh
bool odom_fresh
bool ceiling_range_fresh
bool wall_range_fresh
uint32 fault_mask
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



(provide :px4_wall_ceiling_control/SupervisorState "1ec3406c7c8a31f63daae31442cabb75")


