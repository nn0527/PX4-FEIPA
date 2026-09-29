;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::ControlSetpoint)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'ControlSetpoint (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT")
  (make-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))

(in-package "ROS")
;;//! \htmlinclude ControlSetpoint.msg.html
(if (not (find-package "MAVROS_MSGS"))
  (ros::roseus-add-msgs "mavros_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOURCE_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(shadow '*SOURCE_NONE* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(defconstant px4_wall_ceiling_control::ControlSetpoint::*SOURCE_NONE* 0)
(intern "*SOURCE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(shadow '*SOURCE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(defconstant px4_wall_ceiling_control::ControlSetpoint::*SOURCE_CEILING* 1)
(intern "*SOURCE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(shadow '*SOURCE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(defconstant px4_wall_ceiling_control::ControlSetpoint::*SOURCE_WALL* 2)
(intern "*KIND_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(shadow '*KIND_NONE* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(defconstant px4_wall_ceiling_control::ControlSetpoint::*KIND_NONE* 0)
(intern "*KIND_LOCAL*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(shadow '*KIND_LOCAL* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(defconstant px4_wall_ceiling_control::ControlSetpoint::*KIND_LOCAL* 1)
(intern "*KIND_ATTITUDE*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(shadow '*KIND_ATTITUDE* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLSETPOINT"))
(defconstant px4_wall_ceiling_control::ControlSetpoint::*KIND_ATTITUDE* 2)

(defun px4_wall_ceiling_control::ControlSetpoint-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::ControlSetpoint::*SOURCE_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::ControlSetpoint::*SOURCE_CEILING*)
        ((= const 2) 'px4_wall_ceiling_control::ControlSetpoint::*SOURCE_WALL*)
        ((= const 0) 'px4_wall_ceiling_control::ControlSetpoint::*KIND_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::ControlSetpoint::*KIND_LOCAL*)
        ((= const 2) 'px4_wall_ceiling_control::ControlSetpoint::*KIND_ATTITUDE*)
        (t nil)))

(defclass px4_wall_ceiling_control::ControlSetpoint
  :super ros::object
  :slots (_header _source _kind _valid _local _attitude _reason ))

(defmethod px4_wall_ceiling_control::ControlSetpoint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:source __source) 0)
    ((:kind __kind) 0)
    ((:valid __valid) nil)
    ((:local __local) (instance mavros_msgs::PositionTarget :init))
    ((:attitude __attitude) (instance mavros_msgs::AttitudeTarget :init))
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _source (round __source))
   (setq _kind (round __kind))
   (setq _valid __valid)
   (setq _local __local)
   (setq _attitude __attitude)
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
  (:kind
   (&optional __kind)
   (if __kind (setq _kind __kind)) _kind)
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:local
   (&rest __local)
   (if (keywordp (car __local))
       (send* _local __local)
     (progn
       (if __local (setq _local (car __local)))
       _local)))
  (:attitude
   (&rest __attitude)
   (if (keywordp (car __attitude))
       (send* _attitude __attitude)
     (progn
       (if __attitude (setq _attitude (car __attitude)))
       _attitude)))
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
    ;; uint8 _kind
    1
    ;; bool _valid
    1
    ;; mavros_msgs/PositionTarget _local
    (send _local :serialization-length)
    ;; mavros_msgs/AttitudeTarget _attitude
    (send _attitude :serialization-length)
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
     ;; uint8 _kind
       (write-byte _kind s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; mavros_msgs/PositionTarget _local
       (send _local :serialize s)
     ;; mavros_msgs/AttitudeTarget _attitude
       (send _attitude :serialize s)
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
   ;; uint8 _kind
     (setq _kind (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; mavros_msgs/PositionTarget _local
     (send _local :deserialize buf ptr-) (incf ptr- (send _local :serialization-length))
   ;; mavros_msgs/AttitudeTarget _attitude
     (send _attitude :deserialize buf ptr-) (incf ptr- (send _attitude :serialization-length))
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::ControlSetpoint :md5sum-) "905a0ba96daedb050b5d2c986dd2f262")
(setf (get px4_wall_ceiling_control::ControlSetpoint :datatype-) "px4_wall_ceiling_control/ControlSetpoint")
(setf (get px4_wall_ceiling_control::ControlSetpoint :definition-)
      "std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 KIND_NONE=0
uint8 KIND_LOCAL=1
uint8 KIND_ATTITUDE=2

uint8 source
uint8 kind
bool valid
mavros_msgs/PositionTarget local
mavros_msgs/AttitudeTarget attitude
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

================================================================================
MSG: mavros_msgs/PositionTarget
# Message for SET_POSITION_TARGET_LOCAL_NED
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402.

std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

uint16 type_mask
uint16 IGNORE_PX = 1	# Position ignore flags
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8	# Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64	# Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512	# Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: mavros_msgs/AttitudeTarget
# Message for SET_ATTITUDE_TARGET
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402, #418.

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1	# body_rate.x
uint8 IGNORE_PITCH_RATE = 2	# body_rate.y
uint8 IGNORE_YAW_RATE = 4	# body_rate.z
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128	# orientation field

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
float32 thrust

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :px4_wall_ceiling_control/ControlSetpoint "905a0ba96daedb050b5d2c986dd2f262")


