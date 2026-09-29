;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::BehaviorCommand)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'BehaviorCommand (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND")
  (make-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude BehaviorCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOURCE_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*SOURCE_NONE* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*SOURCE_NONE* 0)
(intern "*SOURCE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*SOURCE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*SOURCE_CEILING* 1)
(intern "*SOURCE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*SOURCE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*SOURCE_WALL* 2)
(intern "*STATE_DISABLED*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*STATE_DISABLED* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*STATE_DISABLED* 0)
(intern "*STATE_APPROACH*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*STATE_APPROACH* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*STATE_APPROACH* 1)
(intern "*STATE_TRACK*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*STATE_TRACK* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*STATE_TRACK* 2)
(intern "*STATE_RETREAT*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*STATE_RETREAT* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*STATE_RETREAT* 3)
(intern "*STATE_FAULT*" (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(shadow '*STATE_FAULT* (find-package "PX4_WALL_CEILING_CONTROL::BEHAVIORCOMMAND"))
(defconstant px4_wall_ceiling_control::BehaviorCommand::*STATE_FAULT* 4)

(defun px4_wall_ceiling_control::BehaviorCommand-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::BehaviorCommand::*SOURCE_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::BehaviorCommand::*SOURCE_CEILING*)
        ((= const 2) 'px4_wall_ceiling_control::BehaviorCommand::*SOURCE_WALL*)
        ((= const 0) 'px4_wall_ceiling_control::BehaviorCommand::*STATE_DISABLED*)
        ((= const 1) 'px4_wall_ceiling_control::BehaviorCommand::*STATE_APPROACH*)
        ((= const 2) 'px4_wall_ceiling_control::BehaviorCommand::*STATE_TRACK*)
        ((= const 3) 'px4_wall_ceiling_control::BehaviorCommand::*STATE_RETREAT*)
        ((= const 4) 'px4_wall_ceiling_control::BehaviorCommand::*STATE_FAULT*)
        (t nil)))

(defclass px4_wall_ceiling_control::BehaviorCommand
  :super ros::object
  :slots (_header _source _state _active _target_distance_m _max_speed_mps _reason ))

(defmethod px4_wall_ceiling_control::BehaviorCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:source __source) 0)
    ((:state __state) 0)
    ((:active __active) nil)
    ((:target_distance_m __target_distance_m) 0.0)
    ((:max_speed_mps __max_speed_mps) 0.0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _source (round __source))
   (setq _state (round __state))
   (setq _active __active)
   (setq _target_distance_m (float __target_distance_m))
   (setq _max_speed_mps (float __max_speed_mps))
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
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:active
   (&optional (__active :null))
   (if (not (eq __active :null)) (setq _active __active)) _active)
  (:target_distance_m
   (&optional __target_distance_m)
   (if __target_distance_m (setq _target_distance_m __target_distance_m)) _target_distance_m)
  (:max_speed_mps
   (&optional __max_speed_mps)
   (if __max_speed_mps (setq _max_speed_mps __max_speed_mps)) _max_speed_mps)
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
    ;; uint8 _state
    1
    ;; bool _active
    1
    ;; float32 _target_distance_m
    4
    ;; float32 _max_speed_mps
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
     ;; uint8 _state
       (write-byte _state s)
     ;; bool _active
       (if _active (write-byte -1 s) (write-byte 0 s))
     ;; float32 _target_distance_m
       (sys::poke _target_distance_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_speed_mps
       (sys::poke _max_speed_mps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _active
     (setq _active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _target_distance_m
     (setq _target_distance_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_speed_mps
     (setq _max_speed_mps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::BehaviorCommand :md5sum-) "c5b91608717c6c01656752aca3ecaa53")
(setf (get px4_wall_ceiling_control::BehaviorCommand :datatype-) "px4_wall_ceiling_control/BehaviorCommand")
(setf (get px4_wall_ceiling_control::BehaviorCommand :definition-)
      "std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 STATE_DISABLED=0
uint8 STATE_APPROACH=1
uint8 STATE_TRACK=2
uint8 STATE_RETREAT=3
uint8 STATE_FAULT=4

uint8 source
uint8 state
bool active
float32 target_distance_m
float32 max_speed_mps
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



(provide :px4_wall_ceiling_control/BehaviorCommand "c5b91608717c6c01656752aca3ecaa53")


