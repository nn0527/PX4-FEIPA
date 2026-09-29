;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::ControllerStatus)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'ControllerStatus (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS")
  (make-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ControllerStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOURCE_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))
(shadow '*SOURCE_NONE* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))
(defconstant px4_wall_ceiling_control::ControllerStatus::*SOURCE_NONE* 0)
(intern "*SOURCE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))
(shadow '*SOURCE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))
(defconstant px4_wall_ceiling_control::ControllerStatus::*SOURCE_CEILING* 1)
(intern "*SOURCE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))
(shadow '*SOURCE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::CONTROLLERSTATUS"))
(defconstant px4_wall_ceiling_control::ControllerStatus::*SOURCE_WALL* 2)

(defun px4_wall_ceiling_control::ControllerStatus-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::ControllerStatus::*SOURCE_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::ControllerStatus::*SOURCE_CEILING*)
        ((= const 2) 'px4_wall_ceiling_control::ControllerStatus::*SOURCE_WALL*)
        (t nil)))

(defclass px4_wall_ceiling_control::ControllerStatus
  :super ros::object
  :slots (_header _source _healthy _input_fresh _target_reached _detail ))

(defmethod px4_wall_ceiling_control::ControllerStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:source __source) 0)
    ((:healthy __healthy) nil)
    ((:input_fresh __input_fresh) nil)
    ((:target_reached __target_reached) nil)
    ((:detail __detail) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _source (round __source))
   (setq _healthy __healthy)
   (setq _input_fresh __input_fresh)
   (setq _target_reached __target_reached)
   (setq _detail (string __detail))
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
  (:healthy
   (&optional (__healthy :null))
   (if (not (eq __healthy :null)) (setq _healthy __healthy)) _healthy)
  (:input_fresh
   (&optional (__input_fresh :null))
   (if (not (eq __input_fresh :null)) (setq _input_fresh __input_fresh)) _input_fresh)
  (:target_reached
   (&optional (__target_reached :null))
   (if (not (eq __target_reached :null)) (setq _target_reached __target_reached)) _target_reached)
  (:detail
   (&optional __detail)
   (if __detail (setq _detail __detail)) _detail)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _source
    1
    ;; bool _healthy
    1
    ;; bool _input_fresh
    1
    ;; bool _target_reached
    1
    ;; string _detail
    4 (length _detail)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _source
       (write-byte _source s)
     ;; bool _healthy
       (if _healthy (write-byte -1 s) (write-byte 0 s))
     ;; bool _input_fresh
       (if _input_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _target_reached
       (if _target_reached (write-byte -1 s) (write-byte 0 s))
     ;; string _detail
       (write-long (length _detail) s) (princ _detail s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _source
     (setq _source (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _healthy
     (setq _healthy (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _input_fresh
     (setq _input_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _target_reached
     (setq _target_reached (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _detail
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _detail (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::ControllerStatus :md5sum-) "a4190f296fb56a105a72ae34460a659f")
(setf (get px4_wall_ceiling_control::ControllerStatus :datatype-) "px4_wall_ceiling_control/ControllerStatus")
(setf (get px4_wall_ceiling_control::ControllerStatus :definition-)
      "std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 source
bool healthy
bool input_fresh
bool target_reached
string detail

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



(provide :px4_wall_ceiling_control/ControllerStatus "a4190f296fb56a105a72ae34460a659f")


