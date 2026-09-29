;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::AttachmentControlCandidate)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'AttachmentControlCandidate (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE")
  (make-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))

(in-package "ROS")
;;//! \htmlinclude AttachmentControlCandidate.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOURCE_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(shadow '*SOURCE_NONE* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(defconstant px4_wall_ceiling_control::AttachmentControlCandidate::*SOURCE_NONE* 0)
(intern "*SOURCE_CEILING*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(shadow '*SOURCE_CEILING* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(defconstant px4_wall_ceiling_control::AttachmentControlCandidate::*SOURCE_CEILING* 1)
(intern "*SOURCE_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(shadow '*SOURCE_WALL* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(defconstant px4_wall_ceiling_control::AttachmentControlCandidate::*SOURCE_WALL* 2)
(intern "*KIND_NONE*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(shadow '*KIND_NONE* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(defconstant px4_wall_ceiling_control::AttachmentControlCandidate::*KIND_NONE* 0)
(intern "*KIND_VERTICAL_VELOCITY*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(shadow '*KIND_VERTICAL_VELOCITY* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(defconstant px4_wall_ceiling_control::AttachmentControlCandidate::*KIND_VERTICAL_VELOCITY* 1)
(intern "*KIND_ATTITUDE_THRUST*" (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(shadow '*KIND_ATTITUDE_THRUST* (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTCONTROLCANDIDATE"))
(defconstant px4_wall_ceiling_control::AttachmentControlCandidate::*KIND_ATTITUDE_THRUST* 2)

(defun px4_wall_ceiling_control::AttachmentControlCandidate-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::AttachmentControlCandidate::*SOURCE_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::AttachmentControlCandidate::*SOURCE_CEILING*)
        ((= const 2) 'px4_wall_ceiling_control::AttachmentControlCandidate::*SOURCE_WALL*)
        ((= const 0) 'px4_wall_ceiling_control::AttachmentControlCandidate::*KIND_NONE*)
        ((= const 1) 'px4_wall_ceiling_control::AttachmentControlCandidate::*KIND_VERTICAL_VELOCITY*)
        ((= const 2) 'px4_wall_ceiling_control::AttachmentControlCandidate::*KIND_ATTITUDE_THRUST*)
        (t nil)))

(defclass px4_wall_ceiling_control::AttachmentControlCandidate
  :super ros::object
  :slots (_header _source _kind _acquire_request _keep_ownership _valid _velocity_z_enu_mps _attitude_setpoint _thrust_normalized _reason ))

(defmethod px4_wall_ceiling_control::AttachmentControlCandidate
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:source __source) 0)
    ((:kind __kind) 0)
    ((:acquire_request __acquire_request) nil)
    ((:keep_ownership __keep_ownership) nil)
    ((:valid __valid) nil)
    ((:velocity_z_enu_mps __velocity_z_enu_mps) 0.0)
    ((:attitude_setpoint __attitude_setpoint) (instance geometry_msgs::Quaternion :init))
    ((:thrust_normalized __thrust_normalized) 0.0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _source (round __source))
   (setq _kind (round __kind))
   (setq _acquire_request __acquire_request)
   (setq _keep_ownership __keep_ownership)
   (setq _valid __valid)
   (setq _velocity_z_enu_mps (float __velocity_z_enu_mps))
   (setq _attitude_setpoint __attitude_setpoint)
   (setq _thrust_normalized (float __thrust_normalized))
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
  (:acquire_request
   (&optional (__acquire_request :null))
   (if (not (eq __acquire_request :null)) (setq _acquire_request __acquire_request)) _acquire_request)
  (:keep_ownership
   (&optional (__keep_ownership :null))
   (if (not (eq __keep_ownership :null)) (setq _keep_ownership __keep_ownership)) _keep_ownership)
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:velocity_z_enu_mps
   (&optional __velocity_z_enu_mps)
   (if __velocity_z_enu_mps (setq _velocity_z_enu_mps __velocity_z_enu_mps)) _velocity_z_enu_mps)
  (:attitude_setpoint
   (&rest __attitude_setpoint)
   (if (keywordp (car __attitude_setpoint))
       (send* _attitude_setpoint __attitude_setpoint)
     (progn
       (if __attitude_setpoint (setq _attitude_setpoint (car __attitude_setpoint)))
       _attitude_setpoint)))
  (:thrust_normalized
   (&optional __thrust_normalized)
   (if __thrust_normalized (setq _thrust_normalized __thrust_normalized)) _thrust_normalized)
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
    ;; bool _acquire_request
    1
    ;; bool _keep_ownership
    1
    ;; bool _valid
    1
    ;; float32 _velocity_z_enu_mps
    4
    ;; geometry_msgs/Quaternion _attitude_setpoint
    (send _attitude_setpoint :serialization-length)
    ;; float32 _thrust_normalized
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
     ;; uint8 _kind
       (write-byte _kind s)
     ;; bool _acquire_request
       (if _acquire_request (write-byte -1 s) (write-byte 0 s))
     ;; bool _keep_ownership
       (if _keep_ownership (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _velocity_z_enu_mps
       (sys::poke _velocity_z_enu_mps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Quaternion _attitude_setpoint
       (send _attitude_setpoint :serialize s)
     ;; float32 _thrust_normalized
       (sys::poke _thrust_normalized (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; bool _acquire_request
     (setq _acquire_request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _keep_ownership
     (setq _keep_ownership (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _velocity_z_enu_mps
     (setq _velocity_z_enu_mps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Quaternion _attitude_setpoint
     (send _attitude_setpoint :deserialize buf ptr-) (incf ptr- (send _attitude_setpoint :serialization-length))
   ;; float32 _thrust_normalized
     (setq _thrust_normalized (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::AttachmentControlCandidate :md5sum-) "a60e02517b779e0ed9645ce45b1715de")
(setf (get px4_wall_ceiling_control::AttachmentControlCandidate :datatype-) "px4_wall_ceiling_control/AttachmentControlCandidate")
(setf (get px4_wall_ceiling_control::AttachmentControlCandidate :definition-)
      "std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_CEILING=1
uint8 SOURCE_WALL=2

uint8 KIND_NONE=0
uint8 KIND_VERTICAL_VELOCITY=1
uint8 KIND_ATTITUDE_THRUST=2

uint8 source
uint8 kind
bool acquire_request
bool keep_ownership
bool valid

# ROS-side ENU world convention.
float32 velocity_z_enu_mps
# ROS-side ENU/FLU attitude convention; MAVROS performs PX4 conversion.
geometry_msgs/Quaternion attitude_setpoint
# Positive normalized collective thrust expected by mavros_msgs/AttitudeTarget.
float32 thrust_normalized
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
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :px4_wall_ceiling_control/AttachmentControlCandidate "a60e02517b779e0ed9645ce45b1715de")


