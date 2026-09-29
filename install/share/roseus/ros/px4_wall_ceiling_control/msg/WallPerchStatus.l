;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::WallPerchStatus)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'WallPerchStatus (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS")
  (make-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))

(in-package "ROS")
;;//! \htmlinclude WallPerchStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_IDLE*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_IDLE* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_IDLE* 0)
(intern "*STATE_FRONT_WALL_DETECT*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_FRONT_WALL_DETECT* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_FRONT_WALL_DETECT* 1)
(intern "*STATE_STABILIZE_HOVER*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_STABILIZE_HOVER* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_STABILIZE_HOVER* 2)
(intern "*STATE_SLOW_APPROACH*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_SLOW_APPROACH* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_SLOW_APPROACH* 3)
(intern "*STATE_FLIP_TO_WALL*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_FLIP_TO_WALL* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_FLIP_TO_WALL* 4)
(intern "*STATE_WALL_CAPTURE*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_WALL_CAPTURE* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_WALL_CAPTURE* 5)
(intern "*STATE_WALL_HOLD*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_WALL_HOLD* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_WALL_HOLD* 6)
(intern "*STATE_WALL_PIN*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_WALL_PIN* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_WALL_PIN* 7)
(intern "*STATE_DETACH_ROTATE*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_DETACH_ROTATE* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_DETACH_ROTATE* 8)
(intern "*STATE_RECOVER*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_RECOVER* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_RECOVER* 9)
(intern "*STATE_EXIT*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_EXIT* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_EXIT* 10)
(intern "*STATE_ABORT*" (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(shadow '*STATE_ABORT* (find-package "PX4_WALL_CEILING_CONTROL::WALLPERCHSTATUS"))
(defconstant px4_wall_ceiling_control::WallPerchStatus::*STATE_ABORT* 11)

(defun px4_wall_ceiling_control::WallPerchStatus-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_IDLE*)
        ((= const 1) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_FRONT_WALL_DETECT*)
        ((= const 2) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_STABILIZE_HOVER*)
        ((= const 3) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_SLOW_APPROACH*)
        ((= const 4) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_FLIP_TO_WALL*)
        ((= const 5) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_WALL_CAPTURE*)
        ((= const 6) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_WALL_HOLD*)
        ((= const 7) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_WALL_PIN*)
        ((= const 8) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_DETACH_ROTATE*)
        ((= const 9) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_RECOVER*)
        ((= const 10) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_EXIT*)
        ((= const 11) 'px4_wall_ceiling_control::WallPerchStatus::*STATE_ABORT*)
        (t nil)))

(defclass px4_wall_ceiling_control::WallPerchStatus
  :super ros::object
  :slots (_header _state _state_name _enabled _acquire_request _keep_ownership _candidate_valid _front_range_fresh _top_range_fresh _sensor_health_fresh _top_preflight_ready _odom_fresh _flight_state_fresh _front_ready _flip_ready _top_contact_ready _top_clear_ready _fault_detected _neutral_setpoint_requested _attitude_setpoint_valid _front_distance_raw_m _front_distance_filtered_m _top_distance_raw_m _top_distance_filtered_m _progress _attitude_setpoint _thrust_normalized _fault_count _reason ))

(defmethod px4_wall_ceiling_control::WallPerchStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:state_name __state_name) "")
    ((:enabled __enabled) nil)
    ((:acquire_request __acquire_request) nil)
    ((:keep_ownership __keep_ownership) nil)
    ((:candidate_valid __candidate_valid) nil)
    ((:front_range_fresh __front_range_fresh) nil)
    ((:top_range_fresh __top_range_fresh) nil)
    ((:sensor_health_fresh __sensor_health_fresh) nil)
    ((:top_preflight_ready __top_preflight_ready) nil)
    ((:odom_fresh __odom_fresh) nil)
    ((:flight_state_fresh __flight_state_fresh) nil)
    ((:front_ready __front_ready) nil)
    ((:flip_ready __flip_ready) nil)
    ((:top_contact_ready __top_contact_ready) nil)
    ((:top_clear_ready __top_clear_ready) nil)
    ((:fault_detected __fault_detected) nil)
    ((:neutral_setpoint_requested __neutral_setpoint_requested) nil)
    ((:attitude_setpoint_valid __attitude_setpoint_valid) nil)
    ((:front_distance_raw_m __front_distance_raw_m) 0.0)
    ((:front_distance_filtered_m __front_distance_filtered_m) 0.0)
    ((:top_distance_raw_m __top_distance_raw_m) 0.0)
    ((:top_distance_filtered_m __top_distance_filtered_m) 0.0)
    ((:progress __progress) 0.0)
    ((:attitude_setpoint __attitude_setpoint) (instance geometry_msgs::Quaternion :init))
    ((:thrust_normalized __thrust_normalized) 0.0)
    ((:fault_count __fault_count) 0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _state_name (string __state_name))
   (setq _enabled __enabled)
   (setq _acquire_request __acquire_request)
   (setq _keep_ownership __keep_ownership)
   (setq _candidate_valid __candidate_valid)
   (setq _front_range_fresh __front_range_fresh)
   (setq _top_range_fresh __top_range_fresh)
   (setq _sensor_health_fresh __sensor_health_fresh)
   (setq _top_preflight_ready __top_preflight_ready)
   (setq _odom_fresh __odom_fresh)
   (setq _flight_state_fresh __flight_state_fresh)
   (setq _front_ready __front_ready)
   (setq _flip_ready __flip_ready)
   (setq _top_contact_ready __top_contact_ready)
   (setq _top_clear_ready __top_clear_ready)
   (setq _fault_detected __fault_detected)
   (setq _neutral_setpoint_requested __neutral_setpoint_requested)
   (setq _attitude_setpoint_valid __attitude_setpoint_valid)
   (setq _front_distance_raw_m (float __front_distance_raw_m))
   (setq _front_distance_filtered_m (float __front_distance_filtered_m))
   (setq _top_distance_raw_m (float __top_distance_raw_m))
   (setq _top_distance_filtered_m (float __top_distance_filtered_m))
   (setq _progress (float __progress))
   (setq _attitude_setpoint __attitude_setpoint)
   (setq _thrust_normalized (float __thrust_normalized))
   (setq _fault_count (round __fault_count))
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
  (:state_name
   (&optional __state_name)
   (if __state_name (setq _state_name __state_name)) _state_name)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:acquire_request
   (&optional (__acquire_request :null))
   (if (not (eq __acquire_request :null)) (setq _acquire_request __acquire_request)) _acquire_request)
  (:keep_ownership
   (&optional (__keep_ownership :null))
   (if (not (eq __keep_ownership :null)) (setq _keep_ownership __keep_ownership)) _keep_ownership)
  (:candidate_valid
   (&optional (__candidate_valid :null))
   (if (not (eq __candidate_valid :null)) (setq _candidate_valid __candidate_valid)) _candidate_valid)
  (:front_range_fresh
   (&optional (__front_range_fresh :null))
   (if (not (eq __front_range_fresh :null)) (setq _front_range_fresh __front_range_fresh)) _front_range_fresh)
  (:top_range_fresh
   (&optional (__top_range_fresh :null))
   (if (not (eq __top_range_fresh :null)) (setq _top_range_fresh __top_range_fresh)) _top_range_fresh)
  (:sensor_health_fresh
   (&optional (__sensor_health_fresh :null))
   (if (not (eq __sensor_health_fresh :null)) (setq _sensor_health_fresh __sensor_health_fresh)) _sensor_health_fresh)
  (:top_preflight_ready
   (&optional (__top_preflight_ready :null))
   (if (not (eq __top_preflight_ready :null)) (setq _top_preflight_ready __top_preflight_ready)) _top_preflight_ready)
  (:odom_fresh
   (&optional (__odom_fresh :null))
   (if (not (eq __odom_fresh :null)) (setq _odom_fresh __odom_fresh)) _odom_fresh)
  (:flight_state_fresh
   (&optional (__flight_state_fresh :null))
   (if (not (eq __flight_state_fresh :null)) (setq _flight_state_fresh __flight_state_fresh)) _flight_state_fresh)
  (:front_ready
   (&optional (__front_ready :null))
   (if (not (eq __front_ready :null)) (setq _front_ready __front_ready)) _front_ready)
  (:flip_ready
   (&optional (__flip_ready :null))
   (if (not (eq __flip_ready :null)) (setq _flip_ready __flip_ready)) _flip_ready)
  (:top_contact_ready
   (&optional (__top_contact_ready :null))
   (if (not (eq __top_contact_ready :null)) (setq _top_contact_ready __top_contact_ready)) _top_contact_ready)
  (:top_clear_ready
   (&optional (__top_clear_ready :null))
   (if (not (eq __top_clear_ready :null)) (setq _top_clear_ready __top_clear_ready)) _top_clear_ready)
  (:fault_detected
   (&optional (__fault_detected :null))
   (if (not (eq __fault_detected :null)) (setq _fault_detected __fault_detected)) _fault_detected)
  (:neutral_setpoint_requested
   (&optional (__neutral_setpoint_requested :null))
   (if (not (eq __neutral_setpoint_requested :null)) (setq _neutral_setpoint_requested __neutral_setpoint_requested)) _neutral_setpoint_requested)
  (:attitude_setpoint_valid
   (&optional (__attitude_setpoint_valid :null))
   (if (not (eq __attitude_setpoint_valid :null)) (setq _attitude_setpoint_valid __attitude_setpoint_valid)) _attitude_setpoint_valid)
  (:front_distance_raw_m
   (&optional __front_distance_raw_m)
   (if __front_distance_raw_m (setq _front_distance_raw_m __front_distance_raw_m)) _front_distance_raw_m)
  (:front_distance_filtered_m
   (&optional __front_distance_filtered_m)
   (if __front_distance_filtered_m (setq _front_distance_filtered_m __front_distance_filtered_m)) _front_distance_filtered_m)
  (:top_distance_raw_m
   (&optional __top_distance_raw_m)
   (if __top_distance_raw_m (setq _top_distance_raw_m __top_distance_raw_m)) _top_distance_raw_m)
  (:top_distance_filtered_m
   (&optional __top_distance_filtered_m)
   (if __top_distance_filtered_m (setq _top_distance_filtered_m __top_distance_filtered_m)) _top_distance_filtered_m)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
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
  (:fault_count
   (&optional __fault_count)
   (if __fault_count (setq _fault_count __fault_count)) _fault_count)
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
    ;; string _state_name
    4 (length _state_name)
    ;; bool _enabled
    1
    ;; bool _acquire_request
    1
    ;; bool _keep_ownership
    1
    ;; bool _candidate_valid
    1
    ;; bool _front_range_fresh
    1
    ;; bool _top_range_fresh
    1
    ;; bool _sensor_health_fresh
    1
    ;; bool _top_preflight_ready
    1
    ;; bool _odom_fresh
    1
    ;; bool _flight_state_fresh
    1
    ;; bool _front_ready
    1
    ;; bool _flip_ready
    1
    ;; bool _top_contact_ready
    1
    ;; bool _top_clear_ready
    1
    ;; bool _fault_detected
    1
    ;; bool _neutral_setpoint_requested
    1
    ;; bool _attitude_setpoint_valid
    1
    ;; float32 _front_distance_raw_m
    4
    ;; float32 _front_distance_filtered_m
    4
    ;; float32 _top_distance_raw_m
    4
    ;; float32 _top_distance_filtered_m
    4
    ;; float32 _progress
    4
    ;; geometry_msgs/Quaternion _attitude_setpoint
    (send _attitude_setpoint :serialization-length)
    ;; float32 _thrust_normalized
    4
    ;; uint32 _fault_count
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
     ;; string _state_name
       (write-long (length _state_name) s) (princ _state_name s)
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _acquire_request
       (if _acquire_request (write-byte -1 s) (write-byte 0 s))
     ;; bool _keep_ownership
       (if _keep_ownership (write-byte -1 s) (write-byte 0 s))
     ;; bool _candidate_valid
       (if _candidate_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _front_range_fresh
       (if _front_range_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _top_range_fresh
       (if _top_range_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _sensor_health_fresh
       (if _sensor_health_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _top_preflight_ready
       (if _top_preflight_ready (write-byte -1 s) (write-byte 0 s))
     ;; bool _odom_fresh
       (if _odom_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _flight_state_fresh
       (if _flight_state_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _front_ready
       (if _front_ready (write-byte -1 s) (write-byte 0 s))
     ;; bool _flip_ready
       (if _flip_ready (write-byte -1 s) (write-byte 0 s))
     ;; bool _top_contact_ready
       (if _top_contact_ready (write-byte -1 s) (write-byte 0 s))
     ;; bool _top_clear_ready
       (if _top_clear_ready (write-byte -1 s) (write-byte 0 s))
     ;; bool _fault_detected
       (if _fault_detected (write-byte -1 s) (write-byte 0 s))
     ;; bool _neutral_setpoint_requested
       (if _neutral_setpoint_requested (write-byte -1 s) (write-byte 0 s))
     ;; bool _attitude_setpoint_valid
       (if _attitude_setpoint_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _front_distance_raw_m
       (sys::poke _front_distance_raw_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_distance_filtered_m
       (sys::poke _front_distance_filtered_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _top_distance_raw_m
       (sys::poke _top_distance_raw_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _top_distance_filtered_m
       (sys::poke _top_distance_filtered_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _progress
       (sys::poke _progress (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Quaternion _attitude_setpoint
       (send _attitude_setpoint :serialize s)
     ;; float32 _thrust_normalized
       (sys::poke _thrust_normalized (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _fault_count
       (write-long _fault_count s)
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
   ;; string _state_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _acquire_request
     (setq _acquire_request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _keep_ownership
     (setq _keep_ownership (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _candidate_valid
     (setq _candidate_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _front_range_fresh
     (setq _front_range_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _top_range_fresh
     (setq _top_range_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _sensor_health_fresh
     (setq _sensor_health_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _top_preflight_ready
     (setq _top_preflight_ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _odom_fresh
     (setq _odom_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flight_state_fresh
     (setq _flight_state_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _front_ready
     (setq _front_ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flip_ready
     (setq _flip_ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _top_contact_ready
     (setq _top_contact_ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _top_clear_ready
     (setq _top_clear_ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fault_detected
     (setq _fault_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _neutral_setpoint_requested
     (setq _neutral_setpoint_requested (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _attitude_setpoint_valid
     (setq _attitude_setpoint_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _front_distance_raw_m
     (setq _front_distance_raw_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_distance_filtered_m
     (setq _front_distance_filtered_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _top_distance_raw_m
     (setq _top_distance_raw_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _top_distance_filtered_m
     (setq _top_distance_filtered_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _progress
     (setq _progress (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Quaternion _attitude_setpoint
     (send _attitude_setpoint :deserialize buf ptr-) (incf ptr- (send _attitude_setpoint :serialization-length))
   ;; float32 _thrust_normalized
     (setq _thrust_normalized (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _fault_count
     (setq _fault_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::WallPerchStatus :md5sum-) "14c66e7871a6c1f7a0bda70d194bcb80")
(setf (get px4_wall_ceiling_control::WallPerchStatus :datatype-) "px4_wall_ceiling_control/WallPerchStatus")
(setf (get px4_wall_ceiling_control::WallPerchStatus :definition-)
      "std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_FRONT_WALL_DETECT=1
uint8 STATE_STABILIZE_HOVER=2
uint8 STATE_SLOW_APPROACH=3
uint8 STATE_FLIP_TO_WALL=4
uint8 STATE_WALL_CAPTURE=5
uint8 STATE_WALL_HOLD=6
uint8 STATE_WALL_PIN=7
uint8 STATE_DETACH_ROTATE=8
uint8 STATE_RECOVER=9
uint8 STATE_EXIT=10
uint8 STATE_ABORT=11

uint8 state
string state_name
bool enabled
bool acquire_request
bool keep_ownership
bool candidate_valid
bool front_range_fresh
bool top_range_fresh
bool sensor_health_fresh
bool top_preflight_ready
bool odom_fresh
bool flight_state_fresh
bool front_ready
bool flip_ready
bool top_contact_ready
bool top_clear_ready
bool fault_detected
bool neutral_setpoint_requested
bool attitude_setpoint_valid
float32 front_distance_raw_m
float32 front_distance_filtered_m
float32 top_distance_raw_m
float32 top_distance_filtered_m
float32 progress
geometry_msgs/Quaternion attitude_setpoint
# MAVROS AttitudeTarget convention: positive normalized collective thrust.
float32 thrust_normalized
uint32 fault_count
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



(provide :px4_wall_ceiling_control/WallPerchStatus "14c66e7871a6c1f7a0bda70d194bcb80")


