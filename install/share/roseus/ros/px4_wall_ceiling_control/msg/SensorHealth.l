;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::SensorHealth)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'SensorHealth (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::SENSORHEALTH")
  (make-package "PX4_WALL_CEILING_CONTROL::SENSORHEALTH"))

(in-package "ROS")
;;//! \htmlinclude SensorHealth.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass px4_wall_ceiling_control::SensorHealth
  :super ros::object
  :slots (_header _ceiling_valid _front_valid _ceiling_age_s _front_age_s _ceiling_sequence _front_sequence _ceiling_invalid_count _front_invalid_count ))

(defmethod px4_wall_ceiling_control::SensorHealth
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ceiling_valid __ceiling_valid) nil)
    ((:front_valid __front_valid) nil)
    ((:ceiling_age_s __ceiling_age_s) 0.0)
    ((:front_age_s __front_age_s) 0.0)
    ((:ceiling_sequence __ceiling_sequence) 0)
    ((:front_sequence __front_sequence) 0)
    ((:ceiling_invalid_count __ceiling_invalid_count) 0)
    ((:front_invalid_count __front_invalid_count) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ceiling_valid __ceiling_valid)
   (setq _front_valid __front_valid)
   (setq _ceiling_age_s (float __ceiling_age_s))
   (setq _front_age_s (float __front_age_s))
   (setq _ceiling_sequence (round __ceiling_sequence))
   (setq _front_sequence (round __front_sequence))
   (setq _ceiling_invalid_count (round __ceiling_invalid_count))
   (setq _front_invalid_count (round __front_invalid_count))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ceiling_valid
   (&optional (__ceiling_valid :null))
   (if (not (eq __ceiling_valid :null)) (setq _ceiling_valid __ceiling_valid)) _ceiling_valid)
  (:front_valid
   (&optional (__front_valid :null))
   (if (not (eq __front_valid :null)) (setq _front_valid __front_valid)) _front_valid)
  (:ceiling_age_s
   (&optional __ceiling_age_s)
   (if __ceiling_age_s (setq _ceiling_age_s __ceiling_age_s)) _ceiling_age_s)
  (:front_age_s
   (&optional __front_age_s)
   (if __front_age_s (setq _front_age_s __front_age_s)) _front_age_s)
  (:ceiling_sequence
   (&optional __ceiling_sequence)
   (if __ceiling_sequence (setq _ceiling_sequence __ceiling_sequence)) _ceiling_sequence)
  (:front_sequence
   (&optional __front_sequence)
   (if __front_sequence (setq _front_sequence __front_sequence)) _front_sequence)
  (:ceiling_invalid_count
   (&optional __ceiling_invalid_count)
   (if __ceiling_invalid_count (setq _ceiling_invalid_count __ceiling_invalid_count)) _ceiling_invalid_count)
  (:front_invalid_count
   (&optional __front_invalid_count)
   (if __front_invalid_count (setq _front_invalid_count __front_invalid_count)) _front_invalid_count)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _ceiling_valid
    1
    ;; bool _front_valid
    1
    ;; float32 _ceiling_age_s
    4
    ;; float32 _front_age_s
    4
    ;; uint32 _ceiling_sequence
    4
    ;; uint32 _front_sequence
    4
    ;; uint32 _ceiling_invalid_count
    4
    ;; uint32 _front_invalid_count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _ceiling_valid
       (if _ceiling_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _front_valid
       (if _front_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _ceiling_age_s
       (sys::poke _ceiling_age_s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_age_s
       (sys::poke _front_age_s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _ceiling_sequence
       (write-long _ceiling_sequence s)
     ;; uint32 _front_sequence
       (write-long _front_sequence s)
     ;; uint32 _ceiling_invalid_count
       (write-long _ceiling_invalid_count s)
     ;; uint32 _front_invalid_count
       (write-long _front_invalid_count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _ceiling_valid
     (setq _ceiling_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _front_valid
     (setq _front_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _ceiling_age_s
     (setq _ceiling_age_s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_age_s
     (setq _front_age_s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _ceiling_sequence
     (setq _ceiling_sequence (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _front_sequence
     (setq _front_sequence (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ceiling_invalid_count
     (setq _ceiling_invalid_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _front_invalid_count
     (setq _front_invalid_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::SensorHealth :md5sum-) "890f31b2daff364b0bc989c3503517bf")
(setf (get px4_wall_ceiling_control::SensorHealth :datatype-) "px4_wall_ceiling_control/SensorHealth")
(setf (get px4_wall_ceiling_control::SensorHealth :definition-)
      "std_msgs/Header header

bool ceiling_valid
bool front_valid

# Age of the last valid sample as measured at this node. Infinity means that
# no valid sample has been received since startup.
float32 ceiling_age_s
float32 front_age_s

# Monotonic counters allow consumers and tests to distinguish real samples
# from repeated status publications.
uint32 ceiling_sequence
uint32 front_sequence
uint32 ceiling_invalid_count
uint32 front_invalid_count

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



(provide :px4_wall_ceiling_control/SensorHealth "890f31b2daff364b0bc989c3503517bf")


