;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::AttachmentMechanismStatus)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'AttachmentMechanismStatus (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMSTATUS")
  (make-package "PX4_WALL_CEILING_CONTROL::ATTACHMENTMECHANISMSTATUS"))

(in-package "ROS")
;;//! \htmlinclude AttachmentMechanismStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass px4_wall_ceiling_control::AttachmentMechanismStatus
  :super ros::object
  :slots (_header _source _sequence _acknowledged _released _fault _reason ))

(defmethod px4_wall_ceiling_control::AttachmentMechanismStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:source __source) 0)
    ((:sequence __sequence) 0)
    ((:acknowledged __acknowledged) nil)
    ((:released __released) nil)
    ((:fault __fault) nil)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _source (round __source))
   (setq _sequence (round __sequence))
   (setq _acknowledged __acknowledged)
   (setq _released __released)
   (setq _fault __fault)
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
  (:sequence
   (&optional __sequence)
   (if __sequence (setq _sequence __sequence)) _sequence)
  (:acknowledged
   (&optional (__acknowledged :null))
   (if (not (eq __acknowledged :null)) (setq _acknowledged __acknowledged)) _acknowledged)
  (:released
   (&optional (__released :null))
   (if (not (eq __released :null)) (setq _released __released)) _released)
  (:fault
   (&optional (__fault :null))
   (if (not (eq __fault :null)) (setq _fault __fault)) _fault)
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
    ;; uint32 _sequence
    4
    ;; bool _acknowledged
    1
    ;; bool _released
    1
    ;; bool _fault
    1
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
     ;; uint32 _sequence
       (write-long _sequence s)
     ;; bool _acknowledged
       (if _acknowledged (write-byte -1 s) (write-byte 0 s))
     ;; bool _released
       (if _released (write-byte -1 s) (write-byte 0 s))
     ;; bool _fault
       (if _fault (write-byte -1 s) (write-byte 0 s))
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
   ;; uint32 _sequence
     (setq _sequence (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _acknowledged
     (setq _acknowledged (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _released
     (setq _released (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fault
     (setq _fault (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::AttachmentMechanismStatus :md5sum-) "e3810df96eca58328a3f90eb36d48cc3")
(setf (get px4_wall_ceiling_control::AttachmentMechanismStatus :datatype-) "px4_wall_ceiling_control/AttachmentMechanismStatus")
(setf (get px4_wall_ceiling_control::AttachmentMechanismStatus :definition-)
      "std_msgs/Header header

uint8 source
uint32 sequence
bool acknowledged
bool released
bool fault
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



(provide :px4_wall_ceiling_control/AttachmentMechanismStatus "e3810df96eca58328a3f90eb36d48cc3")


