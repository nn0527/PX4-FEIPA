
(cl:in-package :asdf)

(defsystem "px4_wall_ceiling_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :mavros_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AttachmentControlCandidate" :depends-on ("_package_AttachmentControlCandidate"))
    (:file "_package_AttachmentControlCandidate" :depends-on ("_package"))
    (:file "AttachmentMechanismCommand" :depends-on ("_package_AttachmentMechanismCommand"))
    (:file "_package_AttachmentMechanismCommand" :depends-on ("_package"))
    (:file "AttachmentMechanismStatus" :depends-on ("_package_AttachmentMechanismStatus"))
    (:file "_package_AttachmentMechanismStatus" :depends-on ("_package"))
    (:file "BehaviorCommand" :depends-on ("_package_BehaviorCommand"))
    (:file "_package_BehaviorCommand" :depends-on ("_package"))
    (:file "CeilingAttachmentStatus" :depends-on ("_package_CeilingAttachmentStatus"))
    (:file "_package_CeilingAttachmentStatus" :depends-on ("_package"))
    (:file "ControlSetpoint" :depends-on ("_package_ControlSetpoint"))
    (:file "_package_ControlSetpoint" :depends-on ("_package"))
    (:file "ControllerStatus" :depends-on ("_package_ControllerStatus"))
    (:file "_package_ControllerStatus" :depends-on ("_package"))
    (:file "OperatorCommand" :depends-on ("_package_OperatorCommand"))
    (:file "_package_OperatorCommand" :depends-on ("_package"))
    (:file "SensorHealth" :depends-on ("_package_SensorHealth"))
    (:file "_package_SensorHealth" :depends-on ("_package"))
    (:file "SupervisorState" :depends-on ("_package_SupervisorState"))
    (:file "_package_SupervisorState" :depends-on ("_package"))
    (:file "WallPerchStatus" :depends-on ("_package_WallPerchStatus"))
    (:file "_package_WallPerchStatus" :depends-on ("_package"))
  ))