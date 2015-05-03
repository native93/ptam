
(cl:in-package :asdf)

(defsystem "ptam-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :ptam-msg
)
  :components ((:file "_package")
    (:file "move_robot" :depends-on ("_package_move_robot"))
    (:file "_package_move_robot" :depends-on ("_package"))
    (:file "VisibilityCalculator" :depends-on ("_package_VisibilityCalculator"))
    (:file "_package_VisibilityCalculator" :depends-on ("_package"))
    (:file "Frontier_check" :depends-on ("_package_Frontier_check"))
    (:file "_package_Frontier_check" :depends-on ("_package"))
    (:file "trajectory_checker" :depends-on ("_package_trajectory_checker"))
    (:file "_package_trajectory_checker" :depends-on ("_package"))
    (:file "FrontierExtractor" :depends-on ("_package_FrontierExtractor"))
    (:file "_package_FrontierExtractor" :depends-on ("_package"))
    (:file "cluster" :depends-on ("_package_cluster"))
    (:file "_package_cluster" :depends-on ("_package"))
  ))