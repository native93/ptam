
(cl:in-package :asdf)

(defsystem "ptam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "points_visible" :depends-on ("_package_points_visible"))
    (:file "_package_points_visible" :depends-on ("_package"))
    (:file "map" :depends-on ("_package_map"))
    (:file "_package_map" :depends-on ("_package"))
    (:file "map_info" :depends-on ("_package_map_info"))
    (:file "_package_map_info" :depends-on ("_package"))
    (:file "RandT" :depends-on ("_package_RandT"))
    (:file "_package_RandT" :depends-on ("_package"))
    (:file "point" :depends-on ("_package_point"))
    (:file "_package_point" :depends-on ("_package"))
    (:file "pos_robot" :depends-on ("_package_pos_robot"))
    (:file "_package_pos_robot" :depends-on ("_package"))
  ))