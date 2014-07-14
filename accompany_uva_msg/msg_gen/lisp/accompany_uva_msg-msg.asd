
(cl:in-package :asdf)

(defsystem "accompany_uva_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "TrackedHuman" :depends-on ("_package_TrackedHuman"))
    (:file "_package_TrackedHuman" :depends-on ("_package"))
    (:file "TrackedHumans" :depends-on ("_package_TrackedHumans"))
    (:file "_package_TrackedHumans" :depends-on ("_package"))
    (:file "Appearance" :depends-on ("_package_Appearance"))
    (:file "_package_Appearance" :depends-on ("_package"))
    (:file "HumanLocationsParticles" :depends-on ("_package_HumanLocationsParticles"))
    (:file "_package_HumanLocationsParticles" :depends-on ("_package"))
    (:file "HumanLocationsParticle" :depends-on ("_package_HumanLocationsParticle"))
    (:file "_package_HumanLocationsParticle" :depends-on ("_package"))
    (:file "HumanLocations" :depends-on ("_package_HumanLocations"))
    (:file "_package_HumanLocations" :depends-on ("_package"))
    (:file "HumanDetection" :depends-on ("_package_HumanDetection"))
    (:file "_package_HumanDetection" :depends-on ("_package"))
    (:file "HumanDetections" :depends-on ("_package_HumanDetections"))
    (:file "_package_HumanDetections" :depends-on ("_package"))
  ))