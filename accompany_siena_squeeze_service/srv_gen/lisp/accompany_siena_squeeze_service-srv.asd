
(cl:in-package :asdf)

(defsystem "accompany_siena_squeeze_service-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AccompanyAction" :depends-on ("_package_AccompanyAction"))
    (:file "_package_AccompanyAction" :depends-on ("_package"))
  ))