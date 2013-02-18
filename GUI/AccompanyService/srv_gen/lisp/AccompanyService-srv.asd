
(cl:in-package :asdf)

(defsystem "AccompanyService-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "db_msg" :depends-on ("_package_db_msg"))
    (:file "_package_db_msg" :depends-on ("_package"))
    (:file "AccompanyAction" :depends-on ("_package_AccompanyAction"))
    (:file "_package_AccompanyAction" :depends-on ("_package"))
  ))