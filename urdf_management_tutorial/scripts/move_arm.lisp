
(REQUIRE :ASDF)
(DEFMETHOD ASDF:PERFORM :AROUND ((O ASDF:LOAD-OP) (C ASDF:CL-SOURCE-FILE))
  (HANDLER-CASE (CALL-NEXT-METHOD O C)
                (INVALID-FASL NIL
                 (ASDF:PERFORM (MAKE-INSTANCE 'ASDF:COMPILE-OP) C)
                 (CALL-NEXT-METHOD))))
(PUSH :ROSLISP-STANDALONE-EXECUTABLE *FEATURES*)
(DECLAIM (MUFFLE-CONDITIONS COMPILER-NOTE))
(PUSH #P"/home/jannikbu/actionlib_ws/src/roslisp/asdf/" ASDF:*CENTRAL-REGISTRY*)
(DEFUN ROSLISP-DEBUGGER-HOOK (CONDITION ME)
  (DECLARE (IGNORE ME))
  (FLET ((FAILURE-QUIT (&KEY RECKLESSLY-P)
           (QUIT :UNIX-STATUS 1 :RECKLESSLY-P RECKLESSLY-P)))
    (HANDLER-CASE
     (PROGN
      (FORMAT *ERROR-OUTPUT* "~&Roslisp exiting due to condition: ~a~&"
              CONDITION)
      (FINISH-OUTPUT *ERROR-OUTPUT*)
      (FAILURE-QUIT))
     (CONDITION NIL (FAILURE-QUIT :RECKLESSLY-P T)))))
(UNLESS
    (LET ((V (POSIX-GETENV "ROSLISP_BACKTRACE_ON_ERRORS")))
      (AND (STRINGP V) (> (LENGTH V) 0)))
  (SETQ *INVOKE-DEBUGGER-HOOK* #'ROSLISP-DEBUGGER-HOOK))
(HANDLER-BIND ((STYLE-WARNING #'MUFFLE-WARNING) (WARNING #'PRINT))
  (ASDF:OPERATE 'ASDF:LOAD-OP :ROS-LOAD-MANIFEST :VERBOSE NIL)
  (SETF (SYMBOL-VALUE (INTERN "*CURRENT-ROS-PACKAGE*" :ROS-LOAD))
          "urdf_management_tutorial")
  (LET ((*STANDARD-OUTPUT* (MAKE-BROADCAST-STREAM))
        (SYS "urdf-management-tutorial"))
    (HANDLER-CASE (ASDF:OPERATE 'ASDF:LOAD-OP SYS :VERBOSE NIL)
                  (ASDF:MISSING-COMPONENT (C)
                   (ERROR
                    "Couldn't find asdf system (filename ~a.asd and system name ~a) or some dependency.  Original condition was ~a."
                    SYS SYS C))))
  (LOAD
   (MERGE-PATHNAMES
    (MAKE-PATHNAME :NAME "move_arm-init.lisp" :DIRECTORY
                   '(:RELATIVE "roslisp" "urdf_management_tutorial"))
    (FUNCALL (SYMBOL-FUNCTION (INTERN "ROS-HOME" :ROS-LOAD))))
   :IF-DOES-NOT-EXIST NIL)
  (FUNCALL
   (SYMBOL-FUNCTION
    (READ-FROM-STRING "urdf-management-tutorial:move-to-grasp-position")))
  (QUIT))