

(defsystem urdf-management
  :author "Jannik Buckelo <jannikbu@cs.uni-bremen.de>"
  :licence "BSD"

  :depends-on (:cl-urdf
               :roslisp
               :urdf_management-srv
               :std_msgs-msg
               :s-xml)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "service" :depends-on ("package"))))))
