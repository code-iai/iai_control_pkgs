(in-package :urdf-management)

(defvar *robot-model* nil)
(defvar *urdf-pub* nil)

(def-service-callback AlterUrdf (action xml names)
  (ros-info (urdf-management) "Altering robot description.")
  (let ((success (cond
                   ((eql action (symbol-code 'urdf_management-srv:alterurdf-request :add))
                    (add-to-robot xml))
                   ((eql action (symbol-code 'urdf_management-srv:alterurdf-request :remove))
                    (remove-from-robot names)))))
    (when success (publish-urdf))
    (make-response :success success)))

(defun alter-urdf-service ()
  "Registers the service to alter the robot description."
  (setf *robot-model* (parse-urdf (get-param "robot_description")))
  (setf *urdf-pub* (advertise "/dynamic_robot_description" 'std_msgs-msg:String :latch t))
  (publish-urdf)
  (register-service "alter_urdf" 'AlterUrdf)
  (ros-info (urdf-management) "Ready to alter urdf."))

(defun add-to-robot (xml)
  "Adds the links and joints descripted to the robot model."
  (let ((parsed-xml (s-xml:parse-xml-string (format nil "<container>~a</container>" xml)
                                            :output-type :xml-struct))
        (link-descriptions nil)
        (joint-descriptions nil)
        (new-links nil)
        (new-joints nil))

    ;; Get the descriptions of the joints and links from the xml
    (dolist (child (s-xml:xml-element-children parsed-xml))
      (case (s-xml:xml-element-name child)
        (:|link| (push child link-descriptions))
        (:|joint| (push child joint-descriptions))))

    ;; Create the links from the description
    (dolist (link-desc link-descriptions)
      (let ((link (create-link link-desc)))
        (when (not link)
          (ros-error (urdf-management) "Invalid link description: ~a" link-desc)
          (return-from add-to-robot nil))
        (push link new-links)))

    ;; Add the new links to the robot-model
    (dolist (link new-links)
      (add-link-to-robot link))
           
    ;; Create the joints from the description
    (dolist (joint-desc joint-descriptions)
      (let ((joint (create-joint joint-desc)))
        (when (not joint)
          (ros-error (urdf-management) "Invalid joint description: ~a" joint-desc)
          (return-from add-to-robot nil))
        (push joint new-joints)))
    
    ;; Add the new joints to the robot-model
    (dolist (joint new-joints)
      (add-joint-to-robot joint)))

  t)

(defun create-link (link-desc)
  "Parses the xml description of the link. If it's a valid link the link is returned else nil."
  (let ((link (cl-urdf::parse-xml-node :|link| link-desc)))
    link))

(defun add-link-to-robot (link)
  "Adds the `link' to the robot model."
  (let ((link-table (links *robot-model*)))
    (setf (gethash (name link) link-table) link)))

(defun create-joint (joint-desc)
  "Parses the xml description of the joint. If it's a valid joint the joint is returned else nil."
  (let ((joint (cl-urdf::parse-xml-node :|joint| joint-desc *robot-model*)))
    joint))

(defun add-joint-to-robot (joint)
  "Adds the `joint' to the robot model."
  (let ((joint-table (joints *robot-model*)))
    (setf (gethash (name joint) joint-table) joint)))

(defun remove-from-robot (names)
  "Searches for links and joints with the given names in the robot model and removes them."
  (let ((link-table (links *robot-model*))
        (joint-table (joints *robot-model*)))
    (map 'vector (lambda (name)
                   (remhash name link-table)
                   (remhash name joint-table))
         names)
    t))

(defun publish-urdf ()
  "Publsihes generates an urdf description from the robot model and publishes it."
  (publish-msg *urdf-pub* :data (generate-urdf-string *robot-model*)))