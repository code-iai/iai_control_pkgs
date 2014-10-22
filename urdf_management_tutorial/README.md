# urdf_management tutorial


## Installation
First, make sure you have the following packages installed:
  * apt-get install TODO figure out which packages are needed

Chekcout the following repos into your workspace:
  * https://github.com/code-iai/iai_control_pkgs
  * https://github.com/code-iai/iai_common_msgs
  * https://github.com/cram-code/cram_physics.git

Additional repos required for this tutorial:
  * https://github.com/cram-code/cram_bridge.git

Build them all by running ```catkin_make```


## Start-up
Start a roscore:
  * ```roscore```

Start controller for the joints and set the robot_description parameter to the pr2.urdf:
  * ```roslaunch loopback_controller_manager_examples pr2_all_controllers_simulation_dynamic_state.launch```

TODO should this be replaced with a start script
Start the repl: 
  * ```rosrun roslisp_repl repl```

To start the urdf_management action server type the following into the repl:
  * ```,```
  * ```roslisp-load-system```
  * ...
  * ```(start-ros-node "urdf_management")```
  * ```(alter-urdf-server)```

Start rviz:
  * ```rosrun rviz rviz```

In rviz,
  * set the fixed frame to ```base_link```
  * add a plugin of type ```DynamicRobotModel```

You hould see the PR2 in rviz:
![rviz view](https://raw.github.com/jannikb/iai_control_pkgs/urdf/urdf_management_tutorial/doc/pr2.png)


## Adding a link
Start a new repl:
  * ```roslisp_repl```
In the repl:
  * load urdf-management
  * load actionlib-lisp
```lisp
(roslisp:start-ros-node "client")
(defparameter *client* (actionlib-lisp:make-simple-action-client "/alterurdf" "iai_urdf_msgs/AlterUrdfAction"))
(actionlib-lisp:send-goal *client* (roslisp:make-message "iai_urdf_msgs/AlterUrdfGoal" action 1 xml_elements_to_add 
"<link name=\"spatula\"><visual><origin rpy=\"0 0 0 \" xyz=\"0 0 0\" /><geometry><mesh filename=\"package://urdf_management/meshes/kitchen/hand-tools/edeka_spatula1.dae\" /></geometry></visual></link><joint name=\"joint_spatula\" type=\"fixed\"><parent link=\"l_gripper_r_finger_tip_link\" /><child link=\"spatula\" /><origin rpy=\"-1.57 0 0.5\" xyz=\"0.22 0 0\"/></joint>")
                          :done-cb #'(lambda (state result)
                                       (format t "Done.~%State: ~a~%Result: ~a~%" state result))
                          :active-cb #'(lambda () (format t "Active.~%"))
                          :feedback-cb #'(lambda (x) (format t "feed-cb:~a~%" x)))
```

In rviz you should now be able to see a spatula in the left gripper of the PR2.
![rviz view](https://raw.github.com/jannikb/iai_control_pkgs/urdf/urdf_management_tutorial/doc/pr2_with_spatula.png)

Start a new repl:
  * ```roslisp_repl```
In the repl:
  * load cram_pr2_controller
```lisp
(roslisp:start-ros-node "client")
(defun make-joint-state-list (joint-names joint-positions)
  "Takes a list of `joint-names` and a list of `joint-positions`, and
returns a list joint-states. Input lists need to be of equal length."
  (mapcar (lambda (name position)
            (cl-robot-models:make-joint-state :name name :position position))
          joint-names joint-positions))
(defparameter *l-arm-joint-names*
  '("l_upper_arm_roll_joint"
    "l_shoulder_pan_joint"
    "l_shoulder_lift_joint"
    "l_forearm_roll_joint"
    "l_elbow_flex_joint"
    "l_wrist_flex_joint"
    "l_wrist_roll_joint"))
(defparameter *l-arm-grasping-configuration*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *l-arm-joint-names*
    '(0.593 1.265 0.964 0.524 -2.1 -0.067 4.419))))
(defparameter *pr2-controller* (make-pr2-arm-position-controller-handle "/l_arm_controller/joint_trajectory_action" *l-arm-joint-names*))
(move-arm *pr2-controller* *l-arm-grasping-configuration* 4.0)
```

Now you should see the left arm moving with the spatula in the gripper.

## Removing a link
Start a new repl:
  * ```roslisp_repl```
In the repl:
  * load urdf-management
  * load actionlib-lisp
```lisp
(roslisp:start-ros-node "client")
(defparameter *client* (actionlib-lisp:make-simple-action-client "/alterurdf" "iai_urdf_msgs/AlterUrdfAction"))
(actionlib-lisp:send-goal *client* (roslisp:make-message "iai_urdf_msgs/AlterUrdfGoal" action 2 element_names_to_remove (vector "joint_spatula" "spatula"))
                          :done-cb #'(lambda (state result)
                                       (format t "Done.~%State: ~a~%Result: ~a~%" state result))
                          :active-cb #'(lambda () (format t "Active.~%"))
                          :feedback-cb #'(lambda (x) (format t "feed-cb:~a~%" x)))
```