# Grab Coke Project

### project video:  https://youtu.be/7hkFugC8I-g

### Quickstart

First, catkin_make the workspace

	 cd 01_ws
	 catkin_make

Second,  next is sim into by launching baxter.sh and open the baxter gazebo and baxter world:

	 ./baxter.sh sim
	 roslaunch baxter_gazebo baxter_world.launch
	 
Then, add the table and coke:

	 roslaunch exmpl_models add_table_and_coke.launch

and then launch the grabber nodes

	 roslaunch baxter_launch_files baxter_object_grabber_nodes.launch

Last,

	 rosrun object_grabber example_object_grabber_action_client


### Function
This robot is single function,  can only grab the coke can from the table and put it down to the preset position.

### Overview

#### A. Correct Gazebo World

#### 1. Add coke model:

	- Add coke model file in **learning_ros_noetic/exmpl_models/**
	- Add launch file in order to launch coke. **add_coke.launch**

- Add coke launch to world launch file **baxter_on_mobot.launch**

  ```xml
  <!-- add tables and blocks -->
     <include file="$(find exmpl_models)/launch/add_table.launch"> </include>
     <include file="$(find exmpl_models)/launch/add_table2.launch"> </include>
     <include file="$(find exmpl_models)/launch/add_coke.launch"> </include>
     <include file="$(find exmpl_models)/launch/add_coke2.launch"> </include>
  ```

- launch **roslaunch baxter_variations baxter_on_mobot.launch**



#### B. Find(Coke) can

#### 1. Modify object_finder

- **object_finder_as.cpp**: Change the object height **0.15**; And modify **find_upright_coke_can** function.

  ```c++
  bool ObjectFinder::find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose) {
     Eigen::Vector3f plane_normal;
      double plane_dist;
      //bool valid_plane;
      Eigen::Vector3f major_axis;
      Eigen::Vector3f centroid;
      bool found_object = true; //should verify this
      double block_height = 0.15; //this height is specific to the coke can model
      //if insufficient points in plane, find_plane_fit returns "false"
      //should do more sanity testing on found_object status
      
      found_object = pclUtils_.find_plane_fit(0.3, 0.9, -0.7, 0.3, surface_height + 0.025, surface_height + 0.045, 0.001,
              plane_normal, plane_dist, major_axis, centroid);
    
      if (plane_normal(2) < 0) plane_normal(2) *= -1.0; //in world frame, normal must point UP
      Eigen::Matrix3f R;
      Eigen::Vector3f y_vec;
      R.col(0) = major_axis;
      R.col(2) = plane_normal;
      R.col(1) = plane_normal.cross(major_axis);
      Eigen::Quaternionf quat(R);
      object_pose.header.frame_id = "base_link";
      object_pose.pose.position.x = centroid(0);
      object_pose.pose.position.y = centroid(1);
      //the coke can model has its origin in the middle of the block, not the top surface
      //so lower the block model origin by half the block height from upper surface
      object_pose.pose.position.z = centroid(2)-0.5*block_height;
      //create R from normal and major axis, then convert R to quaternion
  
      object_pose.pose.orientation.x = quat.x();
      object_pose.pose.orientation.y = quat.y();
      object_pose.pose.orientation.z = quat.z();
      object_pose.pose.orientation.w = quat.w();
      return found_object;
  
  }
  ```



#### C. Grab the can

#### a. Extend object_grabber.action file. add:

```
#object_grabber action message
#pass in an object code and the object's frame (w/rt named frame_id)
#object_grabber will plan approach, grasp and lift of object
#returns codes regarding outcome

int32 UPRIGHT_CYLINDER = 1
int32 GRAB_UPRIGHT_CYLINDER = 1 #synonym
int32 COKE_CAN = 2
int32 GRAB_CORK_CAN= 2 #synonym
int32 GRAB_W_TOOL_Z_APPROACH = 2 #another synonym
# int32 PLACE_UPRIGHT_CYLINDER = 3 #new drop-off command
int32 PLACE_COKE_CAN= 4 #drop-off toy block
int32 DROPOFF_ALONG_TOOL_Z = 4 #synonym

int32 MOVE_FLANGE_TO = 5 #specify flange pose to reach
int32 FINE_MOVE_FLANGE_TO = 6 #precision move to specified flange pose
int32 MOVE_TO_PRE_POSE = 7 # get hand out of way of camera
int32 JSPACE_MOVE_FLANGE_TO = 8 # joint-space move to specified flange pose

int32 CLOSE_GRIPPER = 100  #commands to open/close gripper; should specify test value
int32 OPEN_GRIPPER = 101

#moved these properties to object_manipulation_properties package/library
#float64 TOY_BLOCK_APPROACH_DIST = 0.05
#expect w/ gripper closed on toy block, finger separation should be more than this value:
#float64 TOY_BLOCK_GRIPPER_CLOSE_TEST_VAL = 80.0
```



#### b. Change the strategy to approach(object_grabber.cpp) 

##### 1. add get_grab_poses function

```c++
bool ObjectGrabber::get_grab_poses(int object_id, geometry_msgs::PoseStamped object_pose_stamped){
    //fill in 3 necessary poses: approach, grasp, depart_w_object
    //find out what the default grasp strategy is for this gripper/object combination:
    manip_properties_srv_.request.gripper_ID    = gripper_id_; //this is known from parameter server
    manip_properties_srv_.request.object_ID     = object_id;
    manip_properties_srv_.request.query_code    = object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);

    int n_grasp_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d grasp options for this gripper/object combo; choosing 1st option (default)",n_grasp_strategy_options);
    if (n_grasp_strategy_options<1) return false;
    int grasp_option = object_manipulation_properties::objectManipulationQueryResponse::GRASP_FROM_SIDE;
    // int grasp_option = manip_properties_srv_.response.grasp_strategy_options[1];    // Ue this grasp strategy for finding corresponding grasp pose
    ROS_INFO("chosen grasp strategy is code %d",grasp_option);
    

    // Get Grasp Transforms    
    manip_properties_srv_.request.grasp_option = grasp_option; //from side grasp strategy
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS;

    manip_properties_client_.call(manip_properties_srv_);
    int n_grasp_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_grasp_pose_options<1) {
                    ROS_WARN("no pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);;
                    return false;
                }
    
    grasp_object_pose_wrt_gripper_ = manip_properties_srv_.response.gripper_pose_options[0];
    //ROS_INFO("default grasped pose of object w/rt gripper: ");
    //xformUtils.printPose(grasp_object_pose_wrt_gripper_);
    
    // ------------ Approach strategy/pose
     manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);

    int n_approach_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d approach options for this gripper/object combo; choosing 1st option (default)",n_approach_strategy_options);

    if (n_approach_strategy_options<1) return false;
    // int approach_option = manip_properties_srv_.response.grasp_strategy_options[1];
    int approach_option = object_manipulation_properties::objectManipulationQueryRequest::APPROACH_LATERAL_SLIDE;
    ROS_INFO("chosen approach strategy is code %d",approach_option);
    
    // Use this grasp strategy to find approach pose
    manip_properties_srv_.request.grasp_option = approach_option; //default option for grasp strategy
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS;
    manip_properties_client_.call(manip_properties_srv_);

    int n_approach_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_approach_pose_options<1) {
                    ROS_WARN("no approach pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);
                    ROS_WARN("should not happen--apparent bug in manipulation properties service");
                    return false;
                }   
    approach_object_pose_wrt_gripper_= manip_properties_srv_.response.gripper_pose_options[0]; //using the 0'th, i.e. default option
    //ROS_INFO("default approach pose, expressed as pose of object w/rt gripper at approach: ");
    //xformUtils.printPose(approach_object_pose_wrt_gripper_);   


    // ------------ Depart strategy/pose
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY;
    manip_properties_client_.call(manip_properties_srv_);
    
    int n_depart_strategy_options = manip_properties_srv_.response.grasp_strategy_options.size();
    ROS_INFO("there are %d depart options for this gripper/object combo; choosing 1st option (default)",n_depart_strategy_options);

    if (n_depart_strategy_options<1) return false;
    // int depart_option = manip_properties_srv_.response.grasp_strategy_options[0];
    int depart_option = object_manipulation_properties::objectManipulationQueryRequest::DEPART_Z_TOOL;
    ROS_INFO("chosen depart strategy is code %d",depart_option);
    
    // Use depart option to find depart pose. 
    manip_properties_srv_.request.grasp_option = depart_option;    
    
    manip_properties_srv_.request.query_code = object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS;
     manip_properties_client_.call(manip_properties_srv_);
    int n_depart_pose_options = manip_properties_srv_.response.gripper_pose_options.size();
    if (n_depart_pose_options<1) {
                    ROS_WARN("no depart pose options returned for gripper_ID %d and object_ID %d",gripper_id_,object_id);
                    ROS_WARN("should not happen--apparent bug in manipulation properties service");
                    return false;
                }   

    // Note: using the 0'th, i.e. default option. this is expressed somewhat strangely.  
    // Often, this pose will be identical to approach_object_pose_wrt_gripper_
    // expresses original (ungrasped) pose of object from viewpoint of gripper when gripper is at the depart pose,
    // though this depart pose presumably would be holding the object of interest                
    depart_object_pose_wrt_gripper_= manip_properties_srv_.response.gripper_pose_options[0]; 
    //ROS_INFO("default depart pose, expressed as original pose of object w/rt gripper at depart: ");
    //xformUtils.printPose(depart_object_pose_wrt_gripper_);   
    
    // Gripper pose wrt System Frame Transformations:
    // use these relative values to compute gripper poses w/rt system ref frame--using the object's pose w/rt its
    //      named frame_id
    //      object_pose_stamped
    // Logic: 
    // grasp_object_pose_wrt_gripper_: tells object pose w/rt generic_gripper_frame;
    // We also know: object pose w/rt system_ref_frame (i.e. base)
    // use these to solve for generic_gripper_frame w/rt some named frame (e.g. object's frame_id)
    // given A_obj/gripper and A_obj/sys --> A_gripper/sys
    // stf of object frame w/rt its specified frame_id (from object_pose_stamped)
    ROS_INFO("computing grasp stf: ");
    tf::StampedTransform object_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_pose_stamped, "object_frame");
    geometry_msgs::PoseStamped object_wrt_gripper_ps;
    object_wrt_gripper_ps.pose = grasp_object_pose_wrt_gripper_;
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    tf::StampedTransform object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    //ROS_INFO("object w/rt gripper stf: ");
    //xformUtils.printStampedTf(object_wrt_gripper_stf);
    tf::StampedTransform gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    //ROS_INFO("gripper w/rt object stf: ");
    //xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    tf::StampedTransform gripper_stf;
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    grasp_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);
    //ROS_INFO("computed gripper pose at grasp location: ");
    //xformUtils.printStampedPose(grasp_pose_);
    
    //do same for approach and depart poses:
    //approach_object_pose_wrt_gripper_
    ROS_INFO("computing approach stf: ");
    object_wrt_gripper_ps.pose = approach_object_pose_wrt_gripper_; //transform approach pose
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    //ROS_INFO("object w/rt gripper stf: ");
    //xformUtils.printStampedTf(object_wrt_gripper_stf);
    gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    //ROS_INFO("gripper w/rt object stf: ");
    //xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    approach_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);    
    // ROS_INFO("computed gripper pose at approach location: ");
    //xformUtils.printStampedPose(approach_pose_);   
    
    //finally, repeat for depart pose:
    //ROS_INFO("computing depart stf: ");
    object_wrt_gripper_ps.pose = depart_object_pose_wrt_gripper_; //transform depart pose
    object_wrt_gripper_ps.header.frame_id = "generic_gripper_frame";
    object_wrt_gripper_stf = xformUtils.convert_poseStamped_to_stampedTransform(object_wrt_gripper_ps, "object_frame"); 
    //ROS_INFO("object w/rt gripper stf: ");
    //xformUtils.printStampedTf(object_wrt_gripper_stf);
    gripper_wrt_object_stf = xformUtils.stamped_transform_inverse(object_wrt_gripper_stf); //object_wrt_gripper_stf.inverse();
    //ROS_INFO("gripper w/rt object stf: ");
    //xformUtils.printStampedTf(gripper_wrt_object_stf);
    //now compute gripper pose w/rt whatever frame object was expressed in:
    if (!xformUtils.multiply_stamped_tfs(object_stf,gripper_wrt_object_stf,gripper_stf)) {
          ROS_WARN("illegal stamped-transform multiply");
          return false;
    }
    //extract stamped pose from stf; this is the desired generic_gripper_frame w/rt a named frame_id
    // that corresponds to desired grasp transform for given object at a given pose w/rt frame_id
    depart_pose_ = xformUtils.get_pose_from_stamped_tf(gripper_stf);      
    // ROS_INFO("computed gripper pose at depart location: ");
    //xformUtils.printStampedPose(depart_pose_);      
    
    return true;

}
```

#### 2. modify rethink_gripper_rt_manip_fncs.cpp file, add rethink_grasp_COKE_CAN_ID function

```c++
void rethink_grasp_COKE_CAN_ID(int query_code, int grasp_option,
        object_manipulation_properties::objectManipulationQueryResponse&response) {

    std::vector<geometry_msgs::Pose> object_grasp_poses_wrt_gripper;
    std::vector<geometry_msgs::Pose> object_approach_poses_wrt_gripper;
    std::vector<geometry_msgs::Pose> object_depart_poses_wrt_gripper;
    geometry_msgs::Pose object_pose_wrt_gripper;
    Eigen::Matrix3d R_object_wrt_gripper;
    Eigen::Vector3d object_origin_wrt_gripper_frame;
    Eigen::Vector3d x_axis, y_axis, z_axis;
    XformUtils xformUtils;


    ROS_INFO("query, baxter gripper, cork_can; query code %d, grasp option %d", query_code, grasp_option);
    //here are two grasp poses using baxter gripper to grasp can_coke:
    object_pose_wrt_gripper.position.x = 0.0; //easy--align gripper origin w/ object origin
    object_pose_wrt_gripper.position.y = 0.0;
    object_pose_wrt_gripper.position.z = 0.0;
    // object_pose_wrt_gripper.orientation.x = 1.0; //x-axis parallel, z-axis anti-parallel:
    // object_pose_wrt_gripper.orientation.y = 0.0; 
    // object_pose_wrt_gripper.orientation.z = 0.0;
    // object_pose_wrt_gripper.orientation.w = 1.0;
    // Orientation

    object_pose_wrt_gripper.orientation.x = -1.0;
    object_pose_wrt_gripper.orientation.y = 1.0;
    object_pose_wrt_gripper.orientation.z = -1.0;
    object_pose_wrt_gripper.orientation.w = 1.0;
    object_grasp_poses_wrt_gripper.clear();
    object_grasp_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
    // object_pose_wrt_gripper.orientation.x = 0.0; //x-axis anti-parallel, z-axis anti-parallel:
    // object_pose_wrt_gripper.orientation.y = 1.0;//grasp from side 
    object_pose_wrt_gripper.orientation.x = -1.0;
    object_pose_wrt_gripper.orientation.y = 1.0;
    object_pose_wrt_gripper.orientation.z = -1.0;
    object_pose_wrt_gripper.orientation.w = 1.0;
    object_grasp_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
    Eigen::Affine3d affine_object_wrt_gripper, affine_object_wrt_gripper_approach, affine_object_wrt_gripper_depart;
    //affine_object_wrt_gripper = xformUtils.transformPoseToEigenAffine3d(geometry_msgs::Pose pose);   

    switch (query_code) {
        case object_manipulation_properties::objectManipulationQueryRequest::GRASP_STRATEGY_OPTIONS_QUERY:
            //respond to query for grasp options
            response.grasp_strategy_options.clear();
            //in this case, the only option specified is grasp-from-above--which really means
            // from object z-direction...still works if object pose is tilted
            // only alternative, at present, is GRASP_FROM_SIDE
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::GRASP_FROM_ABOVE);
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::GRASP_FROM_SIDE);
            //this version does not populate tolerances of grasp;
            response.valid_reply = true;
            break;
        case object_manipulation_properties::objectManipulationQueryRequest::APPROACH_STRATEGY_OPTIONS_QUERY:
            response.grasp_strategy_options.clear();
            //only options, at present, are APPROACH_Z_TOOL and APPROACH_LATERAL_SLIDE
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::APPROACH_Z_TOOL);
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::APPROACH_LATERAL_SLIDE);        
            response.valid_reply = true;
            break;
        case object_manipulation_properties::objectManipulationQueryRequest::DEPART_STRATEGY_OPTIONS_QUERY:
            response.grasp_strategy_options.clear();
            //only options, at present, are DEPART_Z_TOOL and DEPART_LATERAL_SLIDE
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::DEPART_Z_TOOL);
            response.grasp_strategy_options.push_back(object_manipulation_properties::objectManipulationQueryResponse::DEPART_LATERAL_SLIDE);
            response.valid_reply = true;
            break;
            //inquiry for grasp transform w/ specified grasp option:
        case object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS:
            if (grasp_option == object_manipulation_properties::objectManipulationQueryRequest::GRASP_FROM_SIDE) {
                //fill in grasp pose: pose of object frame w/rt gripper frame
                //geometry_msgs/Pose[] gripper_pose_options
                response.gripper_pose_options.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    response.gripper_pose_options.push_back(object_pose_wrt_gripper);
                }
                //return the two grasp-pose options
                response.valid_reply = true;
                break;
            }else { //this is where to consider alternative grasp strategies for TOY_BLOCK using RETHINK gripper
                ROS_WARN("this grasp option not specified for RETHINK gripper and object COKE_CAN_ID");
                response.valid_reply = false;
                break;
            }

        case object_manipulation_properties::objectManipulationQueryRequest::GET_APPROACH_POSE_TRANSFORMS:
            ROS_INFO("get approach pose transforms...");
            if (grasp_option == object_manipulation_properties::objectManipulationQueryRequest::APPROACH_LATERAL_SLIDE) {
                //compute the approach transform, i.e. pose of object w/rt gripper in approach pose from above
                ROS_INFO("approach grasp along tool-z direction: ");
                object_approach_poses_wrt_gripper.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    object_pose_wrt_gripper = object_grasp_poses_wrt_gripper[i];
                    object_pose_wrt_gripper.position.x += 0.1; // add approach dist in gripper z direction    
                    object_approach_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
                    response.gripper_pose_options.clear();

                }
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    response.gripper_pose_options.push_back(object_pose_wrt_gripper);
                }
                response.valid_reply = true;
                break;
            }else { //this is where to consider alternative grasp strategies for TOY_BLOCK using RETHINK gripper
                ROS_WARN("this grasp option not specified for RETHINK gripper and object COKE_CAN_ID");
                response.valid_reply = false;
                break;
            }
        case object_manipulation_properties::objectManipulationQueryRequest::GET_DEPART_POSE_TRANSFORMS:
            if (grasp_option == object_manipulation_properties::objectManipulationQueryRequest::DEPART_Z_TOOL) {
                //compute the depart transform, i.e. pose of object w/rt gripper to depart above
                //in this case, identical to approach poses
                ROS_INFO("depart strategy along -tool-z direction");
                object_approach_poses_wrt_gripper.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    object_pose_wrt_gripper = object_grasp_poses_wrt_gripper[i];
                    object_pose_wrt_gripper.position.z += 0.1; // add approach dist in gripper z direction    
                    object_approach_poses_wrt_gripper.push_back(object_pose_wrt_gripper);
                }
                response.gripper_pose_options.clear();
                for (int i = 0; i < object_grasp_poses_wrt_gripper.size(); i++) {
                    response.gripper_pose_options.push_back(object_pose_wrt_gripper);
                }
                response.valid_reply = true;
                break;
            } else { //this is where to consider alternative grasp strategies for TOY_BLOCK using RETHINK gripper
                ROS_WARN("this grasp option not specified for RETHINK gripper and object COKE_CAN_ID");
                response.valid_reply = false;
                break;
            }

        default:
            ROS_WARN("grasp poses for rethink gripper unknown for object code COKE_CAN_ID");
            response.valid_reply = false;
            break;
    }

}
```

#### 3. change default(object_grabber.cpp) grab_object function

```c++
if(!get_grab_poses(object_id,object_pose_stamped))
```



#### c. Modify Pose

Modify **fetch_and_stack_client.cpp**

```c++
//send command to set grasped block on top of existing block:
    ROS_INFO("sending a goal: stack coke");
    
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::DROPOFF_OBJECT;
    g_object_pose.header.frame_id = "system_ref_frame"
    g_object_pose.pose.position.x = 0.5;
    g_object_pose.pose.position.y = -0.34;
    g_object_pose.pose.position.z = 1.0;

    goal.dropoff_frame = g_object_pose; //frame per PCL perception
    goal.dropoff_frame.pose.position.z+=0.035; //set height to one block thickness higher
                                               // so new block will stack on prior block
    goal.object_code= ObjectIdCodes::COKE_CAN_UPRIGHT;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to drop off coke; quitting");
        return 0;
    }        
    return 0;
```













