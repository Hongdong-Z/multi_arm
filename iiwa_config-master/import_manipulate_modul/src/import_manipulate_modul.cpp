#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    ROS_INFO("start");
    ros::init(argc, argv, "import_manipulate_k_modul");
    ROS_INFO("start");
    ros::AsyncSpinner spinner(1);
    
    spinner.start();

    ros::NodeHandle node_handle;

    // Visualization
    moveit_visual_tools::MoveItVisualTools visual_tools("scenes"); // basic frame
    visual_tools.deleteAllMarkers();

    // Advertise the required topic
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
        ROS_INFO("here");
    }
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    ROS_INFO("out");
    // Define the attached object message
    // we will use this message to add or subtract the object from the world and to attach the object to the robot
    moveit_msgs::AttachedCollisionObject attached_object;

    std::string robot_name;
    node_handle.getParam("imkm/robot_name", robot_name);


    if (robot_name == "iiwa")
    {
        /* code */
        std::string efeName;
        node_handle.getParam("imkm/iiwaEndEffector", efeName);
        attached_object.link_name = efeName; // tooling attached on left arm
        //The frame which obj based on
        attached_object.object.header.frame_id = "world";
        // The id of the object
        attached_object.object.id = "k_modul";

        // A default pose
        geometry_msgs::Pose pose;
        tf2::Quaternion q;
        q.setRPY(0, 0, 1.57);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = 0.25;
        pose.position.y = 0.8;
        pose.position.z = 0.15;
        // Added named_frame
        
        std::cout<< pose << std::endl;
        //Define the attached object
        Eigen::Vector3d f_scale;
        f_scale << 0.04, 0.04, 0.04;
        shapes::Mesh* mesh = shapes::createMeshFromResource("package://dual_iiwa_description/meshes/Koppelmodul.stl",
                                                            f_scale );
        shapes::ShapeMsg shape_msg;

        shapes::constructMsgFromShape(mesh, shape_msg);

        // frame need to be add
        
        attached_object.object.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
        attached_object.object.mesh_poses.push_back(pose);

        //Note that attaching an object to the robot requires 
        //the corresponing operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;
        // Since we are attaching the object to the robot hand to simulate picking up the object,
        //we want the collision checker to ignore collisions between the object and the robot hand
        attached_object.touch_links = std::vector<std::string>{"tooling"};

        //Add an object into the enviroment
        ROS_INFO("Adding the object into the world at the given position.");
        //Define a planningScene message
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        //publish this message
        planning_scene_diff_publisher.publish(planning_scene);
    }
    else if (robot_name == "panda")
    {
        std::string efeName;
        node_handle.getParam("imkm/pandaEndEffector", efeName);
        attached_object.link_name = efeName; // tooling attached on left arm
        //The frame which obj based on
        attached_object.object.header.frame_id = "world";
        // The id of the object
        attached_object.object.id = "k_modul";

        // A default pose
        geometry_msgs::Pose pose;
        tf2::Quaternion q;
        q.setRPY(0, 0, 3.14);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = 0.52;
        pose.position.y = 0;
        pose.position.z = 0.15;
        // Added named_frame
        
        std::cout<< pose << std::endl;
        //Define the attached object
        Eigen::Vector3d f_scale;
        f_scale << 0.012, 0.012, 0.012;
        shapes::Mesh* mesh = shapes::createMeshFromResource("package://dual_panda_description/meshes/KoppelModul.stl",
                                                            f_scale );
        shapes::ShapeMsg shape_msg;

        shapes::constructMsgFromShape(mesh, shape_msg);

        // frame need to be add
        
        attached_object.object.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
        attached_object.object.mesh_poses.push_back(pose);

        //Note that attaching an object to the robot requires 
        //the corresponing operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;
        // Since we are attaching the object to the robot hand to simulate picking up the object,
        //we want the collision checker to ignore collisions between the object and the robot hand
        attached_object.touch_links = std::vector<std::string>{efeName};

        //Add an object into the enviroment
        ROS_INFO("Adding the object into the world at the given position.");
        //Define a planningScene message
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        //publish this message
        planning_scene_diff_publisher.publish(planning_scene);
    }
    else if (robot_name == "comau")
    {
        /* code */
         std::string efeName;
        node_handle.getParam("imkm/comauEndEffector", efeName);
        attached_object.link_name = efeName; // tooling attached on left arm
        //The frame which obj based on
        attached_object.object.header.frame_id = "world";
        // The id of the object
        attached_object.object.id = "k_modul";

        // A default pose
        geometry_msgs::Pose pose;
        tf2::Quaternion q;
        q.setRPY(1.57, 0, 3.14);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = 2.5;
        pose.position.y = 0;
        pose.position.z = 0.37;
        // Added named_frame
        
        std::cout<< pose << std::endl;
        //Define the attached object
        Eigen::Vector3d f_scale;
        f_scale << 0.01, 0.01, 0.01;
        //f_scale << 1, 1, 1;
        shapes::Mesh* mesh = shapes::createMeshFromResource("package://comau_nj290_description/meshes/koppel_model.stl",
                                                            f_scale );
        shapes::ShapeMsg shape_msg;

        shapes::constructMsgFromShape(mesh, shape_msg);

        // frame need to be add
        
        attached_object.object.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
        attached_object.object.mesh_poses.push_back(pose);

        //Note that attaching an object to the robot requires 
        //the corresponing operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;
        // Since we are attaching the object to the robot hand to simulate picking up the object,
        //we want the collision checker to ignore collisions between the object and the robot hand
        attached_object.touch_links = std::vector<std::string>{efeName};

        //Add an object into the enviroment
        ROS_INFO("Adding the object into the world at the given position.");
        //Define a planningScene message
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        //publish this message
        planning_scene_diff_publisher.publish(planning_scene);


        
        ////addd bohrer
        attached_object.object.id = "bohrer";

        // A default pose
        geometry_msgs::Pose pose1;
        tf2::Quaternion q1;
        q1.setRPY(0, 1.57, 4.71);
        pose1.orientation.x = q1.x();
        pose1.orientation.y = q1.y();
        pose1.orientation.z = q1.z();
        pose1.orientation.w = q1.w();
        pose1.position.x = 1.9;
        pose1.position.y = 1;
        pose1.position.z = 0.35;
        // Added named_frame
        
        std::cout<< pose1 << std::endl;
        //Define the attached object
        Eigen::Vector3d f_scale1;
        f_scale1 << 0.01, 0.01, 0.01;
        //f_scale << 1, 1, 1;
        shapes::Mesh* mesh1 = shapes::createMeshFromResource("package://comau_nj290_description/meshes/bohrer.stl",
                                                            f_scale1 );
        shapes::ShapeMsg shape_msg1;

        shapes::constructMsgFromShape(mesh1, shape_msg1);

        // frame need to be add
        
        attached_object.object.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg1));
        attached_object.object.mesh_poses.push_back(pose1);

        //Note that attaching an object to the robot requires 
        //the corresponing operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;
        // Since we are attaching the object to the robot hand to simulate picking up the object,
        //we want the collision checker to ignore collisions between the object and the robot hand
        attached_object.touch_links = std::vector<std::string>{efeName};

        //Add an object into the enviroment
        ROS_INFO("Adding the object into the world at the given position.");
        //Define a planningScene message
        moveit_msgs::PlanningScene planning_scene1;
        planning_scene1.world.collision_objects.push_back(attached_object.object);
        planning_scene1.is_diff = true;
        //publish this message
        planning_scene_diff_publisher.publish(planning_scene1);


        //// add drucker
        attached_object.object.id = "drucker";

        // A default pose
        geometry_msgs::Pose pose2;
        tf2::Quaternion q2;
        q2.setRPY(1.57, 0, 3.14);
        pose2.orientation.x = q2.x();
        pose2.orientation.y = q2.y();
        pose2.orientation.z = q2.z();
        pose2.orientation.w = q2.w();
        pose2.position.x = 3.1;
        pose2.position.y = 1;
        pose2.position.z = 0.35;
        // Added named_frame
        
        std::cout<< pose2 << std::endl;
        //Define the attached object
        Eigen::Vector3d f_scale2;
        f_scale2 << 0.01, 0.01, 0.01;
        //f_scale << 1, 1, 1;
        shapes::Mesh* mesh2 = shapes::createMeshFromResource("package://comau_nj290_description/meshes/drucker.stl",
                                                            f_scale2 );
        shapes::ShapeMsg shape_msg2;

        shapes::constructMsgFromShape(mesh2, shape_msg2);

        // frame need to be add
        
        attached_object.object.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg2));
        attached_object.object.mesh_poses.push_back(pose2);

        //Note that attaching an object to the robot requires 
        //the corresponing operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;
        // Since we are attaching the object to the robot hand to simulate picking up the object,
        //we want the collision checker to ignore collisions between the object and the robot hand
        attached_object.touch_links = std::vector<std::string>{efeName};

        //Add an object into the enviroment
        ROS_INFO("Adding the object into the world at the given position.");
        //Define a planningScene message
        moveit_msgs::PlanningScene planning_scene2;
        planning_scene2.world.collision_objects.push_back(attached_object.object);
        planning_scene2.is_diff = true;
        //publish this message
        planning_scene_diff_publisher.publish(planning_scene2);
      
    }
    else
    {

        std::cerr << "Please give the name of to manipulate robot: iiwa panda comau" << std::endl;
    }
    
    
    

    ros::shutdown();
    return 0;
}
