#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf2_broadcaster");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped tcp_l, tcp_r, wypt_l_manuf_start, wypt_l_seg1_start, wypt_m_coupler_coupling,
                                    wypt_m_coupler_mag, wypt_m_fixture, wypt_m_manuf_end, wypt_m_seg1_end,
                                    wypt_m_seg2_end, wypt_m_seg2_start, 
                                    lh0, lh1, lh2, lh3, lh4, lh5, lh6,
                                    rh0, rh1, rh2, rh3, rh4, rh5, rh6;


    lh0.header.frame_id = "world";
    lh0.child_frame_id = "lh0";
    lh0.transform.translation.x = 0.48;
    lh0.transform.translation.y = 0;
    lh0.transform.translation.z = 0.5;
    tf2::Quaternion qlh0;
    qlh0.setRPY(-3.14159, 0, 6.28);
    lh0.transform.rotation.x = qlh0.x();
    lh0.transform.rotation.y = qlh0.y();
    lh0.transform.rotation.z = qlh0.z();
    lh0.transform.rotation.w = qlh0.w();

    rh0.header.frame_id = "world";
    rh0.child_frame_id = "rh0";
    rh0.transform.translation.x = 0.56;
    rh0.transform.translation.y = 0;
    rh0.transform.translation.z = 0.5;
    tf2::Quaternion qrh0;
        qrh0.setRPY(-3.14159, 0, 3.14159);
    rh0.transform.rotation.x = qrh0.x();
    rh0.transform.rotation.y = qrh0.y();
    rh0.transform.rotation.z = qrh0.z();
    rh0.transform.rotation.w = qrh0.w();

    lh1.header.frame_id = "world";
    lh1.child_frame_id = "lh1";
    lh1.transform.translation.x = 0.4;
    lh1.transform.translation.y = 0;
    lh1.transform.translation.z = 0.65;
    tf2::Quaternion qlh1;
    qlh1.setRPY(-3.14159, 0, 6.28);
    lh1.transform.rotation.x = qlh1.x();
    lh1.transform.rotation.y = qlh1.y();
    lh1.transform.rotation.z = qlh1.z();
    lh1.transform.rotation.w = qlh1.w();

    rh1.header.frame_id = "world";
    rh1.child_frame_id = "rh1";
    rh1.transform.translation.x = 0.63;
    rh1.transform.translation.y = 0;
    rh1.transform.translation.z = 0.65;
    tf2::Quaternion qrh1;
        qrh1.setRPY(-3.14159, 0, 3.14159);
    rh1.transform.rotation.x = qrh1.x();
    rh1.transform.rotation.y = qrh1.y();
    rh1.transform.rotation.z = qrh1.z();
    rh1.transform.rotation.w = qrh1.w();
    

    lh2.header.frame_id = "world";
    lh2.child_frame_id = "lh2";
    lh2.transform.translation.x = 0.24;
    lh2.transform.translation.y = 0;
    lh2.transform.translation.z = 0.8;
    tf2::Quaternion qlh2;
    qlh2.setRPY(-3.14159, 0, 6.28);
    lh2.transform.rotation.x = qlh2.x();
    lh2.transform.rotation.y = qlh2.y();
    lh2.transform.rotation.z = qlh2.z();
    lh2.transform.rotation.w = qlh2.w();

    rh2.header.frame_id = "world";
    rh2.child_frame_id = "rh2";
    rh2.transform.translation.x = 0.83;
    rh2.transform.translation.y = 0;
    rh2.transform.translation.z = 0.8;
    tf2::Quaternion qrh2;
        qrh2.setRPY(-3.14159, 0, 3.14159);
    rh2.transform.rotation.x = qrh2.x();
    rh2.transform.rotation.y = qrh2.y();
    rh2.transform.rotation.z = qrh2.z();
    rh2.transform.rotation.w = qrh2.w();


    lh3.header.frame_id = "world";
    lh3.child_frame_id = "lh3";
    lh3.transform.translation.x = 0.12;
    lh3.transform.translation.y = 0;
    lh3.transform.translation.z = 0.6;
    tf2::Quaternion qlh3;
    qlh3.setRPY(-3.14159, 0, 6.28);
    lh3.transform.rotation.x = qlh3.x();
    lh3.transform.rotation.y = qlh3.y();
    lh3.transform.rotation.z = qlh3.z();
    lh3.transform.rotation.w = qlh3.w();

    rh3.header.frame_id = "world";
    rh3.child_frame_id = "rh3";
    rh3.transform.translation.x = 0.9;
    rh3.transform.translation.y = 0;
    rh3.transform.translation.z = 0.6;
    tf2::Quaternion qrh3;
        qrh3.setRPY(-3.14159, 0, 3.14159);
    rh3.transform.rotation.x = qrh3.x();
    rh3.transform.rotation.y = qrh3.y();
    rh3.transform.rotation.z = qrh3.z();
    rh3.transform.rotation.w = qrh3.w();


    lh4.header.frame_id = "world";
    lh4.child_frame_id = "lh4";
    lh4.transform.translation.x = 0.22;
    lh4.transform.translation.y = 0;
    lh4.transform.translation.z = 0.42;
    tf2::Quaternion qlh4;
    qlh4.setRPY(-3.14159, 0, 6.28);
    lh4.transform.rotation.x = qlh4.x();
    lh4.transform.rotation.y = qlh4.y();
    lh4.transform.rotation.z = qlh4.z();
    lh4.transform.rotation.w = qlh4.w();

    rh4.header.frame_id = "world";
    rh4.child_frame_id = "rh4";
    rh4.transform.translation.x = 0.8;
    rh4.transform.translation.y = 0;
    rh4.transform.translation.z = 0.42;
    tf2::Quaternion qrh4;
        qrh4.setRPY(-3.14159, 0, 3.14159);
    rh4.transform.rotation.x = qrh4.x();
    rh4.transform.rotation.y = qrh4.y();
    rh4.transform.rotation.z = qrh4.z();
    rh4.transform.rotation.w = qrh4.w();


    lh5.header.frame_id = "world";
    lh5.child_frame_id = "lh5";
    lh5.transform.translation.x = 0.40;
    lh5.transform.translation.y = 0;
    lh5.transform.translation.z = 0.35;
    tf2::Quaternion qlh5;
    qlh5.setRPY(-3.14159, 0, 6.28);
    lh5.transform.rotation.x = qlh5.x();
    lh5.transform.rotation.y = qlh5.y();
    lh5.transform.rotation.z = qlh5.z();
    lh5.transform.rotation.w = qlh5.w();

    rh5.header.frame_id = "world";
    rh5.child_frame_id = "rh5";
    rh5.transform.translation.x = 0.69;
    rh5.transform.translation.y = 0;
    rh5.transform.translation.z = 0.35;
    tf2::Quaternion qrh5;
        qrh5.setRPY(-3.14159, 0, 3.14159);
    rh5.transform.rotation.x = qrh5.x();
    rh5.transform.rotation.y = qrh5.y();
    rh5.transform.rotation.z = qrh5.z();
    rh5.transform.rotation.w = qrh5.w();

    lh6.header.frame_id = "world";
    lh6.child_frame_id = "lh6";
    lh6.transform.translation.x = 0.48;
    lh6.transform.translation.y = 0;
    lh6.transform.translation.z = 0.32;
    tf2::Quaternion qlh6;
    qlh6.setRPY(-3.14159, 0, 6.28);
    lh6.transform.rotation.x = qlh6.x();
    lh6.transform.rotation.y = qlh6.y();
    lh6.transform.rotation.z = qlh6.z();
    lh6.transform.rotation.w = qlh6.w();

    rh6.header.frame_id = "world";
    rh6.child_frame_id = "rh6";
    rh6.transform.translation.x = 0.56;
    rh6.transform.translation.y = 0;
    rh6.transform.translation.z = 0.32;
    tf2::Quaternion qrh6;
        qrh6.setRPY(-3.14159, 0, 3.14159);
    rh6.transform.rotation.x = qrh6.x();
    rh6.transform.rotation.y = qrh6.y();
    rh6.transform.rotation.z = qrh6.z();
    rh6.transform.rotation.w = qrh6.w();


    ros::Rate rate(10.0);

 
    while (node.ok()){
        lh0.header.stamp = ros::Time::now();
        rh0.header.stamp = ros::Time::now();
        tfb.sendTransform(lh0);
        tfb.sendTransform(rh0);

        lh1.header.stamp = ros::Time::now();
        rh1.header.stamp = ros::Time::now();
        tfb.sendTransform(lh1);
        tfb.sendTransform(rh1);

        lh1.header.stamp = ros::Time::now();
        rh1.header.stamp = ros::Time::now();
        tfb.sendTransform(lh1);
        tfb.sendTransform(rh1);

        lh2.header.stamp = ros::Time::now();
        rh2.header.stamp = ros::Time::now();
        tfb.sendTransform(lh2);
        tfb.sendTransform(rh2);

        lh3.header.stamp = ros::Time::now();
        rh3.header.stamp = ros::Time::now();
        tfb.sendTransform(lh3);
        tfb.sendTransform(rh3);

        lh4.header.stamp = ros::Time::now();
        rh4.header.stamp = ros::Time::now();
        tfb.sendTransform(lh4);
        tfb.sendTransform(rh4);

        lh5.header.stamp = ros::Time::now();
        rh5.header.stamp = ros::Time::now();
        tfb.sendTransform(lh5);
        tfb.sendTransform(rh5);

        lh6.header.stamp = ros::Time::now();
        rh6.header.stamp = ros::Time::now();
        tfb.sendTransform(lh6);
        tfb.sendTransform(rh6);
        
        rate.sleep();
        printf("sending\n");
    } 
};
