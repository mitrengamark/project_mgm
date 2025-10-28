
#include <gazebo_ackermann_drive_plugin/ackermann_kinematic.h>
#include <tf/transform_datatypes.h>

AckermannKinematic::AckermannKinematic(double wheel_base) {
    wheel_base_ = wheel_base;
    vehicle_odom_ = nav_msgs::Odometry::Ptr(new nav_msgs::Odometry());
}

AckermannKinematic::~AckermannKinematic() {
}

nav_msgs::Odometry AckermannKinematic::Update(double steering_angle, double speed, double dt) {
    update_pose(steering_angle, speed, dt);
    update_twist(steering_angle, speed);
    return *vehicle_odom_;
}

void AckermannKinematic::init_odom(nav_msgs::Odometry init_odom) {
    vehicle_odom_ = nav_msgs::Odometry::Ptr(new nav_msgs::Odometry(init_odom));
}

void AckermannKinematic::update_pose(double steering_angle, double speed, double dt) {
    // update vehicle model
    double delta_x = speed * dt * cos(yaw_);
    double delta_y = speed * dt * sin(yaw_);
    double delta_theta = speed * dt * tan(steering_angle) / wheel_base_;
    yaw_ += delta_theta;

    // update pose
    vehicle_odom_->pose.pose.position.x += delta_x;
    vehicle_odom_->pose.pose.position.y += delta_y;

    // calculate quaternion from yaw
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_); 
    // convert quaternion to message
    tf::quaternionTFToMsg(q, vehicle_odom_->pose.pose.orientation);
}

void AckermannKinematic::update_twist(double steering_angle, double speed) {
    // update twist
    vehicle_odom_->twist.twist.linear.x = speed;
    vehicle_odom_->twist.twist.angular.z = speed * tan(steering_angle) / wheel_base_;
}