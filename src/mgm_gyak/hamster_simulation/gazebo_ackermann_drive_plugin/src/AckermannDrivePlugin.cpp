	/*
 *    Filename: AckermannDrivePlugin.cpp
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 */

#include <gazebo_ackermann_drive_plugin/AckermannDrivePlugin.h>
#include <string>
#include <iostream>


namespace gazebo{


AckermannDrivePlugin::AckermannDrivePlugin() : 
    
    steeringfrontLeftPid_(10, 0.0, 0.01),
    steeringfrontRightPid_(10, 0.0, 0.01),
    linearZeroCounter_(0)
    {
    steeringfrontLeftPid_.SetCmdMax(10);
    steeringfrontRightPid_.SetCmdMax(10);

    steeringfrontLeftPid_.SetCmdMin(-10);
    steeringfrontRightPid_.SetCmdMin(-10);
    
    currentCommand_.drive.speed = 0.0;
    currentCommand_.drive.steering_angle = 0.0;

}

AckermannDrivePlugin::~AckermannDrivePlugin() {

}

void AckermannDrivePlugin::commandCallback(
    const ackermann_msgs::AckermannDriveStamped::Ptr& cmd) {

    currentCommand_ = *cmd;

    double speed = fabs(currentCommand_.drive.speed);

    //
    // Speed constraints
    //

    if (speed < minSpeed_ && speed > 0) {
        currentCommand_.drive.speed = minSpeed_;
    }

    currentCommand_.drive.speed = copysign(min(speed, maxSpeed_), cmd->drive.speed);
}

void AckermannDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

	
    initParams(sdf);

    ros::NodeHandle node(robotNamespace_);

    cmdSubscriber_ = node.subscribe(
        driveTopic_, 2, &AckermannDrivePlugin::commandCallback, this);

    odomPublisher_ = node.advertise<nav_msgs::Odometry>(odomTopic_, 5, false);
    gtPublisher_ = node.advertise<nav_msgs::Odometry>(groundTruthTopic_, 5, false);
  
    this->model_ = model;
    if(!model_) {
        return;
    }
    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
                steeringfrontRightPid_);
                
    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
                steeringfrontLeftPid_);
     
    this->model_->GetJointController()->SetPositionTarget(
        frontRightWheelSteeringJoint_, 0.0);

    this->model_->GetJointController()->SetPositionTarget(
        frontLeftWheelSteeringJoint_, 0.0);

    for (auto&& wheelJoint : wheelJoints_) {
        this->model_->GetJoint(wheelJoint)->SetParam(
            "fmax", 0, (double)torque_);
        this->model_->GetJoint(wheelJoint)->SetParam(
            "vel", 0, 0.0);

    }
    
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

    ROS_INFO("AckermannDrive plugin loaded!");
    
}

void AckermannDrivePlugin::Update(const common::UpdateInfo &info) {

    if(!model_) {
        return;
    }

    if(this->model_->GetLink(baseLink_) == NULL) {
        return;
    }

     if (lastUpdate_ == common::Time()) {
        lastUpdate_ = info.simTime;
        return;
    }

    double timeDeltaSec = (info.simTime - lastUpdate_).Double();
    if (timeDeltaSec > 0.02) {

        //
        // Steering constraints
        //

        steering_command = fmax(-0.296706, fmin(0.296706, currentCommand_.drive.steering_angle));

        auto linearVelocity = (double)currentCommand_.drive.speed / wheelRadius_;

        bool isRotationOnly = (double)fabs(currentCommand_.drive.speed) < (minSpeed_ / 2) &&
            fabs(steering_command) > 0.02;

        bool stop = fabs(currentCommand_.drive.speed) < (minSpeed_ / 2) && 
            fabs(steering_command) < 0.02;

        if (isRotationOnly) {
            linearZeroCounter_ += 0.04;
            linearVelocity = maxSpeed_/wheelRadius_ * 0.1 * sin(linearZeroCounter_);
            linearVelocity *= 0.99;

            if (linearVelocity < 0) {
                steering_command *= -1;
            }
        }

        for (auto&& wheelJoint : wheelJoints_) {

            if(!model_) {
                return;
            }

            if (stop) {
            this->model_->GetJoint(wheelJoint)->SetParam(
                "fmax", 0, (double)torque_ * 100);
            } else {
                this->model_->GetJoint(wheelJoint)->SetParam(
                    "fmax", 0, (double)torque_);
            }

            this->model_->GetJoint(wheelJoint)->SetParam("vel", 0, linearVelocity);
        }

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
                    rightWheelSteering(steering_command));

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
                    leftWheelSteering(steering_command));

        publishOdometry(timeDeltaSec);

        ros::spinOnce();

        lastUpdate_ = info.simTime;
    }
}

inline double AckermannDrivePlugin::rightWheelSteering(double baseAngle) {
	double steering_angle =0;
	auto turning_radius = wheelBase_ / tan(fabs(baseAngle));
	if (baseAngle >0)
	{
		steering_angle = atan(wheelBase_ / (turning_radius + wheelSeparartion_ / 2));
	}
	else
	{
		steering_angle = -atan(wheelBase_ / (turning_radius - wheelSeparartion_ / 2));
	}
	return steering_angle;
	// return atan(2 * wheelSeparartion_ * sin(baseAngle)/ (
			// 2 * wheelSeparartion_ * cos(baseAngle) - 
				// wheelBase_ * sin(baseAngle)));
}

inline double AckermannDrivePlugin::leftWheelSteering(double baseAngle) {
	double steering_angle =0;
	auto turning_radius = wheelBase_ / tan(fabs(baseAngle));
	if (baseAngle >0)
	{
		steering_angle = atan(wheelBase_ / (turning_radius - wheelSeparartion_ / 2));
		
	}
	else
	{
		steering_angle =-atan(wheelBase_ / (turning_radius + wheelSeparartion_ / 2));
	}
	return steering_angle;
	// return atan(2 * wheelSeparartion_ * sin(baseAngle)/ (
			// 2 * wheelSeparartion_ * cos(baseAngle) + 
				// wheelBase_ * sin(baseAngle)));
}

void AckermannDrivePlugin::publishOdometry(double dt) {

    auto position = this->model_->GetLink(baseLink_)->WorldPose().Pos();
    auto rotation = this->model_->GetLink(baseLink_)->WorldPose().Rot();
    auto linear_vel = this->model_->GetLink(baseLink_)->RelativeLinearVel();
    auto angular_vel = this->model_->GetLink(baseLink_)->RelativeAngularVel();

    //
    //Ground truth ros message publishing
    //
    gtMsg.header.frame_id = "map";
    gtMsg.header.stamp = ros::Time::now();
    gtMsg.child_frame_id = baseFrame_;
    gtMsg.pose.pose.position.x = position.X();
    gtMsg.pose.pose.position.y = position.Y();
    gtMsg.pose.pose.position.z = position.Z();
    
    tf::Quaternion q_gt(rotation.X(), rotation.Y(), rotation.Z(), rotation.W());
    tf::quaternionTFToMsg(q_gt, gtMsg.pose.pose.orientation);
    
    gtMsg.twist.twist.linear.x = linear_vel.X();
    gtMsg.twist.twist.linear.y = linear_vel.Y();
    gtMsg.twist.twist.linear.z = linear_vel.Z();

    gtMsg.twist.twist.angular.x = angular_vel.X() ;
    gtMsg.twist.twist.angular.y = angular_vel.Y();
    gtMsg.twist.twist.angular.z = angular_vel.Z();
    gtPublisher_.publish(gtMsg);

    //
    //Odom ros message publishing
    //

    if (currentCommand_.drive.speed !=0) {

        auto noised_velx = linear_vel.X() + odomNoise_.gaussian(mean_, stddev_);
        auto noised_steering_angle = steering_command + odomNoise_.gaussian(mean_, stddev_);

        odomMsg = vehicle_predict->Update(noised_steering_angle, noised_velx, dt);   
    }
    
    odomMsg.header = gtMsg.header;
    odomMsg.child_frame_id = baseFrame_;
    
    odomPublisher_.publish(odomMsg);

    //
    //Tf publishing
    //

    tf::Transform transform;
    if(is_tfBroadcasting_)
    {
    	if (is_tf_gt)
		{
			transform.setOrigin(tf::Vector3(gtMsg.pose.pose.position.x, gtMsg.pose.pose.position.y, gtMsg.pose.pose.position.z));
			transform.setRotation(q_gt);
		}
		else if (is_tf_odom)
		{
			transform.setOrigin(tf::Vector3(odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z));
			tf::Quaternion q_odom;
            tf::quaternionMsgToTF(odomMsg.pose.pose.orientation, q_odom);
            transform.setRotation(q_odom);
		}
		
        tfBroadcaster_.sendTransform(tf::StampedTransform(
            transform, ros::Time::now(), odomFrame_, baseFrame_));
	}
}

void AckermannDrivePlugin::initParams(sdf::ElementPtr sdf) {

    robotNamespace_ = getParam<std::string>("robotNamespace", "", sdf);
    wheelRadius_ = getParam<double>("wheelRadius", 0.029, sdf);

    wheelJoints_.push_back(getParam<std::string>(
        "frontLeftWheelJoint", "front_left_wheel_joint", sdf));
    wheelJoints_.push_back(getParam<std::string>(
        "frontRightWheelJoint", "front_right_wheel_joint", sdf));
    wheelJoints_.push_back(getParam<std::string>(
        "reartLeftWheelJoint", "rear_left_wheel_joint", sdf));
    wheelJoints_.push_back(getParam<std::string>(
        "rearRightWheelJoint", "rear_right_wheel_joint", sdf));

    frontRightWheelSteeringJoint_ = getParam<std::string>(
        "frontRightWheelSteeringJoint", "front_right_wheel_steering_joint", sdf);
    frontLeftWheelSteeringJoint_ = getParam<std::string>(
        "frontLeftWheelSteeringJoint", "front_left_wheel_steering_joint", sdf);

    torque_ = getParam<double>("torque", 0.01, sdf);
    wheelSeparartion_ = getParam<double>("wheelSeparation", 0.17, sdf);
    wheelBase_ = getParam<double>("wheelBase", 0.166, sdf);
    mean_ = getParam<double>("odomNoiseMean", 0.0, sdf);
    stddev_ = getParam<double>("odomNoiseStddev", 0.01, sdf);
    minSpeed_ = getParam<double> ("minSpeed", -1.2, sdf);
    maxSpeed_ = getParam<double> ("maxSpeed", 1.2, sdf);

    baseFrame_ = getParam<std::string>("baseFrame", "base_link", sdf);
    odomFrame_ = getParam<std::string>("odomFrame", "odom", sdf);

    groundTruthTopic_ = getParam<std::string>("groundTruthTopic_", "odom_gt", sdf);
    odomTopic_ = getParam<std::string>("odomTopic", "odom", sdf);
    driveTopic_ = getParam<std::string>("driveTopic", "ackermann_cmd", sdf);

    baseLink_ = getParam<std::string>("robotBaseLink", "base_link", sdf);
    
    is_tfBroadcasting_ = getParam<bool>("is_tfBroadcasting", true, sdf);
    tf_target = getParam<std::string>("tf_target", "odom", sdf);
    if (is_tfBroadcasting_)
    {
		if (tf_target.find(groundTruthTopic_) != std::string::npos)
		{
			is_tf_gt = true;
			is_tf_odom = false;
			ROS_WARN("Tf is published based on ground truth topic");
		}
		else if (tf_target.find(odomTopic_) != std::string::npos)
		{
			is_tf_gt = false;
			is_tf_odom = true;
			ROS_WARN("Tf is published based on odom topic");
		}
		else
		{
			is_tf_gt = true;
			is_tf_odom = false;
			ROS_WARN("Wrong tf_target input. Tf is published based on ground truth topic");
		}
    }

    if (!robotNamespace_.empty()) {
        for (auto&& wheelJoint : wheelJoints_) {
            wheelJoint = robotNamespace_ + "::" + wheelJoint;
        }

        frontRightWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontRightWheelSteeringJoint_; 
        frontLeftWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontLeftWheelSteeringJoint_;

        baseFrame_ = tf::resolve(robotNamespace_, baseFrame_);
        odomFrame_ = tf::resolve(robotNamespace_, odomFrame_);

        baseLink_ = robotNamespace_ + "::" + baseLink_;
    }

    vehicle_predict = std::make_shared<AckermannKinematic>(wheelBase_);
}

} // namespace gazebo

