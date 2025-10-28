/*
 *    Filename: AckermannDrivePlugin.h
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 *
 */


#ifndef ACKERMANN_KINEMATIC_H_
#define ACKERMANN_KINEMATIC_H_

#include "nav_msgs/Odometry.h"

class AckermannKinematic {

public:

    AckermannKinematic(double wheel_base );

    ~AckermannKinematic();

    void init_odom(nav_msgs::Odometry init_odom);

    nav_msgs::Odometry Update(double steering_angle, double speed, double dt);

private:
    nav_msgs::Odometry::Ptr vehicle_odom_;

    void update_pose(double steering_angle, double speed, double dt);

    void update_twist(double steering_angle, double speed);

    double wheel_base_;

    double yaw_;

};

typedef std::shared_ptr<AckermannKinematic> AckermannKinematicPtr;


#endif /* ACKERMANN_KINEMATIC_H_ */