#include "../include/nmpc_planner.h"

void Planner::storeLatestTargetGTPose(const nav_msgs::Odometry::ConstPtr& msg)
{
 if(!targetEstimationIsActive)   // when it is active, the callback storeLatestTargetEstimatedPose is used.
    targetObjectGTPose = *msg; // GT anyway comes in NED
 else
 {
     //printf("Not using the GT\n");
   if(useGTforTarget)
       targetObjectGTPose = *msg;
 }
}

double Planner::findDistanceDistribution(double rad,int rob)
{
  double refrence_distribution = -3.337 + 0.3645*rad - 0.1944*rob - 0.00701*pow(rad,2) + 1.278*rad*rob;
  return refrence_distribution;
}

void Planner::avoidTeamMates_byComputingExtForce(double sumOfGradients, int direction)
{
    selfPoseTraj.poses.clear();
    geometry_msgs::Pose tmpPose;
    geometry_msgs::Pose2D tmpPose2D;
    float xT =  targetObjectGTPose.pose.pose.position.x;
    float yT = -targetObjectGTPose.pose.pose.position.y;
    int flag_weight = 0;
    float x_obs=0,y_obs=0,z_obs=0;

    for(int t=0;t<16;t++)
    {
        bool atLeastOneMatePresent = false;

        Position3D tCurrentSelfPosition(selfPose.x,selfPose.y,0);

        if(t==0)
        {
            tCurrentSelfPosition(0) = selfPose.x;
            tCurrentSelfPosition(1) = selfPose.y;
            tCurrentSelfPosition(2) = 0;
        }

        else
        {
            tCurrentSelfPosition(0) = StateInput(0,t);
            tCurrentSelfPosition(1) = StateInput(2,t);
            tCurrentSelfPosition(2) = 0;
            tmpPose.position.x = StateInput(0,t);
            tmpPose.position.y = StateInput(2,t);
            tmpPose.position.z = 0;
            tmpPose2D.x = StateInput(0,t);
            tmpPose2D.y = StateInput(2,t);
            selfPoseTraj.poses.push_back(tmpPose);
            selfPoseTraj2OpenBase.push_back(tmpPose2D);
        }

        Position3D virtualPoint = tCurrentSelfPosition;
        Eigen::Vector3d totalForce = Velocity3D::Zero();


        for(int j=0; j<numRobots_-1; j++)
        {
            if(heardFromMates[j])
            {
                Position3D matePosition(matesPoses[j].pose.pose.position.x,matesPoses[j].pose.pose.position.y,0);

                if(t!=0 && heardFromMateTrajs[j])
                {

                    matePosition(0) = matesPoseTrajs[j].poses[t-1].position.x;
                    matePosition(1) = matesPoseTrajs[j].poses[t-1].position.y;
                    matePosition(2) = 0;
                }
                //Position3D matePosition(matesPoseTrajs[j].poses[t].position.x,matesPoseTrajs[j].poses[t].position.y,matesPoseTrajs[j].poses[t].position.z);
                double thetaMate = fmod((atan2(matePosition(1)-yT,matePosition(0)-xT)+2*PI),(2*PI));
                double theta = fmod((atan2(tCurrentSelfPosition(1)-yT,tCurrentSelfPosition(0)-xT)+2*PI),(2*PI));

                Position3D posDiff = virtualPoint - matePosition;
                double neighborDist = posDiff.norm();
                Position3D posDiffUnit = posDiff.normalized();

                double thetaDiff = abs(theta-thetaMate);
                Position3D posDiff2 (tCurrentSelfPosition(0) - xT,tCurrentSelfPosition(1) - yT,0);
                double neighborDist2 = thetaDiff;

                Position3D posDiffMate (matePosition(0) - xT,matePosition(1) - yT, 0);



                Position3D tangent(-posDiff2(1),posDiff2(0),0);
                Position3D tangentUnit = tangent.normalized();

                Position3D tangentNeg(posDiff2(1),-posDiff2(0),0);
                Position3D tangentUnitNeg = tangentNeg.normalized();

                double potentialForce1 = 0.0;
                double potentialForce2 = 0.0;
                potentialForce1 = formationRepulsiveGradientVector[j]->getPotential(neighborDist,neighborDistThreshold);
                // if (neighborDist <= neighborDistThreshold )
                //     attractionWeight = 1000;
                // else            if(neighborDist<neighborDistThreshold)
                atLeastOneMatePresent = true; //if no neighbour is in this range then we do not do mate avoidance.
                //     attractionWeight = 1000;


                // if (j == 2)
                // {
                if(neighborDist<neighborDistThreshold)
                    atLeastOneMatePresent = true; //if no neighbour is in this range then we do not do mate avoidance.

                 //sumOfGradients = posDiff2.norm();
                 //if (sumOfGradients < 1){
                //     potentialForce2 = 0;
                //     sumOfGradients = 0;
                //     }
                sumOfGradients = abs(sumOfGradients);
                //if(sumOfGradients > 5)
                //     sumOfGradients = 5;
                //sumOfGradients = 0;
                totalForce += (potentialForce1) * posDiffUnit * 1 ;

                //ROS_INFO("Potential Force Magnitude = %f",potentialForce1*tFormationRepulGain);
                potentialForce2 = formationRepulsiveGradientVector[j]->getPotential(neighborDist2,0.7);
                // if (neighborDist <= neighborDistThreshold)
                // {
                    if (direction > 0)
                        totalForce +=  0.0*sumOfGradients/attractionWeight*tangentUnit*potentialForce2 ;
                    else
                        totalForce +=  0.0*sumOfGradients/attractionWeight*tangentUnitNeg*potentialForce2 ;
                // }

                total_ang_force += 0.05*sumOfGradients/attractionWeight*potentialForce2;
            }

        }

        if (t==0)
        {
          for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
          {

              //first check if the obstacle point is on the line
              x_obs = obstaclesFromRobots.poses[i].position.x;
              y_obs = obstaclesFromRobots.poses[i].position.y; // NED to NWU
              z_obs = 0; // NED to NWU

              Position3D matePosition(x_obs,y_obs,z_obs);
              double thetaMate = fmod((atan2(matePosition(1)-yT,matePosition(0)-xT)+2*PI),(2*PI));
              double theta = fmod((atan2(tCurrentSelfPosition(1)-yT,tCurrentSelfPosition(0)-xT)+2*PI),(2*PI));

              Position3D posDiff = virtualPoint - matePosition;
              double neighborDist = posDiff.norm();
              Position3D posDiffUnit = posDiff.normalized();

              double thetaDiff = abs(theta-thetaMate);
              Position3D posDiff2 (tCurrentSelfPosition(0) - xT,tCurrentSelfPosition(1) - yT,0);
              double neighborDist2 = thetaDiff;

              Position3D posDiffMate (matePosition(0) - xT,matePosition(1) - yT, 0);

              Position3D tangent(-posDiff2(1),posDiff2(0),0);
              Position3D tangentUnit = tangent.normalized();

              Position3D tangentNeg(posDiff2(1),-posDiff2(0),0);
              Position3D tangentUnitNeg = tangentNeg.normalized();

              double potentialForce1 = 0.0;
              double potentialForce3 = 0.0;
              potentialForce1 = pointCloudRepulsiveGradientVector[i]->getPotential(neighborDist,neighborDistThreshold);

              sumOfGradients = abs(sumOfGradients);
              totalForce += (potentialForce1) * posDiffUnit * 5 ;
              potentialForce3 = pointCloudRepulsiveGradientVector[i]->getPotential(neighborDist2,0.7);

              if (direction > 0)
                  totalForce +=  0.005*sumOfGradients/attractionWeight*tangentUnit*potentialForce3 ;
              else
                  totalForce +=  0.005*sumOfGradients/attractionWeight*tangentUnitNeg*potentialForce3 ;

              // ROS_INFO("Static Obstacle Field x: %f,y: %f,z: %f", totalForce(1),totalForce(2),totalForce(3));

              //
              total_ang_force += 0.005*sumOfGradients/attractionWeight*potentialForce3;

          }
        }

        if(atLeastOneMatePresent)
        {
            obstacle_force(0,t) = totalForce(0);
            obstacle_force(1,t) = totalForce(1);
            obstacle_force(2,t) = 0;//totalForce(2);
        }
        else
        {
            obstacle_force(0,t) = 0;
            obstacle_force(1,t) = 0;
            obstacle_force(2,t) = 0;
        }
    }

}

void Planner::repositionDestinationDueToStaticObstacle(float &x, float &y, float z, float tgtX, float tgtY) // Geometric Hack to avoid obstacles
{
    //here do the free ray check algorithm.
    // remember that the arguments of this function is in NWU but the obstacle received in its update callback  are in NED so it needs to be converted first to NWu.
    //R:meaning of above line?
    float x1 = x, y1 =y, x2 = tgtX, y2=tgtY;

    float z1 = z; float z2 = 0.3; //center of the ground target

    float x_obs=0,y_obs=0,z_obs=0,AB=0,AP=0,PB=0;

    double deltaTheta = PI/20;
    double r = 6;//;
    bool targetFullyBlocked = false;
    bool openingFound = false, leftOpeningFound = false, rightOpeningFound = false;
    bool obstacleWithinRay = false;
    double delta_1 = 0.05;
    double delta_2 = 0.1;

    for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
    {
        //first check if the obstacle point is on the line
        x_obs = obstaclesFromRobots.poses[i].position.x;
        y_obs = obstaclesFromRobots.poses[i].position.y; // NED to NWU
        z_obs = obstaclesFromRobots.poses[i].position.z; // NED to NWU

        AB = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
        AP = sqrt((x_obs-x1)*(x_obs-x1)+(y_obs-y1)*(y_obs-y1)+(z_obs-z1)*(z_obs-z1));
        PB = sqrt((x2-x_obs)*(x2-x_obs)+(y2-y_obs)*(y2-y_obs)+(z2-z_obs)*(z2-z_obs));

        if(fabs(AB - (AP + PB))<delta_1)
        {
            obstacleWithinRay = true;
            break;
        }

    }

    if(obstacleWithinRay==false)
    {
        //ROS_INFO("Clear Visibility to object");
        return;  // freely visible object
        tGoalAttractionGain = 5.0;
    }
    else // do something
    {
        tGoalAttractionGain = 5.0;
        openingFound = false;
        for(int j=1; j<=20; j++)
        {

            float theta = atan2(y1-y2,x1-x2);

            double x1_left = x2 + r*cos(theta - j * deltaTheta);
            double y1_left = y2 + r*sin(theta - j * deltaTheta);

            double x1_right = x2 + r*cos(theta + j * deltaTheta);
            double y1_right = y2 + r*sin(theta + j * deltaTheta);

            leftOpeningFound = true;
            rightOpeningFound = true;

            for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
            {
                //first check if the obstacle point is on the line
                x_obs = obstaclesFromRobots.poses[i].position.x;
                y_obs = obstaclesFromRobots.poses[i].position.y; // NED to NWU
                z_obs = obstaclesFromRobots.poses[i].position.z; // NED to NWU

                AB = sqrt((x2-x1_left)*(x2-x1_left)+(y2-y1_left)*(y2-y1_left)+(z2-z1)*(z2-z1));
                AP = sqrt((x_obs-x1_left)*(x_obs-x1_left)+(y_obs-y1_left)*(y_obs-y1_left)+(z_obs-z1)*(z_obs-z1));
                PB = sqrt((x2-x_obs)*(x2-x_obs)+(y2-y_obs)*(y2-y_obs)+(z2-z_obs)*(z2-z_obs));

                if(fabs(AB - (AP + PB))<delta_2)
                {
                    ROS_INFO("Left Opening closed at delta theta = %f", j*deltaTheta);
                    leftOpeningFound = false;
                    //x = x1_left;
                    //y = y1_left;
                }

                AB = sqrt((x2-x1_right)*(x2-x1_right)+(y2-y1_right)*(y2-y1_right)+(z2-z1)*(z2-z1));
                AP = sqrt((x_obs-x1_right)*(x_obs-x1_right)+(y_obs-y1_right)*(y_obs-y1_right)+(z_obs-z1)*(z_obs-z1));
                PB = sqrt((x2-x_obs)*(x2-x_obs)+(y2-y_obs)*(y2-y_obs)+(z2-z_obs)*(z2-z_obs));

                if(fabs(AB - (AP + PB))<delta_2)
                {
                    ROS_INFO("Right Opening closed at delta theta = %f", j*deltaTheta);
                    //openingFound = true;
                    rightOpeningFound = false;
                    //x = x1_right;
                    //y = y1_right;
                }

                if(!rightOpeningFound && !leftOpeningFound)
                    break;

            }

            if(leftOpeningFound)
            {
                x = x1_left;
                y = y1_left;
                openingFound = true;
            }
            if(rightOpeningFound)
            {
                x = x1_right;
                y = y1_right;
                openingFound = true;
            }

            if(openingFound)
                break;
        }

        return;
    }
}


void Planner::updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int robID)
{
  obstaclesFromRobots = *msg;
  pointCloudRepulsiveGradientVector.resize(obstaclesFromRobots.poses.size());
  for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
  {
    pointCloudRepulsiveGradientVector[i] = new CoTanRepulsiveGradient("pointCloudRepulsiveGradient_Neighbor_" + boost::lexical_cast<std::string>(i)+"_"+boost::lexical_cast<std::string>(robID), neighborDistThreshold, 0.3, 7.0, 5);
  }
}


void Planner::matePoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg, int robID)
{
  //note that robID follows the 1-base, not zero base
  matePose = *msg;
  matesPoses[robID-1].pose.pose.position.x = msg->x;
  matesPoses[robID-1].pose.pose.position.y = msg->y;
  Position3D tCurrentSelfPosition(selfPose.x,selfPose.y,0);
  Position3D tCurrentMatePosition(matePose.x,matePose.y,0);
  // HACK TO SIMULATE COMMUNICATION RADIUS
  if ((tCurrentSelfPosition - tCurrentMatePosition).norm() <= neighborDistThreshold + 2)
  {
    heardFromMate=true;
    heardFromMates[robID-1] = true;
  }
  else
  {
    heardFromMate=false;
    heardFromMates[robID-1] = false;
  }

}



void Planner::matePoseTrajectoryCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int robID)
{
  matesPoseTrajs[robID-1] = *msg;
  heardFromMateTrajs[robID-1] = true;
}



double Planner::quat2eul(nav_msgs::Odometry queryPose_)
{
    tf::Quaternion q(
        queryPose_.pose.pose.orientation.x,
        queryPose_.pose.pose.orientation.y,
        queryPose_.pose.pose.orientation.z,
        queryPose_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double selfRoll, selfPitch, selfYaw;
    m.getRPY(selfRoll, selfPitch, selfYaw);
    return selfYaw;
}

void Planner::selfPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg, int robID) //most important -- self nmpc
{
    // outPoseModifiedToRviz.header = msg->header;
    // outPoseModifiedToRviz.header.frame_id = "world";
    // outPoseToRviz.header = msg->header;
    // outPoseToRviz.header.frame_id = "world";
    selfPose = *msg;

    #ifdef USE_CVXGEN_1ROB

    //for all three solvers some values are fixed or are initialized
    MPCsolver solveMPC;

    ExtForce(0) = 0;
    ExtForce(1) = 0;
    ExtForce(2) = 0 * GRAVITY;
    Eigen::Matrix<double, 6, 1> cur_state = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 2> state_limits = Eigen::Matrix<double, 6, 2>::Zero();
    Eigen::Matrix<double, 3, 2> input_limits = Eigen::Matrix<double, 3, 2>::Zero();
    Eigen::Matrix<double, 6, 1> term_state = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 9, 1> costWeight = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 3, 1> temp_a = Eigen::Matrix<double, 3, 1>::Zero(); // Dynamics Matrix
    Eigen::Matrix<double, 3, 1> temp_b = Eigen::Matrix<double, 3, 1>::Zero(); // Control Transfer Matrix


    float r = distanceThresholdToTarget; // keep 2 m fro the target
    float x4,y4, vx4,vy4, x2,y2, x5,y5, x6,y6, x7,y7, theta, theta_, alpha, beta, gamma;
    float x3 = targetObjectGTPose.pose.pose.position.x + 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.x;
    float y3 = -targetObjectGTPose.pose.pose.position.y - 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.y;

    tf::Quaternion q(
        targetObjectGTPose.pose.pose.orientation.x,
        targetObjectGTPose.pose.pose.orientation.y,
        targetObjectGTPose.pose.pose.orientation.z,
        targetObjectGTPose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double target_roll, target_pitch, target_yaw;
    m.getRPY(target_roll, target_pitch, target_yaw);

    float x1 = selfPose.x;
    float y1 = selfPose.y;
    int flag, direction;
    x2 = matePose.x;
    y2 = matePose.y;

    if(useZeroAsFixedTarget)
    {
        x3 = 0.0;
        y3 = 0.0;
    }
    else
    {
      x3 = targetObjectGTPose.pose.pose.position.x + 15 * HUMAN_SPEED * cos(target_yaw)* INTERNAL_SUB_STEP;
      y3 = targetObjectGTPose.pose.pose.position.y - 15 * HUMAN_SPEED * sin(target_yaw)* INTERNAL_SUB_STEP;
    }

    if (abs(x3 - x3_prev) < 0.1 && abs(y3 - y3_prev) < 0.1)
    {
        flag = 1;
    }

    else
    {
        flag = 0;
    }

    //State Cost
    costWeight(0) = attractionWeight; //x
    costWeight(1) = attractionWeight; //vx
    costWeight(2) = attractionWeight; //y
    costWeight(3) = attractionWeight; //vy
    costWeight(4) = 0; //z
    costWeight(5) = 0; //vz
    //Control Input Cost
    costWeight(6) = 0;       //ax
    costWeight(7) = 0;       //ay
    costWeight(8) = 0;       //az


    //////////////////////////////////////////////////////////////////////////////
    theta = atan2(y1-y3,x1-x3);


    if (theta > 0 && flag == 0)
        direction = 1;
    else if(theta <=0 && flag ==0)
        direction = -1;


    x4 = x3 + r*cos(theta);
    y4 = y3 + r*sin(theta);
    float z4 = 0;
    vx4 = HUMAN_SPEED*cos(theta);
    vy4 = HUMAN_SPEED*sin(theta);

    // Terminal State to Compute Gradients
    term_state(0) = x4;
    term_state(1) = vx4; // want the robot to stop there at the destination
    term_state(2) = y4;
    term_state(3) = vy4;
    term_state(4) = 0;
    term_state(5) = 0;


    repositionDestinationDueToStaticObstacle(x4,y4,z4,x3,y3); // do this in NWU


    gradientVector = Eigen::Matrix<double, 9, 16>::Zero();
    sumOfGradients=0;
    // For terminal state objective
    for(int j=0; j<4; j++)
    {
        gradientVector(j,15) = 2*(StateInput(j,15)-term_state(j))*costWeight(j);
        //ROS_INFO("StateInput(%d,15)-term_state(%d) = %f",j,j,StateInput(j,15)-term_state(j));
    }

    //For control inputs objective
    for(int j=6; j<9; j++)
    {
        //temp_a(j-6) = 2*(StateInput(j,34)+ExtForce(j-6)+obstacle_force(j-6,15))*costWeight(j);
        gradientVector(j,15) = 0;//temp_a(j-6);
    }
    // storing in matrices for second term of control gradient
    for (int j=0;j<6;j+=2)
    {
        int current_index = j/2;
        temp_b(current_index) =
            2*costWeight(j)*(StateInput(j)-term_state(j)+StateInput(j+1)*deltaT+StateInput(current_index+6)*0.5*pow(deltaT,2))*pow(deltaT,2)*0.5 +
            2*costWeight(j+1)*(StateInput(j+1)+StateInput(current_index+6)*deltaT)*deltaT;
        gradientVector(current_index+6,15)+=temp_b(current_index);
    }


    for(int j=0; j<9; j++)
    {
        sumOfGradients += gradientVector(j,15);
    }
    //////////////////////////////////////////////////////////////////////////////

    sumOfGradients = -abs(sumOfGradients);
    if (isnan(-sumOfGradients))
        sumOfGradients=0;

    ROS_INFO("Gradient of Optimization = %f",-abs(sumOfGradients)/attractionWeight);

    x4 = x3 + r*cos(theta+ deltaT*abs(sumOfGradients)/attractionWeight);//+diminishingOmega*deltaT);
    y4 = y3 + r*sin(theta+ deltaT*abs(sumOfGradients)/attractionWeight);//+diminishingOmega*deltaT);

    // filling the terminal state
    term_state(0) = x4;
    term_state(1) = vx4; // want the robot to stop there at the destination
    term_state(2) = y4;
    term_state(3) = vy4;
    term_state(4) = 0;
    term_state(5) = 0;



    // filling the current state
    cur_state(0) = selfPose.x;
    cur_state(1) = 0; //********* Fill the right velocity fill the velocity later with another data type.. for now it is 0
    cur_state(2) = selfPose.y;
    cur_state(3) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(4) = 0;
    cur_state(5) = 0; // fill the velocity later with another data type.. for now it is 0

    //now the state limits
    state_limits(0,0) = -20;        state_limits(0,1) = 20;
    state_limits(1,0) = -0.5;        state_limits(1,1) = 0.5;
    state_limits(2,0) = -20;        state_limits(2,1) = 20;
    state_limits(3,0) = -0.5;        state_limits(3,1) = 0.5;
    state_limits(4,0) = 0;        state_limits(4,1) = 0;
    state_limits(5,0) = 0;         state_limits(5,1) = 0;

    input_limits(0,0) = -0.5;         input_limits(0,1) = 0.5;
    input_limits(1,0) = -0.5;         input_limits(1,1) = 0.5;
    input_limits(2,0) =  0;         input_limits(2,1) = 0;

    avoidTeamMates_byComputingExtForce(sumOfGradients,direction); // also fills in selfPoseTraj message

    //State Cost
    costWeight(0) = attractionWeight; //x
    costWeight(1) = attractionWeight; //vx
    costWeight(2) = attractionWeight; //y
    costWeight(3) = attractionWeight; //vy
    costWeight(4) = 0; //z
    costWeight(5) = 0; //vz

    StateInput = solveMPC.OCPsolDesigner(deltaT, obstacle_force, ExtForce, cur_state, term_state, costWeight, state_limits, input_limits);

    x3_prev = x3;y3_prev = y3;
    //ROS_INFO("attractionWeight = %f",attractionWeight);

    //ROS_INFO("diminishingOmega = %f",diminishingOmega);
    //  Publish most recent output of nmpc
    outPose.x =  StateInput(0,15);
    // outPose.velocity.x =  StateInput(1,15);
    outPose.y =  StateInput(2,15);

    outPose.theta = theta;
    // outPose.velocity.y = -StateInput(3,15);

    wayPoint.generic.target.x = StateInput(0,15);
    wayPoint.generic.target.y = StateInput(2,15);
    wayPoint.generic.frame = 3;
    // wayPoint.bezier.targetTranslation = selfPoseTraj2OpenBase;
    // wayPoint.bezier.step = 0.01;
    // wayPoint.bezier.frame = 3;


    // selfPoseTraj.header = msg->header;

    pubOutPoseSelf_.publish(outPose);
    pubMatlabPoseSelf.publish(wayPoint);
    pubSelfPoseTraj_.publish(selfPoseTraj);

    #endif
}

void Planner::selfIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ///@TODO uncomment the lines below when necessary. For now it is all zero anyway.
  selfIMUReading.header = msg->header;
  selfIMUReading.orientation = msg->orientation;
  selfIMUReading.angular_velocity = msg->angular_velocity;
  //selfIMUReading.angular_velocity_covariance = msg->angular_velocity_covariance;
  selfIMUReading.linear_acceleration = msg->linear_acceleration;
  //selfIMUReading.header_covariance = msg->linear_acceleration_covariance;
}


void Planner::reconf_callback(minion_nmpc_planner::nmpcPlannerParamsConfig &config) //R:Dynamic Reconfigure?
{
  ROS_INFO("Reconfigure Request: repuGain = %f attractGain = %f safeNeighDist = %f  distToTarget = %f   copterDesiredHeightinNED = %f",config.tFormationRepulGain, config.tGoalAttractionGain,config.neighborDistThreshold, config.distanceThresholdToTarget, config.copterDesiredHeightinNED);

  ROS_INFO("Reconfigure Request: INTERNAL_SUB_STEP = %f deltaT = %f",config.INTERNAL_SUB_STEP, config.deltaT);

  tFormationRepulGain=config.tFormationRepulGain;

  tGoalAttractionGain=config.tGoalAttractionGain;

  maxObstacleRepulsionForce = config.maxObstacleRepulsionForce;

  neighborDistThreshold=config.neighborDistThresholdSIM;

  distanceThresholdToTarget=config.distanceThresholdToTargetSIM;

  copterDesiredHeightinNED=config.copterDesiredHeightinNED_SIM;

  INTERNAL_SUB_STEP=config.INTERNAL_SUB_STEP;

  deltaT=config.deltaT;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "nmpc_planner");

  if (argc < 3)
    {
      ROS_WARN("WARNING: you should specify i) selfID and ii) the number of robots in the team including self\n");
      return 1;
    }
  else
  {
    ROS_INFO("nmpc_planner running for = %s using %s robots in the team including self",argv[1],argv[2]);
  }

  ros::NodeHandle nh("~");
  Planner node(&nh,atoi(argv[1]),atoi(argv[2]));
  spin();

  return 0;
}
