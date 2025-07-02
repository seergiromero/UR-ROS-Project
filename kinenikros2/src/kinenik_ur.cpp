//# Copyright 2020 Institute of Industrial and Control Engineering (IOC)
//#                  Universitat Politecnica de Catalunya
//#                  BarcelonaTech
//#     All Rights Reserved.
//#
//#     This program is free software; you can redistribute it and/or modify
//#     it under the terms of the GNU General Public License as published by
//#     the Free Software Foundation; either version 2 of the License, or
//#     (at your option) any later version.
//#
//#     This program is distributed in the hope that it will be useful,
//#     but WITHOUT ANY WARRANTY; without even the implied warranty of
//#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//#     GNU General Public License for more details.
//#
//#     You should have received a copy of the GNU General Public License
//#     along with this program; if not, write to the
//#     Free Software Foundation, Inc.,
//#     59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//#
//#    Author:   Leopold Palomo-Avellaneda
//#              Néstor García

#include <kinenikros2/kinenik_ur.h>

#include <boost/math/special_functions/sign.hpp>
#include <iostream>
#include <limits> // to manage infinity

// Constructors
KinenikUR::KinenikUR(std::string _model){
    init();
    std::map<std::string,URType>::iterator it;

    std::cout << "Searching for: " << _model << std::endl;
    it = list_names.find(_model);
    if (it != list_names.end()){
        setType((*it).second);
    }
    else{
        std::cout << "Not Universal Robot type found. Using default UR3" << std::endl;
        setType(UR3);
    }
}

KinenikUR::KinenikUR(URType _model){
    init();
    setType(_model);
}

KinenikUR::KinenikUR(){
    init();
}


// Init data
bool KinenikUR::setType(std::string _model)
{
    std::map<std::string,URType>::iterator it;

    std::cout << "Searching for: " << _model << std::endl;
    it = list_names.find(_model);
    if (it != list_names.end()){
        setType((*it).second);
        return  true;
    }
    else
    {
        std::cerr << "Wrong model" << std::endl;
        return false;
    }
}


void KinenikUR::init(){
    const double inf = std::numeric_limits<double>::infinity();
    robot_names = {"UR3", "UR5", "UR10", "UR3e", "UR5e", "UR10e", "UR16e"};

    for (size_t i=0; i < robot_names.size() ; i++)
    {
        list_names.insert({robot_names[i], static_cast<URType>(i)});
    }

    std::map<std::string,URType>::iterator it;

    // setting joint names in the same order of the controller
    joint_name[0] = "shoulder_pan_joint";
    joint_name[1] = "shoulder_lift_joint";
    joint_name[2] = "elbow_joint";
    joint_name[3] = "wrist_1_joint";
    joint_name[4] = "wrist_2_joint";
    joint_name[5] = "wrist_3_joint";

    //e series has infinite joint6 range
    low_limit = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -inf};
    high_limit = {M_PI, M_PI, M_PI, M_PI, M_PI, inf};

}


void KinenikUR::setType(URType _model)
{
    const double inf = std::numeric_limits<double>::infinity();

    //e series has infinite joint6 range
    low_limit = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -inf};
    high_limit = {M_PI, M_PI, M_PI, M_PI, M_PI, inf};

    // ur3 has infinite joint6 range
    // ur5 has not infinite joint6 range
    //ur10 has not infinite joint6 range
    switch(_model) {
       case UR3 :
            alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
            a = {0., 0., 0.24365, 0.21325, 0., 0.};
            d = {0.1519, 0., 0., 0.11235, 0.08535, 0.0819};
            offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
          break;

        case UR5 :
            alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
            a = {0., 0., 0.425, 0.39225, 0., 0.};
            d = {0.089159, 0., 0., 0.10915, 0.09465, 0.0823};
            offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
            low_limit[5] = -2*M_PI;
            high_limit[5] = 2*M_PI;
          break;

       case UR10 :
            alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
            a = {0., 0., 0.425, 0.39225, 0., 0.};
            d = {0.089159, 0., 0., 0.10915, 0.09465, 0.0823};
            offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
            low_limit[5] = -2*M_PI;
            high_limit[5] = 2*M_PI;
       break;

        case UR3e : // values from the urdf
             alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
             a = {0., 0., 0.244, 0.213, 0., 0.};
             d = {0.152, 0., 0., 0.104, 0.085, 0.092};
             offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
           break;

         case UR5e :
             alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
             a = {0., 0., 0.425, 0.39225, 0., 0.};
             d = {0.089159, 0., 0., 0.10915, 0.09465, 0.0823};
             offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
           break;

        case UR10e :
             alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
             a = {0., 0., 0.425, 0.39225, 0., 0.};
             d = {0.089159, 0., 0., 0.10915, 0.09465, 0.0823};
             offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
        break;

        case UR16e :
             alpha = {0., M_PI_2, 0., 0., -M_PI_2, -M_PI_2};
             a = {0., -0.4784, -0.36, 0, 0., 0.};
             d = {0.1807, 0., 0., 0.17415, 0.11985, 0.11655};
             offset = {-M_PI, M_PI, 0., 0., M_PI, 0.};
        break;

    }
    Tbase_2_base_link = getTransfromData(0.,0.,0.,0.,0.,-1.0, 0.0);
    invTbase_2_base_link = getTransfromData(0.,0.,0.,0.,0.,1.0, 0.0);
}


//Returns false if x is not in the interval [xmin-tol, xmax+tol]
//Otherwise, returns true and x is adjuesteb to be in the interval [xmin, xmax]
bool KinenikUR::inInterval(double &x, const double xmin, const double xmax, const double tol) {
    //If x is in the interval
    if ((xmin-tol <= x) && (x <= xmax+tol)) {
        //Adjust x if necessary
        if(x < xmin)
        {
            x = xmin;
        }
        else if(xmax < x) {
                x = xmax;

        }

        return true;

    }
    else {
        return false;
    }
}


//Returns theta in the [-pi, pi] interval
double KinenikUR::saturate(double theta) {
    if (inInterval(theta,-M_PI,M_PI)) {
        return theta;
    } else {
        return theta - boost::math::sign(theta)*2.*M_PI*floor(fabs(theta)/(2.*M_PI) + 0.5);
    }
}


//Adjusts the specified joint value to be the nearest one next to the
//reference, in the physically achievable interval
void KinenikUR::adjust(double &theta, const double theta_ref) {
    double tmp_theta;

    theta = saturate(theta);

    //Calculate the other physically achievable joint value
    if (inInterval(theta,0.,0.)) {//If theta is aproximately equal to zero
        tmp_theta = theta + boost::math::sign(theta_ref)*2.*M_PI;
    } else {
        tmp_theta = theta - boost::math::sign(theta)*2.*M_PI;
    }

    //Update joint value if necessary
    if (fabs(theta_ref-theta) > fabs(theta_ref-tmp_theta)) {
        theta = tmp_theta;
    }

    //theta = theta_ref if |theta-theta_ref| < TOL
    inInterval(theta,theta_ref,theta_ref);
}


//Solves the UR3 inverse kinematics for the 3rd and 4th joints
//and decides if the solution is valid
//joint value will be the nearest one next to the reference
void KinenikUR::findTheta34(const Eigen::AffineCompact3d &transform, Solution *solution,
                  const JointPos &theta_min,
                  const JointPos &theta_max,
                  const double s5, const double c5, const double s6, const double c6,
                  const double  x, const double  y) {
    const double theta2 = solution[0].theta[1]+offset[1];
    const double s2 = sin(theta2);
    const double c2 = cos(theta2);

    const double alpha = atan2(y-a[2]*s2,x-a[2]*c2);

    double theta3 = alpha - theta2 - offset[2];
    adjust(theta3,0.5*(theta_min[2]+theta_max[2]));
    if (inInterval(theta3,theta_min[2],theta_max[2])) {
        solution[0].theta[2] = theta3;

        const double nz = transform(2,0);

        const double oz = transform(2,1);

        const double az = transform(2,2);

        const double beta = atan2(c5*(c6*nz-s6*oz)-s5*az,-nz*s6-oz*c6);

        theta3 += offset[2];

        double theta4 = beta - theta2 - theta3 - offset[3];
        adjust(theta4,0.5*(theta_min[3]+theta_max[3]));
        if (inInterval(theta4,theta_min[3],theta_max[3])) {
            solution[0].theta[3] = theta4;

            solution[0].valid = true;
        }
        else{// else theta4 is out of range
            std::cout<<"theta4 is out of range"<<std::endl;
        }
    }
    else{// else theta3 is out of range
        std::cout<<"theta3 is out of range"<<std::endl;
    }
}


//Solves the UR3 inverse kinematics for the 6th joint and the two possible values of the 2nd joint
//and calls the function of the next joints to be found
//joint value will be the nearest one next to the reference
void KinenikUR::findTheta62(const Eigen::AffineCompact3d &transform, Solution *solution,
                  const JointPos &theta_ref,
                  const JointPos &theta_min,
                  const JointPos &theta_max,
                  const double s1, const double c1, const double tol) {
    const double nx = transform(0,0);
    const double ny = transform(1,0);

    const double ox = transform(0,1);
    const double oy = transform(1,1);

    const double theta5 = solution[0].theta[4]+offset[4];
    const double s5 = sin(theta5);

    double theta6;
    if (fabs(s5) < tol) {
        theta6 = theta_ref[5];
    } else {
        theta6 = atan2((s1*ox-c1*oy)/s5,(-s1*nx+c1*ny)/s5) - offset[5];
        adjust(theta6,0.5*(theta_min[5]+theta_max[5]));
    }
    if (inInterval(theta6,theta_min[5],theta_max[5])) {
        for (unsigned int i = 0; i < 2; ++i) {
            solution[i].theta[5] = theta6;
        }

        theta6 += offset[5];
        const double s6 = sin(theta6) ;
        const double c6 = cos(theta6);

        const double nz = transform(2,0);

        const double oz = transform(2,1);

        const double ax = transform(0,2);
        const double ay = transform(1,2);
        const double az = transform(2,2);

        const double px = transform(0,3);
        const double py = transform(1,3);
        const double pz = transform(2,3);

        const double x = s1*(d[4]*(s6*ny+c6*oy)-ay*d[5]+py)+c1*(d[4]*(s6*nx+c6*ox)-ax*d[5]+px);
        const double y = d[4]*(nz*s6+oz*c6)-az*d[5]-d[0]+pz;

        double c_beta = (a[3]*a[3] - a[2]*a[2] - x*x - y*y) / (2.*a[2]*hypot(x,y));

        if (inInterval(c_beta,-1.,1.)) {
            const double c5 = cos(theta5);

            const double alpha = atan2(-y,-x);
            const double beta = acos(c_beta);

            double theta2;

            theta2 = alpha + beta - offset[1];
            adjust(theta2,0.5*(theta_min[1]+theta_max[1]));
            if (inInterval(theta2,theta_min[1],theta_max[1])) {
                solution[0].theta[1] = theta2;

                findTheta34(transform,&solution[0],theta_min,theta_max,s5,c5,s6,c6,x,y);
            }
            else{// else theta2 is out of range
                std::cout<<"theta2 is out of range"<<std::endl;
            }

            theta2 = alpha - beta - offset[1];
            adjust(theta2,0.5*(theta_min[1]+theta_max[1]));
            if (inInterval(theta2,theta_min[1],theta_max[1])) {
                solution[1].theta[1] = theta2;

                findTheta34(transform,&solution[1],theta_min,theta_max,s5,c5,s6,c6,x,y);
            }
            else{// else theta2 is out of range
                std::cout<<"theta2 is out of range"<<std::endl;
            }
        }
        else{// else theta2 has no solution
            std::cout<<"theta2 has no solution"<<std::endl;
        }
    }
    else{// else theta6 is out of range
        std::cout<<"theta6 is out of range"<<std::endl;
    }
}


//Solves the UR3 inverse kinematics for the two possible values of the 5th joint
//and calls the function of the next joints to be found
//joint value will be the nearest one next to the reference
void KinenikUR::findTheta5(const Eigen::AffineCompact3d &transform, Solution *solution,
                 const JointPos &theta_ref,
                 const JointPos &theta_min,
                 const JointPos &theta_max) {
    const double px = transform(0,3);
    const double py = transform(1,3);

    const double theta1 = solution[0].theta[0]+offset[0];
    const double s1 = sin(theta1);
    const double c1 = cos(theta1);

    double c_alpha = (-px*s1+py*c1+d[3]) / d[5];

    if (inInterval(c_alpha,-1.,1.)) {
        const double alpha = acos(c_alpha);

        double theta5;

        theta5 = alpha - offset[4];
        adjust(theta5,0.5*(theta_min[4]+theta_max[4]));
        if (inInterval(theta5,theta_min[4],theta_max[4])) {
            for (unsigned int i = 0; i < 2; ++i) {
                solution[i].theta[4] = theta5;
            }

            findTheta62(transform,&solution[0],theta_ref,theta_min,theta_max,s1,c1);
        }
        else{// else theta5 is out of range
            std::cout<<"theta5 is out of range"<<std::endl;
        }

        theta5 = -alpha - offset[4];
        adjust(theta5,0.5*(theta_min[4]+theta_max[4]));
        if (inInterval(theta5,theta_min[4],theta_max[4])) {
            for (unsigned int i = 2; i < 4; ++i) {
                solution[i].theta[4] = theta5;
            }

            findTheta62(transform,&solution[2],theta_ref,theta_min,theta_max,s1,c1);
        }
        else{// else theta5 is out of range
            std::cout<<"theta5 is out of range"<<std::endl;
        }
    }
    else{// else theta5 has no solution
            std::cout<<"theta5 has no solution"<<std::endl;
    }
}


//Solves the UR3 inverse kinematics for the two possible values of the 1st joint
//and calls the function of the next joints to be found
//joint value will be the nearest one next to the reference
void KinenikUR::findTheta1(const Eigen::AffineCompact3d &transform, Solution *solution,
                 const JointPos &theta_ref,
                 const JointPos &theta_min,
                 const JointPos &theta_max) {
    const double ax = transform(0,2);
    const double ay = transform(1,2);

    const double px = transform(0,3);
    const double py = transform(1,3);

    const double c_alpha = px - d[5]*ax;
    const double s_alpha = py - d[5]*ay;

    double c_beta = d[3] / hypot(c_alpha,s_alpha);

    if (inInterval(c_beta,-1., 1.)) {
        const double alpha = atan2(s_alpha,c_alpha);
        const double beta = acos(c_beta);

        double theta1;

        theta1 = alpha + beta + M_PI_2 - offset[0];
        adjust(theta1,0.5*(theta_min[0]+theta_max[0]));

        if (inInterval(theta1,theta_min[0],theta_max[0])) {
            for (unsigned int i = 0; i < 4; ++i) {
                solution[i].theta[0] = theta1;
            }

            findTheta5(transform,&solution[0],theta_ref,theta_min,theta_max);
        }
        else{// else theta1 is out of range
            std::cout<<"theta1 is out of range"<<std::endl;
        }

        theta1 = alpha - beta + M_PI_2 - offset[0];
        adjust(theta1,0.5*(theta_min[0]+theta_max[0]));

        if (inInterval(theta1,theta_min[0],theta_max[0])) {
            for (unsigned int i = 4; i < 8; ++i) {
                solution[i].theta[0] = theta1;
            }

            findTheta5(transform,&solution[4],theta_ref,theta_min,theta_max);
        }
        else{// else theta1 is out of range
            std::cout<<"theta1 is out of range"<<std::endl;
        }
    }
    else{// else theta1 has no solution
            std::cout<<"theta1 has no solution"<<std::endl;
    }
}

// destructor

KinenikUR::~KinenikUR(){

}

//Solves the inverse kinematics to place the TCP in the given pose
//Returns all possible valid solutions
// transform is refered wrt base_link
// this function is deprecated
bool KinenikUR::solveIK(const Eigen::AffineCompact3d &transform,
                  std::vector<JointPos> &theta,
                  const JointPos &theta_ref,
                  const JointPos &theta_min,
                  const JointPos &theta_max) {
    Solution solution[8];
    findTheta1(transform,solution,theta_ref,theta_min,theta_max);
    theta.clear();
    for (unsigned int i = 0; i < 8; ++i) {
        if (solution[i].valid){
            theta.push_back(solution[i].theta);
        }
    }
    return theta.size() > 0;
}

//Solves the inverse kinematics to place the TCP in the given pose
//Returns all possible valid solutions in the std::vector theta
//retuns true if have found some pose, else false.
// TCP is wrt base_link!!!

bool KinenikUR::solveIK(const Eigen::AffineCompact3d &tcp_pose, std::vector<JointPos> &theta)
{
    Solution solution[8];
    JointPos theta_ref; theta_ref.fill(0.);
    Eigen::AffineCompact3d tcp_pose_wrt_base_link = Tbase_2_base_link * tcp_pose;
    findTheta1(tcp_pose_wrt_base_link,solution,theta_ref,low_limit,high_limit);
    theta.clear();
    for (unsigned int i = 0; i < 8; ++i) {
        if (solution[i].valid){
            theta.push_back(solution[i].theta);
        }
    }
    return theta.size() > 0;
}

// rx = \gamma = eulerZ;
// ry = \beta = eulerY;
// rz = \alpha = eulerX;

//Solves the inverse kinematics to place the TCP in the given pose
// pose is provided by X, Y, Z, RX, RY, RZ as the teach pendant shows
// so, TCP is refered wrt base
//Returns all possible valid solutions in the std::vector theta
//retuns true if have found some pose, else false.
bool KinenikUR::solveIK(const double x, const double y, const double z,
                        const double eulerX, const double eulerY, const double eulerZ,
                        std::vector<JointPos> &theta)
{
    Eigen::AffineCompact3d tcp_pose = getTransfromData(x, y, z, eulerX, eulerY, eulerZ);
    std::cout << "Sending this T" << std::endl;
    std::cout << tcp_pose.matrix() << std::endl;

    Eigen::Quaterniond qt(tcp_pose.rotation());
    std::cout << "Quaternion values (qx, qy, qz, qw) = " << qt.x() << ", "
                                                         << qt.y() << ", "
                                                         << qt.z() << ", "
                                                         << qt.w() << std::endl;


    return (solveIK(tcp_pose, theta));
}

//Solves the inverse kinematics to place the TCP in the given pose
// pose is provided by X, Y, Z, and Quaternion Qx, Qy, Qz and Qw
// so, TCP is refered wrt base
//Returns all possible valid solutions in the std::vector theta
//retuns true if have found some pose, else false.
bool KinenikUR::solveIK(const double x, const double y, const double z,
                        const double qx, const double qy, const double qz, const double qw,
                        std::vector<JointPos> &theta)
{
    Eigen::AffineCompact3d tcp_pose;
    tcp_pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);

    std::cout << "Sending this T" << std::endl;
    std::cout << tcp_pose.matrix() << std::endl;

    Eigen::Quaterniond qt(tcp_pose.rotation());
    std::cout << "Quaternion values (qx, qy, qz, qw) = " << qt.x() << ", "
                                                         << qt.y() << ", "
                                                         << qt.z() << ", "
                                                         << qt.w() << std::endl;

    return (solveIK(tcp_pose, theta));
}


// Return Eigen::AffineCompact3d from X, Y, Z, RX, RY, RZ
Eigen::AffineCompact3d KinenikUR::getTransfromData(const double x, const double y, const double z,
                                                   const double eulerX, const double eulerY, const double eulerZ)
{
    Eigen::AffineCompact3d transform;
    Eigen::Matrix4d mat;
    //void setEulerYPR(tfScalar eulerZ, tfScalar eulerY,tfScalar eulerX)  {
    // from ros  tf Matrix3x3_8h
    // just nomenclature
    //transform.translation() = Eigen::Vector3d(x,y,z);

    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(x, y, z)));
    Eigen::AngleAxisd rollAngle(eulerZ, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(eulerY, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(eulerX, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle ;
    q.normalize();
    Eigen::Affine3d aq = Eigen::Affine3d(q);

    transform = t*aq;

    double ci ( cos(eulerX));
    double cj ( cos(eulerY));
    double ch ( cos(eulerZ));
    double si ( sin(eulerX));
    double sj ( sin(eulerY));
    double sh ( sin(eulerZ));
    double cc = ci * ch;
    double cs = ci * sh;
    double sc = si * ch;
    double ss = si * sh;

    mat << cj * ch, sj * sc - cs, sj * cc + ss, x,
            cj * sh, sj * ss + cc, sj * cs - sc, y,
            -sj,      cj * si,      cj * ci,  z ,
            0.0 ,         .0 ,         0.0 , 1.0 ;

    std::cout << "Function from data" << std::endl << "--------------------------" << std::endl;
    std::cout << "The T created is " << std::endl << mat << std::endl;
    std::cout << "The T affine3d created is " << std::endl << transform.matrix() << std::endl;

    return transform;
}

// Return Eigen::AffineCompact3d from X, Y, Z and Quaternion Qx, Qy, Qz, Qw
Eigen::AffineCompact3d KinenikUR::getTransfromData(const double x, const double y, const double z,
                                                   const double qx, const double qy, const double qz, const double qw)
{
    Eigen::AffineCompact3d transform;
    transform = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
    return transform;
}


// Debug function
// Print solutions
void KinenikUR::printSolution(std::vector<JointPos> &theta)
{
    // controller format
    std::cout << "Theta vector contains " << theta.size() << " solutions" << std::endl;
    uint sol = 1;
    for (std::vector<JointPos>::iterator it = theta.begin() ; it != theta.end(); ++it){
/*        std::cout << "Solution set in controller order (deg)#" << sol << ": [q1, q2, q3, q4, q5, q6]" << std::endl;
        std::cout << "[";
        for(uint8_t i = 0; i < 5;i++){
            std::cout << rad2deg * it->at(i) << ", ";
        }
        std::cout << rad2deg * it->at(5) << " ]" <<  std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
*/
        std::cout << "Solution set in ROS order#" << sol << ": [";
        for(uint8_t i = 0; i < 5;i++){
            std::cout << joint_name[i] << ", ";
        }
        std::cout << joint_name[5] << " ]" <<  std::endl;

        // ROS format
        //"elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint ]" << std::endl;
//        std::cout << "[" << it->at(2) << ", ";
//        std::cout << it->at(1) << ", ";
//        std::cout << it->at(0) << ", ";

        for(uint8_t i = 0; i < 5;i++){
            std::cout << it->at(i) << ", ";
        }
        std::cout << it->at(5) << " ]" <<  std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;

        sol++;
    }
}
