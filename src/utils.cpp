#include "robotics_project/utils.h"

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

Eigen::Vector3d lerp(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t) {
   double t_norm = t / TOTAL_TIME;
   return start + t_norm * (end - start);
}
Eigen::Quaterniond slerp(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end, double t) {
   double t_norm = t / TOTAL_TIME;
   return start.slerp(t_norm, end);
}
Eigen::Matrix4d generalTransformationMatrix(double theta, double alpha, double d, double a) {
   return Eigen::Matrix4d{{cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)},
                          {sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
                          {0, sin(alpha), cos(alpha), d},
                          {0, 0, 0, 1}};
}

Eigen::Matrix<double, 6, 6> getJacobian(const Eigen::Matrix<double, 6, 1>& joints) {
   Eigen::Matrix<double, 6, 6> jacobian;
   jacobian.setZero();
   jacobian.col(0) << d_DH(4) * (cos(joints(0)) * cos(joints(4)) +
                                 cos(joints(1) + joints(2) + joints(3)) * sin(joints(0)) * sin(joints(4))) +
                          d_DH(3) * cos(joints(0)) - a_DH(1) * cos(joints(1)) * sin(joints(0)) -
                          d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * sin(joints(0)) -
                          a_DH(2) * cos(joints(1)) * cos(joints(2)) * sin(joints(0)) +
                          a_DH(2) * sin(joints(0)) * sin(joints(1)) * sin(joints(2)),
       d_DH(4) * (cos(joints(4)) * sin(joints(0)) -
                  cos(joints(1) + joints(2) + joints(3)) * cos(joints(0)) * sin(joints(4))) +
           d_DH(3) * sin(joints(0)) + a_DH(1) * cos(joints(0)) * cos(joints(1)) +
           d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * cos(joints(0)) +
           a_DH(2) * cos(joints(0)) * cos(joints(1)) * cos(joints(2)) -
           a_DH(2) * cos(joints(0)) * sin(joints(1)) * sin(joints(2)),
       0, 0, 0, 1;
   jacobian.col(1) << -cos(joints(0)) * (a_DH(2) * sin(joints(1) + joints(2)) + a_DH(1) * sin(joints(1)) +
                                         d_DH(4) * (sin(joints(1) + joints(2)) * sin(joints(3)) -
                                                    cos(joints(1) + joints(2)) * cos(joints(3))) -
                                         d_DH(4) * sin(joints(4)) *
                                             (cos(joints(1) + joints(2)) * sin(joints(3)) +
                                              sin(joints(1) + joints(2)) * cos(joints(3)))),
       -sin(joints(0)) *
           (a_DH(2) * sin(joints(1) + joints(2)) + a_DH(1) * sin(joints(1)) +
            d_DH(4) * (sin(joints(1) + joints(2)) * sin(joints(3)) - cos(joints(1) + joints(2)) * cos(joints(3))) -
            d_DH(4) * sin(joints(4)) *
                (cos(joints(1) + joints(2)) * sin(joints(3)) + sin(joints(1) + joints(2)) * cos(joints(3)))),
       a_DH(2) * cos(joints(1) + joints(2)) - (d_DH(4) * sin(joints(1) + joints(2) + joints(3) + joints(4))) / 2 +
           a_DH(1) * cos(joints(1)) + (d_DH(4) * sin(joints(1) + joints(2) + joints(3) - joints(4))) / 2 +
           d_DH(4) * sin(joints(1) + joints(2) + joints(3)),
       sin(joints(0)), -cos(joints(0)), 0;
   jacobian.col(2) << cos(joints(0)) *
                          (d_DH(4) * cos(joints(1) + joints(2) + joints(3)) - a_DH(2) * sin(joints(1) + joints(2)) +
                           d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       sin(joints(0)) * (d_DH(4) * cos(joints(1) + joints(2) + joints(3)) - a_DH(2) * sin(joints(1) + joints(2)) +
                         d_DH(4) * sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       a_DH(2) * cos(joints(1) + joints(2)) - (d_DH(4) * sin(joints(1) + joints(2) + joints(3) + joints(4))) / 2 +
           (d_DH(4) * sin(joints(1) + joints(2) + joints(3) - joints(4))) / 2 +
           d_DH(4) * sin(joints(1) + joints(2) + joints(3)),
       sin(joints(0)), -cos(joints(0)), 0;
   jacobian.col(3) << d_DH(4) * cos(joints(0)) *
                          (cos(joints(1) + joints(2) + joints(3)) +
                           sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       d_DH(4) * sin(joints(0)) *
           (cos(joints(1) + joints(2) + joints(3)) + sin(joints(1) + joints(2) + joints(3)) * sin(joints(4))),
       d_DH(4) * (sin(joints(1) + joints(2) + joints(3) - joints(4)) / 2 + sin(joints(1) + joints(2) + joints(3)) -
                  sin(joints(1) + joints(2) + joints(3) + joints(4)) / 2),
       sin(joints(0)), -cos(joints(0)), 0;
   jacobian.col(4) << d_DH(4) * cos(joints(0)) * cos(joints(1)) * cos(joints(4)) * sin(joints(2)) * sin(joints(3)) -
                          d_DH(4) * cos(joints(0)) * cos(joints(1)) * cos(joints(2)) * cos(joints(3)) * cos(joints(4)) -
                          d_DH(4) * sin(joints(0)) * sin(joints(4)) +
                          d_DH(4) * cos(joints(0)) * cos(joints(2)) * cos(joints(4)) * sin(joints(1)) * sin(joints(3)) +
                          d_DH(4) * cos(joints(0)) * cos(joints(3)) * cos(joints(4)) * sin(joints(1)) * sin(joints(2)),
       d_DH(4) * cos(joints(0)) * sin(joints(4)) +
           d_DH(4) * cos(joints(1)) * cos(joints(4)) * sin(joints(0)) * sin(joints(2)) * sin(joints(3)) +
           d_DH(4) * cos(joints(2)) * cos(joints(4)) * sin(joints(0)) * sin(joints(1)) * sin(joints(3)) +
           d_DH(4) * cos(joints(3)) * cos(joints(4)) * sin(joints(0)) * sin(joints(1)) * sin(joints(2)) -
           d_DH(4) * cos(joints(1)) * cos(joints(2)) * cos(joints(3)) * cos(joints(4)) * sin(joints(0)),
       -d_DH(4) * (sin(joints(1) + joints(2) + joints(3) - joints(4)) / 2 +
                   sin(joints(1) + joints(2) + joints(3) + joints(4)) / 2),
       sin(joints(1) + joints(2) + joints(3)) * cos(joints(0)), sin(joints(1) + joints(2) + joints(3)) * sin(joints(0)),
       -cos(joints(1) + joints(2) + joints(3));
   jacobian.col(5) << 0, 0, 0,
       cos(joints(4)) * sin(joints(0)) - cos(joints(1) + joints(2) + joints(3)) * cos(joints(0)) * sin(joints(4)),
       -cos(joints(0)) * cos(joints(4)) - cos(joints(1) + joints(2) + joints(3)) * sin(joints(0)) * sin(joints(4)),
       -sin(joints(1) + joints(2) + joints(3)) * sin(joints(4));
   return jacobian;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> directKinematics(
    const Eigen::Matrix<double, 6, 1>& joints) {
   Eigen::Matrix4d transformation_matrix = generalTransformationMatrix(joints(0), alpha_DH(0), d_DH(0), a_DH(0));
   for (int i = 1; i < 6; i++) {
      transformation_matrix *= generalTransformationMatrix(joints(i), alpha_DH(i), d_DH(i), a_DH(i));
   }

   // pe: position end effector -> position wrist3_links
   Eigen::Vector3d pe(transformation_matrix.block(0, 3, 3, 1));
   Eigen::Matrix3d Re(transformation_matrix.block(0, 0, 3, 3));
   return std::make_tuple(pe, Re, transformation_matrix);
}

Eigen::Matrix<double, 8, 1> getJointConfiguration() {
   // index from 0 to 7
   // Shoulder pan joint           index:4
   // Shoulder lift joint          index:3
   // Elbow joint                  index:0
   // Wrist1 joint                 index:5
   // Wrist2 joint                 index:6
   // Wrist3 joint                 index:7
   // Hand1 joint                  index:1
   // Hand2 joint                  index:2
   boost::shared_ptr<sensor_msgs::JointState const> joint_configuration =
       ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
   return Eigen::Matrix<double, 8, 1>{joint_configuration->position[4], joint_configuration->position[3],
                                      joint_configuration->position[0], joint_configuration->position[5],
                                      joint_configuration->position[6], joint_configuration->position[7],
                                      joint_configuration->position[1], joint_configuration->position[2]};
}

Eigen::Vector3d worldToBaseCoordinates(const Eigen::Vector3d& point) {
   Eigen::Matrix4d transformationMatrix{
       {-1.0, 0.0, 0.0, 0.5}, {0.0, 1.0, 0.0, 0.35}, {0.0, 0.0, -1.0, 1.75}, {0.0, 0.0, 0.0, 1.0}};

   Eigen::Vector4d tmpPoint = transformationMatrix.inverse() * Eigen::Vector4d{point(0), point(1), point(2), 1.0};
   return Eigen::Vector3d{tmpPoint(0), tmpPoint(1), tmpPoint(2)};
}

void insertPath(Path& path, const Eigen::Matrix<double, 8, 1>& jointConfiguration) {
   path.conservativeResize(path.rows() + 1, path.cols());
   path.row(path.rows() - 1) = jointConfiguration;
}

Path moveRobot(const Eigen::Vector3d& finalPosition, const Eigen::Matrix3d& finalRotationMatrix) {
   Eigen::Matrix<double, 8, 1> jointConfiguration = getJointConfiguration();
   Eigen::Matrix<double, 6, 1> jointState =
       Eigen::Matrix<double, 6, 1>{jointConfiguration(0), jointConfiguration(1), jointConfiguration(2),
                                   jointConfiguration(3), jointConfiguration(4), jointConfiguration(5)};
   auto [pe, Re, transformationMatrix] = directKinematics(jointState);

   Eigen::Quaterniond initialQuaternion(Re);
   Eigen::Quaterniond finalQuaternion(finalRotationMatrix);

   return differentialKinematicsQuaternion(jointConfiguration, pe, initialQuaternion, finalPosition, finalQuaternion);
}

double calculateDeterminantJJT(const Eigen::Matrix<double, 6, 6>& jacobian) {
   Eigen::Matrix<double, 6, 6> JJT = jacobian * jacobian.transpose();
   return std::sqrt(abs(JJT.determinant()));
}

double calculateDampingFactor(double w, double wt, double lambda0) {
   if (w < wt) {
      return lambda0 * std::pow(1.0 - (w / wt), 2);
   } else {
      ROS_INFO("NO DAMPING");
      return 0.0;
   }
}
Eigen::Matrix<double, 6, 6> calculateDampedPseudoInverse(const Eigen::Matrix<double, 6, 6>& jacobian, double lambda) {
   return jacobian.transpose() *
          (jacobian * jacobian.transpose() + lambda * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
}

Path differentialKinematicsQuaternion(const Eigen::Matrix<double, 8, 1>& jointConfiguration,
                                      const Eigen::Vector3d& initialPosition,
                                      const Eigen::Quaterniond& initialQuaternion, const Eigen::Vector3d& finalPosition,
                                      const Eigen::Quaterniond& finalQuaternion) {
   Eigen::Matrix<double, 6, 6> jacobian_instantK, pseudoInverseJacobian_instantK;

   Eigen::Vector2d gripperState{jointConfiguration(6), jointConfiguration(7)};

   Eigen::Matrix<double, 6, 1> jointState_instantK, jointStateDot_instantK;
   Path path;

   Eigen::Quaterniond quaternion_instantK;
   Eigen::Matrix<double, 6, 1> velocities_instantK;

   Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity() * 10;
   Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity() * 1;
   jointState_instantK = jointConfiguration.head(6);
   insertPath(path, jointConfiguration);

   for (double instantK = 0.0; instantK < TOTAL_TIME; instantK += TIME_STEP) {
      auto [pe_instantK, Re_instantK, transformationMatrix_instantK] = directKinematics(jointState_instantK);
      quaternion_instantK = Eigen::Quaterniond{Re_instantK};

      Eigen::Vector3d positionalVelocity_instantK = (lerp(initialPosition, finalPosition, instantK) -
                                                     lerp(initialPosition, finalPosition, instantK - TIME_STEP)) /
                                                    TIME_STEP;
      Eigen::Quaterniond quaternionVelocity_instantK = slerp(initialQuaternion, finalQuaternion, instantK + TIME_STEP) *
                                                       slerp(initialQuaternion, finalQuaternion, instantK).conjugate();
      Eigen::Vector3d angularVelocity_instantK = (quaternionVelocity_instantK.vec() * 2.0) / TIME_STEP;

      jacobian_instantK = getJacobian(jointState_instantK);
      static double lambda0 = 0.00015;
      static double wt = 0.01;
      double w = calculateDeterminantJJT(jacobian_instantK);
      double lambda = calculateDampingFactor(w, wt, lambda0);
      ROS_INFO("w: %f", w);
      ROS_INFO("lambda: %f", lambda);
      pseudoInverseJacobian_instantK = calculateDampedPseudoInverse(jacobian_instantK, lambda);
      ROS_INFO("PSEUDOINVERSE");
      std::string pseudoInverseJacobian_instantK_str = "";
      for (int i = 0; i < 6; i++) {
         for (int j = 0; j < 6; j++) {
            pseudoInverseJacobian_instantK_str += std::to_string(pseudoInverseJacobian_instantK(i, j)) + " ";
         }
         pseudoInverseJacobian_instantK_str += "\n";
      }
      ROS_INFO("%s", pseudoInverseJacobian_instantK_str.c_str());
      if (abs(jacobian_instantK.determinant()) < 1.0e-5) {
         ROS_WARN("NEAR SINGULARITY");
      }

      Eigen::Vector3d positionError_instantK = lerp(initialPosition, finalPosition, instantK) - pe_instantK;
      Eigen::Quaterniond quaternionError_instantK =
          slerp(initialQuaternion, finalQuaternion, instantK) * quaternion_instantK.conjugate();

      velocities_instantK << positionalVelocity_instantK + (Kp * positionError_instantK),
          angularVelocity_instantK + (Kq * quaternionError_instantK.vec());

      jointStateDot_instantK = pseudoInverseJacobian_instantK * velocities_instantK;
      jointState_instantK += jointStateDot_instantK * TIME_STEP;
      Eigen::Matrix<double, 8, 1> path_instantK;
      path_instantK << jointState_instantK, gripperState;
      insertPath(path, path_instantK);
      //   ROS_INFO("%f", instantK);
   }

   return path;
}