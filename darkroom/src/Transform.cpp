#include "darkroom/Transform.hpp"

namespace DarkRoom {

    void Transform::getRTmatrix(Matrix4d &RT, VectorXd &pose){
        RT = Matrix4d::Identity();
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(pose(0), 2.0) + pow(pose(1), 2.0) + pow(pose(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * pose(0) / (alpha_squared + 1),
                      2.0 * pose(1) / (alpha_squared + 1),
                      2.0 * pose(2) / (alpha_squared + 1));
        q.normalize();
        // construct RT matrix
        RT.topLeftCorner(3, 3) = q.toRotationMatrix();
        RT.topRightCorner(3, 1) << pose(3), pose(4), pose(5);
    }

    void Transform::getPose(Quaterniond &q, Vector3d &origin, VectorXd &pose){
        double alpha_squared = pow(pow(pose(0), 2.0) + pow(pose(1), 2.0) + pow(pose(2), 2.0), 2.0);
        q = Quaterniond((1 - alpha_squared) / (alpha_squared + 1),
                        2.0 * pose(0) / (alpha_squared + 1),
                        2.0 * pose(1) / (alpha_squared + 1),
                        2.0 * pose(2) / (alpha_squared + 1));
        q.normalize();
        origin = Vector3d(pose(3), pose(4), pose(5));
    }

    void Transform::getTFtransform(VectorXd &x, tf::Transform &tf){
        tf.setOrigin(tf::Vector3(x(3), x(4), x(5)));

// construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0), 2.0) + pow(x(1), 2.0) + pow(x(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * x(0) / (alpha_squared + 1),
                      2.0 * x(1) / (alpha_squared + 1),
                      2.0 * x(2) / (alpha_squared + 1));
        q.normalize();
        Matrix3d rot = q.toRotationMatrix();
        tf::Matrix3x3 rot_matrix(rot(0, 0), rot(0, 1), rot(0, 2),
                                 rot(1, 0), rot(1, 1), rot(1, 2),
                                 rot(2, 0), rot(2, 1), rot(2, 2));

        tf.setBasis(rot_matrix);
    }

    void Transform::getTFtransform(Matrix4d &RT, tf::Transform &tf){
        tf::Matrix3x3 rot_matrix(RT(0, 0), RT(0, 1), RT(0, 2),
                                 RT(1, 0), RT(1, 1), RT(1, 2),
                                 RT(2, 0), RT(2, 1), RT(2, 2));
        tf.setBasis(rot_matrix);
        tf.setOrigin(tf::Vector3(RT(0,3),RT(1,3),RT(2,3)));
    }

    void Transform::publishTF(tf::Transform &tf, const char *frame, const char *name){
        tf_broadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), frame, name));
    }
}