#include "darkroom/PoseEstimatorMultiLighthouse.hpp"
namespace PoseEstimatorMultiLighthouse {

    PoseEstimator::PoseEstimator(int numberOfSensors) :
            Functor<double>(6, 2 * numberOfSensors), numberOfSensors(numberOfSensors) {
        pose = VectorXd(6);

    }

    int PoseEstimator::operator()(const VectorXd &x, VectorXd &fvec) const {
        Matrix4d RT = MatrixXd::Identity(4, 4);
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0), 2.0) + pow(x(1), 2.0) + pow(x(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * x(0) / (alpha_squared + 1),
                      2.0 * x(1) / (alpha_squared + 1),
                      2.0 * x(2) / (alpha_squared + 1));
        q.normalize();
        // construct RT matrix
        RT.topLeftCorner(3, 3) = q.toRotationMatrix();
        RT.topRightCorner(3, 1) << x(3), x(4), x(5);

        VectorXd rt = VectorXd(12);
        rt << RT(0,0), RT(1,0), RT(2,0), RT(0,1), RT(1,1), RT(2,1), RT(0,2), RT(1,2), RT(2,2), RT(0,3), RT(1,3), RT(2,3);

        MatrixXd CD;
        VectorXd b;
        CD.resize(numberOfSensors*2,12);
        b.resize(numberOfSensors*2);

        for(int i=0; i<numberOfSensors;i++) {
            double u = tan(M_PI_2-azimuths[i]), v = tan(elevations[i]-M_PI_2);
            Vector4d C = (lighthousePose.block(1,0,1,4)*u-lighthousePose.block(0,0,1,4)).transpose();
            Vector4d D = (lighthousePose.block(1,0,1,4)*v-lighthousePose.block(2,0,1,4)).transpose();
            double X = rel_pos[i](0), Y = rel_pos[i](1), Z = rel_pos[i](2);
            CD.block(i*2,0,2,12) << C(0)*X, C(1)*X, C(2)*X, C(0)*Y, C(1)*Y, C(2)*Y, C(0)*Z, C(1)*Z, C(2)*Z, C(0), C(1), C(2),
                                    D(0)*X, D(1)*X, D(2)*X, D(0)*Y, D(1)*Y, D(2)*Y, D(0)*Z, D(1)*Z, D(2)*Z, D(0), D(1), D(2);
            b(i*2) = C(3);
            b(i*2+1) = D(3);
        }

        fvec = CD*rt - b;

        // cout << "RT\n" << RT << endl;
        // cout << "position3D frame A\n" <<  pos3D_A << endl;
        // cout << "position3D frame B\n" <<  pos3D_B << endl;
        // cout << "new position3D frame A\n" <<  new_position3d << endl;
        // cout << "error : " << difference.squaredNorm() <<endl;
        // cout << "x : " << x <<endl;
        return 0;
    }

}