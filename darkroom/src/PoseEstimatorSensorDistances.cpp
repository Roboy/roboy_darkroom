#include "darkroom/PoseEstimatorSensorDistances.hpp"

namespace PoseEstimatorSensorDistances {

    PoseEstimator::PoseEstimator(int numberOfSensors, MatrixXd &rays_A, MatrixXd &rays_B) :
            Functor<double>(6, numberOfSensors+3), numberOfSensors(numberOfSensors),
            rays_A(rays_A), rays_B(rays_B){
        pose = VectorXd(6);
        RT_A = MatrixXd::Identity(4, 4);
        cout << rays_A << endl;
        cout << rays_B << endl;
    }

    int PoseEstimator::operator()(const VectorXd &x, VectorXd &fvec) const {
        Matrix4d RT = MatrixXd::Identity(4, 4);
        // construct quaternion (cf unit-sphere projection Terzakis paper)
        double alpha_squared = pow(pow(x(0), 2.0) + pow(x(1), 2.0) + pow(x(2), 2.0), 2.0);
        Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                      2.0 * x(0) / (alpha_squared + 1),
                      2.0 * x(1) / (alpha_squared + 1),
                      2.0 * x(2) / (alpha_squared + 1));
        // construct RT matrix
        RT.topLeftCorner(3, 3) = q.toRotationMatrix();
        RT.topRightCorner(3, 1) << x(3), x(4), x(5);

        MatrixXd new_rays_B(4, numberOfSensors);
        new_rays_B = RT * rays_B;

        cout << "original:\n" << rays_B << endl;
        cout << "new:\n" << new_rays_B << endl;

        MatrixXd sensor0(3, numberOfSensors), sensor1(3, numberOfSensors);

        Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
        proj_matrix0 = RT_A.topLeftCorner(3, 4);
        proj_matrix1 = RT.topLeftCorner(3, 4);

        fvec = VectorXd::Zero(numberOfSensors+3);

        // TODO: use simpler triangulation function
//        for (int i = 0; i < numberOfSensors; i++) {
//            // project onto image plane
//            Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(rays_A(0, i) / rays_A(2, i),
//                                                                        rays_A(1, i) / rays_A(2, i));
//            Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(new_rays_B(0, i) / new_rays_B(2, i),
//                                                                        new_rays_B(1, i) / new_rays_B(2, i));
//            Vector3d pos = triangulate_point(proj_matrix0, proj_matrix1,
//                                              projected_image_location0, projected_image_location1);
//            Vector3d ray(rays_A(0,i), rays_A(1,i), rays_A(2,i));
//            fvec(i) = (-pos.cross(-ray)).norm()/ray.norm();
//            cout << fvec(i) << endl;
//        }
//        for (int i = 1; i < numberOfSensors; i++) {
//            fvec(i) = difference(0, i);
//        }

//        cout << "RT\n" << RT << endl;
        cout << "error : " << fvec.squaredNorm() << endl;
        cout << "x : " << x << endl;
        return 0;
    }

}