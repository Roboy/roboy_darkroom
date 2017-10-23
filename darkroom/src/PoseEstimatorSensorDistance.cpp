#include "darkroom/PoseEstimatorSensorDistance.hpp"

namespace PoseEstimatorSensorDistance {

    int PoseEstimator::counter = 0;

    PoseEstimator::PoseEstimator(int numberOfSamples, double distanceBetweenSensors,
                                                     MatrixXd &rays0_A, MatrixXd &rays0_B,
                                                     MatrixXd &rays1_A, MatrixXd &rays1_B) :
            Functor<double>(6, 1 * numberOfSamples), numberOfSamples(numberOfSamples),
            distanceBetweenSensors(distanceBetweenSensors), rays0_A(rays0_A),
            rays0_B(rays0_B), rays1_A(rays1_A), rays1_B(rays1_B) {
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


        Matrix4d RT_B_corrected;
        RT_B_corrected = RT*RT_B;

        Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
        proj_matrix0 = RT_A.topLeftCorner(3, 4);
        proj_matrix1 = RT_B_corrected.topLeftCorner(3, 4);

        // TODO: use simpler triangulation function
//        cout << "fvec" << endl;
//        for (int i = 0; i < numberOfSamples; i++) {
//            // project onto image plane
//            Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(rays0_A(0, i) / rays0_A(2, i),
//                                                                        rays0_A(1, i) / rays0_A(2, i));
//            Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(rays0_B(0, i) / rays0_B(2, i),
//                                                                        rays0_B(1, i) / rays0_B(2, i));
//
//            Vector3d pos0 = triangulate_point(proj_matrix0, proj_matrix1,
//                                              projected_image_location0, projected_image_location1);
//
//            projected_image_location0 = Eigen::Vector2d(rays1_A(0, i) / rays1_A(2, i),
//                                                        rays1_A(1, i) / rays1_A(2, i));
//            projected_image_location1 = Eigen::Vector2d(rays1_B(0, i) / rays1_B(2, i),
//                                                        rays1_B(1, i) / rays1_B(2, i));
//
//            Vector3d pos1 = triangulate_point(proj_matrix0, proj_matrix1,
//                                              projected_image_location0, projected_image_location1);
//            Vector3d distance = pos1 - pos0;
//            fvec(i) = distance.norm() - distanceBetweenSensors;
////            printf("pos0: %f\t%f\t%f    pos1: %f\t%f\t%f\n", pos0(0), pos0(1), pos0(2), pos1(0), pos1(1), pos1(2));
////            printf("%f\t", distance.norm());
//        }
//        cout << endl;
        counter++;

        cout << "iteration: " << counter << endl;
        cout << "RT_B_corrected\n" << RT_B_corrected << endl;
        cout << "error : " << fvec.squaredNorm() << endl;
        cout << "x : " << x << endl;
        return 0;
    }
}