#include "darkroom/InYourGibbousPhase2.hpp"
namespace InYourGibbousPhase2 {

    InYourGibbousPhase2::InYourGibbousPhase2( int trajectoryPoints) :
            Functor<double>(110, 50), trajectoryPoints(trajectoryPoints) {
    }
    int InYourGibbousPhase2::operator()(const VectorXd &x, VectorXd &fvec) const {
        int k = 0;
        double elevation, azimuth;
        fvec.setZero();
        for(int i=0;i<trajectoryPoints;i++){
            double error_elevation = 0, error_azimuth = 0;
            for(int j=0;j<angles_measured_trajectory[i].size();j++){
                elevation = angles_measured_trajectory[i][j](0), azimuth = angles_measured_trajectory[i][j](1);
                elevation += x(phase_vertical);
                elevation += x(curve_vertical)*pow(sin(elevation)*cos(azimuth),2.0) + x(gibmag_vertical)*cos(elevation+x(gibphase_vertical));
                azimuth += x(phase_horizontal);
                azimuth += x(curve_horizontal)*pow(cos(elevation),2.0) + x(gibmag_horizontal)*cos(azimuth+x(gibphase_horizontal));

                double elevation_truth = M_PI -  atan2(distance_y, relative_positions_trajectory[i][j](2)+x(10+(i*2+1)));
                double azimuth_truth = atan2(distance_y, relative_positions_trajectory[i][j](0)+x(10+(i*2)));

                error_elevation += (elevation-elevation_truth);
                error_azimuth += (azimuth-azimuth_truth);

            }
            fvec((i*2)) = error_elevation;
            fvec((i*2)+1) = error_azimuth;
//            cout << elevation << "\t" << azimuth << endl;
        }

        cout << "error : " << fvec.squaredNorm() <<endl;
        // cout << "x : " << x <<endl;
        return 0;
    }

}