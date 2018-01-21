#include "darkroom/InYourGibbousPhase.hpp"
namespace InYourGibbousPhase {

    InYourGibbousPhase::InYourGibbousPhase(int numberOfSensors) :
            Functor<double>(10, 2 * numberOfSensors), numberOfSensors(numberOfSensors) {
    }
    int InYourGibbousPhase::operator()(const VectorXd &x, VectorXd &fvec) const {
        int j = 0;
        double elevation, azimuth;
        fvec.setZero();
        for(int i=0;i<numberOfSensors;i++){
            elevation = elevation_measured[i], azimuth = azimuth_measured[i];

            elevation += x(phase_vertical);
            azimuth += x(phase_horizontal);
            double temp_elevation1 = x(curve_vertical)*pow(cos(azimuth)*sin(elevation),2.0);
            double temp_elevation2 = x(gibmag_vertical)*cos(elevation+x(gibphase_vertical));
            double temp_azimuth1 = x(curve_horizontal)*pow(-sin(azimuth)*cos(elevation),2.0);
            double temp_azimuth2 = x(gibmag_horizontal)*cos(azimuth+x(gibphase_horizontal));
            elevation += (temp_elevation1+temp_elevation2);
            azimuth += (temp_azimuth1+temp_azimuth2);

            fvec(j) += (elevation-elevation_truth[i]);
            fvec(j+1) += (azimuth-azimuth_truth[i]);
            j+=2;
//            cout << elevation << "\t" << azimuth << endl;
        }

//        cout << "error : " << fvec.squaredNorm() <<endl;
        // cout << "x : " << x <<endl;
        return 0;
    }

}