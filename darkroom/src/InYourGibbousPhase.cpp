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
            elevation += x(curve_vertical)*pow(sin(elevation)*cos(azimuth),2.0) + x(gibmag_vertical)*cos(elevation+x(gibphase_vertical));
            azimuth += x(phase_horizontal);
            azimuth += x(curve_horizontal)*pow(cos(elevation),2.0) + x(gibmag_horizontal)*cos(azimuth+x(gibphase_horizontal));

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