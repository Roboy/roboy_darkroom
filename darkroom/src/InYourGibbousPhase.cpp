#include "darkroom/InYourGibbousPhase.hpp"
namespace InYourGibbousPhase {

    InYourGibbousPhase::InYourGibbousPhase(int numberOfSensors) :
            Functor<double>(10, 2 * numberOfSensors), numberOfSensors(numberOfSensors) {
    }
    enum {phase0 , phase1, tilt0, tilt1, curve0, curve1, gibphase0, gibphase1, gibmag0, gibmag1};
    int InYourGibbousPhase::operator()(const VectorXd &x, VectorXd &fvec) const {
        int j = 0;
        double elevation, azimuth;
        fvec.setZero();
        for(int i=0;i<numberOfSensors;i++){
            elevation = elevation_measured[i], azimuth = azimuth_measured[i];

            elevation += x(phase0) + x(curve0)*pow(sin(elevation)*cos(azimuth),2.0) + x(gibmag0)*cos(elevation+x(gibphase0));
            azimuth += x(phase1) + x(curve1)*pow(cos(elevation),2.0) + x(gibmag1)*cos(azimuth+x(gibphase1));

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