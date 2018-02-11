#include "darkroom/InYourGibbousPhase3.hpp"
namespace InYourGibbousPhase3 {

    InYourGibbousPhase3::InYourGibbousPhase3(int numberOfSamples) :
            Functor<double>(10, 2 * numberOfSamples), numberOfSamples(numberOfSamples) {
    }
    int InYourGibbousPhase3::operator()(const VectorXd &x, VectorXd &fvec) const {
        int j = 0;
        double elevation, azimuth;
        fvec.setZero();
        for(int frame=0;frame<numberOfSamples;frame++){
            for(int i=0;i<elevation_measured[frame].size();i++) {
                elevation = elevation_measured[frame][i], azimuth = azimuth_measured[frame][i];

                elevation += x(phase_vertical);
                azimuth += x(phase_horizontal);
                double temp_elevation1 = x(curve_vertical) * pow(cos(azimuth) * sin(elevation), 2.0);
                double temp_elevation2 = x(gibmag_vertical) * cos(elevation + x(gibphase_vertical));
                double temp_azimuth1 = x(curve_horizontal) * pow(-sin(azimuth) * cos(elevation), 2.0);
                double temp_azimuth2 = x(gibmag_horizontal) * cos(azimuth + x(gibphase_horizontal));
                elevation += (temp_elevation1 + temp_elevation2);
                azimuth += (temp_azimuth1 + temp_azimuth2);

                fvec(j) += (elevation - elevation_model[frame][i]);
                fvec(j + 1) += (azimuth - azimuth_model[frame][i]);
            }
            j+=2;
        }
        return 0;
    }

}