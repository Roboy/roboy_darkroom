#include "darkroom/Triangulation.hpp"

double Triangulation::dist3D_Line_to_Line( Vector3d &pos0, Vector3d &dir1,
                            Vector3d &pos1, Vector3d &dir2,
                            Vector3d &tri0, Vector3d &tri1) {
    Vector3d   u = dir1;
    Vector3d   v = dir2;
    Vector3d   w = pos0 - pos1;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, tc;

    // compute the line parameters of the two closest points
    if (D < 0.0000001) {          // the lines are almost parallel
        sc = 0.0;
        tc = (b>c ? d/b : e/c);    // use the largest denominator
    }
    else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    // get the difference of the two closest points
    Vector3d   dP = w + (sc * u) - (tc * v);  // =  L1(sc) - L2(tc)
    tri0 = sc*dir1;
    tri1 = tc*dir2;
    return dP.norm();   // return the closest distance
}

double Triangulation::triangulateFromLighthouseAngles(Vector2d &angles0, Vector2d &angles1, Matrix4d &RT_0, Matrix4d &RT_1,
                                     Vector3d &triangulated_position, Vector3d &ray0, Vector3d &ray1) {
    ;

    Matrix3d rot0, rot1;
    rot0 = RT_0.topLeftCorner(3, 3);
    rot1 = RT_1.topLeftCorner(3, 3);

    rayFromLighthouseAngles(angles0,ray0,LIGHTHOUSE_A);
    rayFromLighthouseAngles(angles1,ray1,LIGHTHOUSE_B);

    Vector3d ray0_worldFrame, ray1_worldFrame;

    ray0_worldFrame = rot0*ray0;
    ray1_worldFrame = rot1*ray1;

    Vector3d origin0 = RT_0.topRightCorner(3,1), origin1 = RT_1.topRightCorner(3,1);

    Vector3d l0, l1;

    double distance = dist3D_Line_to_Line(origin0, ray0_worldFrame, origin1, ray1_worldFrame, l0, l1);
    triangulated_position = l0+origin0 + (l1+origin1-l0-origin0)/2.0;

    return distance;
}

double Triangulation::triangulateFromRays(Vector3d &ray0, Vector3d &ray1,
                           Matrix4d &RT_0, Matrix4d &RT_1,
                           Vector3d &triangulated_position) {
    Matrix3d rot0, rot1;
    rot0 = RT_0.topLeftCorner(3, 3);
    rot1 = RT_1.topLeftCorner(3, 3);

    Vector3d ray0_worldFrame, ray1_worldFrame;

    ray0_worldFrame = rot0*ray0;
    ray1_worldFrame = rot1*ray1;

    Vector3d origin0 = RT_0.topRightCorner(3,1), origin1 = RT_1.topRightCorner(3,1);

    Vector3d l0, l1;

    double distance = dist3D_Line_to_Line(origin0, ray0_worldFrame, origin1, ray1_worldFrame, l0, l1);
    triangulated_position = l0+origin0 + (l1+origin1-l0-origin0)/2.0;

    return distance;
}

void Triangulation::rayFromLighthouseAngles(Vector2d &angles, Vector3d &ray, bool lighthouse) {
    double elevation = angles(0), azimuth = angles(1);
    elevation += calibration[lighthouse][VERTICAL].phase;
    elevation += calibration[lighthouse][VERTICAL].curve*pow(sin(elevation)*cos(azimuth),2.0)
                 + calibration[lighthouse][VERTICAL].gibmag*cos(elevation+calibration[lighthouse][VERTICAL].gibphase);
    azimuth += calibration[lighthouse][HORIZONTAL].phase;
    azimuth += calibration[lighthouse][HORIZONTAL].curve*pow(cos(elevation),2.0)
               + calibration[lighthouse][HORIZONTAL].gibmag*cos(azimuth+calibration[lighthouse][HORIZONTAL].gibphase);
    Vector3d v(0, sin(elevation), -cos(elevation));
    Vector3d h(cos(azimuth), sin(azimuth), 0);
    Vector3d nv = v.cross(Vector3d(1,0,0));
    Vector3d nh = h.cross(Vector3d(0,0,1));
    ray = nh.cross(nv);
    ray.normalize();
// TODO use tilt
//    Matrix3d tilt_trafo_vertical = Matrix3d::Identity(), tilt_trafo_horizontal = Matrix3d::Identity();
//    tilt_trafo_vertical.block(0, 0, 3, 3) << cos(calibration[lighthouse][VERTICAL].tilt), 0, sin(calibration[lighthouse][VERTICAL].tilt),
//            0, 1, 0,
//            -sin(calibration[lighthouse][VERTICAL].tilt), 0, cos(calibration[lighthouse][VERTICAL].tilt);
//    tilt_trafo_horizontal.block(0, 0, 3, 3) << cos(calibration[lighthouse][HORIZONTAL].tilt), 0, sin(calibration[lighthouse][HORIZONTAL].tilt),
//            0, 1, 0,
//            -sin(calibration[lighthouse][HORIZONTAL].tilt), 0, cos(calibration[lighthouse][HORIZONTAL].tilt);
//    ray = tilt_trafo_horizontal*tilt_trafo_vertical*Vector3d(sin(elevation)*cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation));

    // this is assuming spherical coordinates (which is not quite true, since the two motor axis dont align)
//    ray = Vector3d(sin(elevation)*cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation));
}