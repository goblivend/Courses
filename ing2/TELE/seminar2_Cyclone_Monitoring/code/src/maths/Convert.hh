//
// Created by Thomas Michelot on 1/3/21.
//

#ifndef TELE_CONVERT_HH
#define TELE_CONVERT_HH

#include <cmath>
#include <data/Coordinate.hh>
#include <metadata/GeoTaggedImage.hh>

class Convert {
public:
    static double getLongitude(const GeoTaggedImage &image);

    static double getLatitude(const GeoTaggedImage &image);

    static double getSpeed(const GeoTaggedImage &image0, const GeoTaggedImage &image1);

    static double getOrientation(const GeoTaggedImage &image0, const GeoTaggedImage &image1);

	static constexpr double LINE_OFFSET = 132 /* UPDATE VALUE */;
	static constexpr double COLUMN_OFFSET = 407 /* UPDATE VALUE */;


private:
    static constexpr double CFAC = 781648343 /* UPDATE VALUE */;
    static constexpr double LFAC = 781648343 /* UPDATE VALUE */;
    static constexpr double COFF = 1857 /* UPDATE VALUE */;
    static constexpr double LOFF = 1857 /* UPDATE VALUE */;
    static constexpr double FLAT_DEG_ANGLE = 180.0;
    static constexpr double SUB_LON = 0.0 /* UPDATE VALUE */;
    static constexpr double COORD_ERROR = -999.999;
    static constexpr double SAT_HEIGHT = 42164;
    /*
     * Those constants are explained here
     * https://github.com/pytroll/pyresample/issues/64
     */
    static constexpr double SQUARED_ELLIPSOID_FLATNESS = 1.006739501;
    static constexpr double DISTANCE_EARTH_SATELLITE = 42164.0;
    /*
     * At page 11 there is this constant is declared
     * https://www.data.jma.go.jp/mscweb/en/himawari89/space_segment/hsd_sample/HS_D_users_guide_en_v13.pdf
     */
    static constexpr double WGS84_CTE = 1737122264.0;
    static constexpr double EARTH_RADIUS = 6371.0 /* UPDATE VALUE */;
    static constexpr double DELTA_TIME = 20 /* UPDATE VALUE */;

    static inline double degreeToRadian(double angle_deg) {
        static constexpr double tmp = M_PI / FLAT_DEG_ANGLE;
        return angle_deg * tmp;
    }

    static inline double radianToDegree(double angle_rad) {
        static constexpr double tmp = FLAT_DEG_ANGLE / M_PI;
        return angle_rad * tmp;
    }

    static double intermediateXFromPixelC(int c);

    static double intermediateYFromPixelL(int l);

    static double longitudeFromIntermediate(double x, double y);

    static double latitudeFromIntermediate(double x, double y);

    static double getSA(double x, double y);

    static double getSN(double sd, double x, double y);

    static double getSXY(double sn, double x, double y);

    static double getS3(double sn, double y);

    static double getS1(double sn, double x, double y);

    static double getS2(double sn, double x, double y);

    static double getAngularAtoB(double lat_A, double lon_A,
		                 double lat_B, double lon_B);

    static double getDistance(double angle_rad);

    static double getSpeed(double lat_A, double lon_A,
		           double lat_B, double lon_B);

    static double getOrientation(double lat_A, double lon_A,
		                 double lat_B, double lon_B);
};

#endif //TELE_CONVERT_HH
