//
// Created by Thomas Michelot on 1/3/21.
//

#include "Convert.hh"

double Convert::intermediateXFromPixelC(int c) {
    return (c + COLUMN_OFFSET - COFF) * (1 << 16) / CFAC;
}

double Convert::intermediateYFromPixelL(int L) {
    return (L + LINE_OFFSET - LOFF) * (1 << 16) / LFAC;
}

double Convert::getSA(double x, double y) {
    return pow(SAT_HEIGHT*cos(x) * cos(y), 2) - (pow(cos(y), 2) + SQUARED_ELLIPSOID_FLATNESS * pow(sin(y), 2)) * WGS84_CTE;
}

double Convert::getSN(double sd, double x, double y) {
    return (SAT_HEIGHT * cos(x) * cos(y) - sd) / (cos(y) * cos(y) + SQUARED_ELLIPSOID_FLATNESS * sin(y) * sin(y));
}

double Convert::getS1(double sn, double x, double y) {
    return SAT_HEIGHT - sn * cos(x) * cos(y);
}

double Convert::getS2(double sn, double x, double y) {
    return sn * sin(x) * cos(y);
}

double Convert::getS3(double sn, double y) {
    return - sn * sin(y);
}

double Convert::getSXY(double sn, double x, double y) {
    return sqrt(pow(getS1(sn, x, y), 2) + pow(getS2(sn, x, y), 2));
}

double Convert::latitudeFromIntermediate(double x, double y) {
    double sa = getSA(x, y);
    if (sa < 0) {
        return COORD_ERROR;
    }
    double sd = sqrt(sa);
    double sn = getSN(sd, x, y);
    return radianToDegree(atan(SQUARED_ELLIPSOID_FLATNESS * getS3(sn, y) / getSXY(sn, x, y)));
}

double Convert::longitudeFromIntermediate(double x, double y) {
    double sa = getSA(x, y);
    if (sa < 0) {
        return COORD_ERROR;
    }
    double sd = sqrt(sa);
    double sn = getSN(sd, x, y);
    return radianToDegree(atan(getS2(sn, x, y) / getS1(sn, x, y)) + SUB_LON);
}

double Convert::getAngularAtoB(double lat_A, double lon_A,
		               double lat_B, double lon_B) {
    double lata = degreeToRadian(lat_A);
    double lona = degreeToRadian(lon_A);
    double latb = degreeToRadian(lat_B);
    double lonb = degreeToRadian(lon_B);

    return  acos(sin(lata) * sin(latb) + cos(lata) * cos(latb) * cos(abs(lonb - lona)));
}

double Convert::getDistance(double angle) {
    return EARTH_RADIUS * angle;
}

double Convert::getSpeed(double lat_A, double lon_A,
		         double lat_B, double lon_B) {
    return getDistance(getAngularAtoB(lat_A, lon_A, lat_B, lon_B)) / DELTA_TIME;
}

double Convert::getOrientation(double lat_A, double lon_A,
		               double lat_B, double lon_B) {
    double lata = degreeToRadian(lat_A);
    double lona = degreeToRadian(lon_A);
    double latb = degreeToRadian(lat_B);
    double lonb = degreeToRadian(lon_B);

    return asin(cos(latb)* sin(lonb - lona < 0 ? lona - lonb : lonb - lona) / sin(getAngularAtoB(lat_A, lon_A, lat_B, lon_B)));
}

double Convert::getSpeed(const GeoTaggedImage &image0, const GeoTaggedImage &image1) {
    auto lat_A = getLatitude(image0);
    auto lon_A = getLongitude(image0);
    auto lat_B = getLatitude(image1);
    auto lon_B = getLongitude(image1);

    return getSpeed(lat_A, lon_A, lat_B, lon_B);
}

double Convert::getOrientation(const GeoTaggedImage &image0, const GeoTaggedImage &image1) {
    auto lat_A = getLatitude(image0);
    auto lon_A = getLongitude(image0);
    auto lat_B = getLatitude(image1);
    auto lon_B = getLongitude(image1);

    return getOrientation(lat_A, lon_A, lat_B, lon_B);
}

double Convert::getLongitude(const GeoTaggedImage &image) {
    auto interX = Convert::intermediateXFromPixelC(image.coordinateXY_.getX());
    auto interY = Convert::intermediateYFromPixelL(image.coordinateXY_.getY());
    auto longitude = Convert::longitudeFromIntermediate(interX, interY);

    return longitude;
}

double Convert::getLatitude(const GeoTaggedImage &image) {
    auto interX = Convert::intermediateXFromPixelC(image.coordinateXY_.getX());
    auto interY = Convert::intermediateYFromPixelL(image.coordinateXY_.getY());
    auto latitude = Convert::latitudeFromIntermediate(interX, interY);

    return latitude;
}
