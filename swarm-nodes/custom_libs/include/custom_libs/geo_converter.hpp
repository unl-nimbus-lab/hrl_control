// This is the header file for the coordinate conversion to cpp

#ifndef GEO_CONVERTER_H
#define GEO_CONVERTER_H


#include <math.h>

#define EARTH_R 6378137 // This is the radius of the earth in meters


class geo_converter{

	public:

		double gps_to_meters(double lat1, double lon1, double lat2, double lon2) ; // This gives the distance beteen two gps coordiantes in meters
                double lon_to_meters_signed(double lat1, double lon1, double lat2, double lon2) ; // This gives the distance beteen two longitudes in +/- w.r.t the first longitude
                double lat_to_meters_signed(double lat1, double lon1, double lat2, double lon2) ; // This gives the distance beteen two latitudes in +/- w.r.t the first latitude
		double meters_to_lat(double base_lat, double base_lon, double d ); // This gives the latitde from a base lat,lon coordinates and a particualr distance with a bearing of 0 degrees
		double meters_to_lon(double base_lat, double base_lon, double d ); // This gives the longitude from a base lat,lon coordinates and a particualr distance with a bearing of 90 degrees

};





#endif
