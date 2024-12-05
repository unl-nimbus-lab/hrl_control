
#include "custom_libs/geo_converter.hpp"

double geo_converter::gps_to_meters(double lat1, double lon1, double lat2, double lon2){
	// This gives the distance between two gps coordinates in meters

	double dLat = lat2*M_PI/180 - lat1*M_PI/180 ;
	double dLon = lon2*M_PI/180 - lon1*M_PI/180 ;

	double a = sin(dLat/2)*sin(dLat/2) + cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dLon/2)*sin(dLon/2);
	double c = 2*atan2(sqrt(a),sqrt(1-a));

	return EARTH_R*c;

}


double geo_converter::lon_to_meters_signed(double lat1, double lon1, double lat2, double lon2){
        // This gives the distance between two gps coordinates in meters

        double dLat = lat2*M_PI/180 - lat1*M_PI/180 ;
        double dLon = lon2*M_PI/180 - lon1*M_PI/180 ;

        double a = sin(dLat/2)*sin(dLat/2) + cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dLon/2)*sin(dLon/2);
        double c = 2*atan2(sqrt(a),sqrt(1-a));

        double d = EARTH_R*c;

	if(lon2 < lon1){
		d = d*-1 ;
	}

	return d ;

}

double geo_converter::lat_to_meters_signed(double lat1, double lon1, double lat2, double lon2){
        // This gives the distance between two gps coordinates in meters

        double dLat = lat2*M_PI/180 - lat1*M_PI/180 ;
        double dLon = lon2*M_PI/180 - lon1*M_PI/180 ;

        double a = sin(dLat/2)*sin(dLat/2) + cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dLon/2)*sin(dLon/2);
        double c = 2*atan2(sqrt(a),sqrt(1-a));

        double d = EARTH_R*c;

	if(lat2 < lat1){
		d = d*-1 ;
	}

	return d ;
}


double geo_converter::meters_to_lat(double base_lat, double base_lon, double d ){

	// This gives the latitude from a base lat,lon coordinates and a particular distance with a bearing of 0 degrees.
	double base_lat_rads = base_lat*M_PI/180 ;
	double base_lon_rads = base_lon*M_PI/180 ;

	double bearing = 0 * M_PI/180 ;

	double t_lat = asin(sin(base_lat_rads)*cos(d/EARTH_R) + cos(base_lat_rads)*sin(d/EARTH_R)*cos(bearing));
	double t_lon = base_lon_rads + atan2(sin(bearing)*sin(d/EARTH_R)*cos(base_lat_rads),cos(d/EARTH_R)-sin(base_lat_rads)*sin(t_lat));

	return t_lat*(180/M_PI);

}


double geo_converter::meters_to_lon(double base_lat, double base_lon, double d ){ 

        // This gives the longitude from a base lat,lon coordinates and a particular distance with a bearing of 90 degrees

        double base_lat_rads = base_lat*M_PI/180 ;
        double base_lon_rads = base_lon*M_PI/180 ;

        double bearing = 90 * M_PI/180 ;

        double t_lat = asin(sin(base_lat_rads)*cos(d/EARTH_R) + cos(base_lat_rads)*sin(d/EARTH_R)*cos(bearing));
        double t_lon = base_lon_rads + atan2(sin(bearing)*sin(d/EARTH_R)*cos(base_lat_rads),cos(d/EARTH_R)-sin(base_lat_rads)*sin(t_lat));

        return t_lon*(180/M_PI);

}
