#ifndef _MAPMATCHING_H
#define _MAPMATCHING_H

#include <limits.h>
#include "xdb.h"

//#define DEBUG
//#define DEBUG_SELECT
//#define DEBUG_START_END
//#define	DEBUG_R
//#define HEADING
//#define DEBUG_SCORES
//#define DEBUG_FIND_MM
//#define TESTING_COUNTS
//#define TESTING_TIMES
//#define TESTING_TIMES1
//#define TESTING_TIMES2
//#define TESTING_TIMES3
//#define TESTING_TIMES4

#ifndef TRUE
	#define TRUE  1
	#define FALSE 0
#endif

#ifndef UINT
	typedef unsigned char			BYTE;		// 1B
	typedef int                 	INT;
	typedef unsigned int			UINT;		// 4B
	typedef unsigned char			BOOL;		// 1B
#endif

/**********************************************************************************************************************
* Map matched point.
**********************************************************************************************************************/
typedef struct MM_MAP_POINT
{
	UINT			id;				// id attribute (primary key) value of map matched edge from 'streets' table
	double			latitude;		// map matched latitude
	double			longitude;		// map matched longitude
	double			road_dist;		// distance from GPS point to map matched point	//TODO
} MM_MAP_PT;




/**********************************************************************************************************************
* Map matching and geocoding return values.
**********************************************************************************************************************/
typedef enum
{
	MM_OK				= 0,		// OK
	MM_NO_MEMORY		= 1,		// Not enough memory
	MM_NO_DB_RESPONSE	= 2,		// No response from DB.
	MM_ERR_UNKNOWN		= UINT_MAX,	// Unknown error
	MM_NO_LINES			= 3			// no lines in boundary found
} MM_ERRORS;

/**********************************************************************************************************************
* Map matching workspace.
**********************************************************************************************************************/
typedef struct MM_WORKSPACE
{
	xdb_t		xdb;			// DB data.
} MM_WS;

/**********************************************************************************************************************
* Map matching initialization data.
**********************************************************************************************************************/
typedef struct MM_INIT_DATA
{
	char 		*host;		// Name of the server, where DB is placed.
	char 		*db;		// Name of the DB.
	char 		*user;		// DB User name.
	char 		*passwd;	// DB User password.
	int 		port;		// DB Port number.
} MM_INIT;

/**********************************************************************************************************************
* GPS input point.
**********************************************************************************************************************/
typedef struct MM_GPS_POINT
{
	double		latitude;		// latitude of vehicle
	double		longitude;		// longitude of vehicle

	int		heading;		// heading of vehicle
	int			speed;			// speed of vehicle in km/h
} MM_GPS_PT;

/**********************************************************************************************************************
* GPS input point.
**********************************************************************************************************************/
typedef struct MM_GPS_POINT_W
{
	double		latitude;		// latitude of vehicle
	double		longitude;		// longitude of vehicle

	int		heading;		// heading of vehicle
	int			speed;			// speed of vehicle in km/h
	int			index;			
} MM_GPS_PT_W;


/**********************************************************************************************************************
* Map matching input structure.
**********************************************************************************************************************/
typedef struct MM_REQUEST_TYPE
{
	int 			gps_pts_size; 	// number of MM_GPS_PT(s)
	MM_GPS_PT* 		gps_pts; 		// pointer to first MM_GPS_PT in array
} MM_REQUEST;
/**********************************************************************************************************************
* Map matching input structure.
**********************************************************************************************************************/
typedef struct MM_REQUEST_TYPE_W
{
	int 			gps_pts_size; 	// number of MM_GPS_PT_W(s)
	MM_GPS_PT_W* 		gps_pts; 		// pointer to first MM_GPS_PT_W in array
} MM_REQUEST_W;

/**********************************************************************************************************************
* Map matching output structure.
**********************************************************************************************************************/
typedef struct MM_RESPONSE_TYPE
{
	MM_MAP_PT* 		map_pts;		// pointer to first MM_MAP_PT in array
} MM_RESPONSE;




/**********************************************************************************************************************
* Initializes library workspace. Waill have to be called once, before using any other methods of the library.
* @in init_data		Map matching initialization data.
* @out workspace	Workspace of the library, if successfull.
* @return			Return code.
**********************************************************************************************************************/
MM_ERRORS mm_init(MM_INIT *init_data, MM_WS** workspace);

/**********************************************************************************************************************
* Releases all resources used by map matching library.
* @in workspace	Workspace of the library.
**********************************************************************************************************************/
void mm_close(MM_WS* workspace);

/**********************************************************************************************************************
* Used to find all map matching points and it's related data for all GPS points stored in input array. Before first
* call of this method, mm_init must be called once by every thread. Count of map matched points will be the same as
* count of GPS input points. Output array must be allocated before calling method.
* @in ws			Workspace of the library.
* @in mm_request	Input data.
* @out mm_response	Output data.
* @return ret_code	Return code.
**********************************************************************************************************************/
MM_ERRORS mm_map_match(MM_WS * ws, MM_REQUEST* mm_request, MM_RESPONSE* mm_response);

#ifdef TESTING
	void debug_print();
#endif	//TESTING

#endif	/* MAPMATCHING */
