// Map Matching v.0.2.1

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#include "mapmatching.h"

#ifdef TESTING_TIMES
#include <sys/time.h>
#endif // TESTING_TIMES

#define MAX(a, b) ((a) > (b) ? a : b)
#define MIN(a, b) ((a) < (b) ? a : b)

// new DB structure
#define NEW_STREETS_TABLE "way_splitted"
#define NEW_LINE_ATTRIBUTE "geo_line"
#define WAY_ID_ATTRIBUTE "id"
#define WAY_LENGTH_ATTRIBUTE "distance"

//#define CONS_TO_FIX_LL 	1.7		// constant to eliminate different length od lat and lon degree in UK - WORKS ONLY FOR UK!!!!

#define GPS_DISCREPANCY 0.003 // 0.003	//+ 200-300 m boundary enlargement when finding nearest lines - DON'T LOWER THIS VALUE
#define MAX_CANDIDATES 30	  // 15		// max candidates - closest to GPS point
#define MAX_DIST_FROM_GPS 200 // max accepted distance from line to GPS point
#define MAX_DB_LINES 700	  // 250		// max lines from DB in one mm cycle - IF LOWER, ROUND ABOUTS in URBAN AREA WON'T MM.
#define MAX_GPS_COUNT 250	  // max count of input gps coords in one mm cycle
#define EMIS_COEF 1 / sqrt(2 * M_PI * 20)
#define TRANS_COEF 1 / 1.4427
#define MAX_D -1
#define MINMOVE 5
#define EMIS_PARAM 30	   // 20		// its average value of difference between GPS point and real position
#define TRANS_PARAM 1.4427 // from theory = 1/(ln 2)
#define MIN_GPS_DIFF 0.002 // 0.001	// minimal distance between gps points to be accepted
#define MAX_GPS_DIFF 0.001 //			// max distance between gps points - if bigger distance, new route will be started
#define MAX_GPS_PTS_SKIP 20

//#define REV_LN2 		1.4427

typedef struct POINT_COORDS
{
	double x;
	double y;
} __attribute__((packed)) POINT_LL;

typedef struct WKBLine_t
{
	BYTE order;
	UINT wkbType;
	UINT num_points;
	POINT_LL points[1];
} __attribute__((packed)) WKBLINE;

/**********************************************************************************************************************
 * Map matched point.
 **********************************************************************************************************************/
typedef struct MM_MAP_POINT_W
{
	UINT id;		  // id attribute (primary key) value of map matched edge from 'streets' table
	double latitude;  // map matched latitude
	double longitude; // map matched longitude
	double road_dist; // distance from GPS point to map matched point	//TODO
	int geocoded;
	int line_id;
	int start_Node;
	int end_Node;
	double dist_to_start;
	double dist_start_end;
	WKBLINE *line;
	int one_way;
} MM_MAP_PT_W;
/**********************************************************************************************************************
 * Map matching output structure.
 **********************************************************************************************************************/
typedef struct MM_RESPONSE_TYPE_W
{
	MM_MAP_PT_W *map_pts; // pointer to first MM_MAP_PT in array
} MM_RESPONSE_W;
typedef struct TIME_SLOT
{
	MM_GPS_PT_W *gps_pt;				   // GPS point in this time slot.
	int line_id[MAX_CANDIDATES];		   // Array of line id for candidates
	long double scores[MAX_CANDIDATES];	   // score for HMM
	int parents[MAX_CANDIDATES];		   // Index number in previous time slot.
	double distances[MAX_CANDIDATES];	   // Dist from gps to closet point on the line.
	double dist_to_start[MAX_CANDIDATES];  // Dist form  candidate (point on lien) to start point of the line - for routing
	int start_Node[MAX_CANDIDATES];		   // ID Node on start of the line - for routing.
	int end_Node[MAX_CANDIDATES];		   // ID Node on end of the line - for routing.
	double dist_start_end[MAX_CANDIDATES]; // Dist form  start to end point of the line - for routing
#ifdef HEADING
	int heading[MAX_CANDIDATES];	 // Angle in degrees between candidate, first point on the line in the direction to start and X axis.
#endif								 // HEADING
	POINT_LL points[MAX_CANDIDATES]; // candidate - closest point on the line from gps point.
	int count;						 // Count of candidates in this time slot.
	double dist_to_next;			 // Length to next timeslot - dist from thos GPS to tne next one
	int one_way[MAX_CANDIDATES];
	int last_Node[MAX_CANDIDATES];
	WKBLINE *line[MAX_CANDIDATES];
} TIMESLOT;

typedef struct // structure for routing
{
	double x;				   // x of the node
	double y;				   // y of the node
	double Fcost;			   // A star value - uusing when calculating routing
	double Gcost;			   // A star value - uusing when calculating routing
	int neighborsCount;		   // Count of neighbors nodes
	int neighborsID[10];	   // list od neigbors ID
	int neighborsDistance[10]; // lis of distances from this node to neigbbors
	BOOL closeList;			   // A star value - uusing when calculating routing
	BOOL openList;			   // A star value - uusing when calculating routing
	int parent;				   // remembers parent node during calculation
	WKBLINE *line[10];
	int startNode[10];
} Node;

#ifdef TESTING_COUNTS
int temp_count = 0;
int pointOnSegment_count = 0;
int HaversineDistance_count = 0;
int FindPath_count = 0;
int snap_to_segment_count = 0;
int get_transition_probability_count = 0;
int create_candidates_set_count = 0;
int mm_map_match_count = 0;
#endif // TESTING_COUNTS

#ifdef TESTING_TIMES
struct timespec tsBefore, tsAfter;

const int NANO_SECONDS_IN_SEC = 1000000000;

void TimeSpecFinal(int *time, struct timespec *tstart, struct timespec *tend)
{
	//*time += abs(tend->tv_sec*1000 + tend->tv_nsec/1000000) - (tstart->tv_sec*1000 + tstart->tv_nsec/1000000);//ms
	if ((tend->tv_nsec - tstart->tv_nsec) < 0)
	{
		*time += (tend->tv_sec - tstart->tv_sec - 1) * 1000000;
		*time += (tend->tv_nsec - tstart->tv_nsec) / 1000;
		*time += 1000000;
	}
	else
	{
		*time += (tend->tv_sec - tstart->tv_sec) * 1000000;
		*time += (tend->tv_nsec - tstart->tv_nsec) / 1000;
	}
}

void TimeSpecDiff(struct timespec *ts3, struct timespec *ts1, struct timespec *ts2)
{
	ts3->tv_sec = ts1->tv_sec - ts2->tv_sec;
	ts3->tv_nsec = ts1->tv_nsec - ts2->tv_nsec;
	if (ts3->tv_nsec < 0)
	{
		ts3->tv_sec--;
		ts3->tv_nsec += NANO_SECONDS_IN_SEC;
	}
}

#ifdef TESTING_TIMES1
int db_time = 0, create_candidates_time = 0, boundary_time = 0, init_time = 0, mm_part_time = 0;
#endif // TESTING_TIMES1

#ifdef TESTING_TIMES2
int nodes_time = 0, snap_time = 0;
#endif // TESTING_TIMES2

#ifdef TESTING_TIMES3
int pointOnSegment_time = 0, distances_time = 0;
#endif // TESTING_TIMES3

#ifdef TESTING_TIMES4
int test_time = 0, test_time2 = 0, test_time3 = 0;
#endif // TESTING_TIMES4

#endif // TESTING_TIMES

static double dist2(double v_lon, double v_lat, double w_lon, double w_lat)
{
	return pow(v_lon - w_lon, 2) + pow(v_lat - w_lat, 2);
}

void pointOnSegment(double p_lon, double p_lat, double v_lon, double v_lat, double w_lon, double w_lat, double *f_lon, double *f_lat)
{
#ifdef TESTING_COUNTS
	pointOnSegment_count++;
#endif // TESTING_COUNTS

#ifdef TESTING_TIMES4
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES4

	double l2 = dist2(v_lon, v_lat, w_lon, w_lat);

#ifdef TESTING_TIMES4
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&test_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES4

	if (l2 == 0)
	{
		*f_lon = v_lon;
		*f_lat = v_lat;
		return;
	}

#ifdef TESTING_TIMES4
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES4

	double t = ((p_lon - v_lon) * (w_lon - v_lon) + (p_lat - v_lat) * (w_lat - v_lat)) / l2;

#ifdef TESTING_TIMES4
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&test_time2, &tsBefore, &tsAfter);
#endif // TESTING_TIMES4

	if (t < 0)
	{
		*f_lon = v_lon;
		*f_lat = v_lat;
		return;
	}

	if (t > 1)
	{
		*f_lon = w_lon;
		*f_lat = w_lat;
		return;
	}

#ifdef TESTING_TIMES4
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES4

	*f_lon = v_lon + t * (w_lon - v_lon);
	*f_lat = v_lat + t * (w_lat - v_lat);

#ifdef TESTING_TIMES4
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&test_time3, &tsBefore, &tsAfter);
#endif // TESTING_TIMES4

	return;
}

#define DEGREE_LENGTH 111321.0 // length of degree on equaton in meters

double vzd_by_degree(double lon1, double lat1, double lon2, double lat2)
{
	/*double a1 =   (lat1 - lat2)*DEGREE_LENGTH;
	double a2 = pow (a1,2);
	double b1 = (lon1 - lon2)*DEGREE_LENGTH;
	double b2 = cos( lat1 * (0.0175));
	double b3 = pow ((b1 * b2),2);
	double c = sqrt(a2+b3);
	double d = sqrt(pow(((lat1 - lat2)*DEGREE_LENGTH),2) + pow((((lon1 - lon2)*DEGREE_LENGTH)*(cos( lat1 * (0.0175)))),2));    //0.175 = PI/180 for rad calc*/
#ifdef TESTING_COUNTS
	HaversineDistance_count++;
#endif // TESTING_COUNTS

	return sqrt(pow(((lat1 - lat2) * DEGREE_LENGTH), 2) + pow((((lon1 - lon2) * DEGREE_LENGTH) * (cos(lat1 * (0.0174533)))), 2)); // 0.175 = PI/180 for rad calc
}

/*
static double HaversineDistance(double lon1, double lat1, double lon2, double lat2)
{
#ifdef TESTING
	HaversineDistance_count++;
#endif	//TESTING

	const double EartRadiusKm = 6371;
	double d2r = (M_PI/ 180);
	double dlon = (lon2 - lon1) * d2r;
	double dlat = (lat2 - lat1) * d2r;
	double a = pow(sin(dlat / 2.0), 2) + cos(lat1 * d2r) * cos(lat2 * d2r) * pow(sin(dlon / 2.0), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	return c * EartRadiusKm*1000;
}
*/

double FindPath(Node *nodes, int nodesCount, int StartNode1, int StartNode2, int targetNode, int *start_line_Node, int *end_line_Node)
{
#ifdef TESTING_COUNTS
	FindPath_count++;
#endif // TESTING_COUNTS

	int numberOfOpenListItems, i, j, activeFcost, neighborsNode, activeNode, neighborsActive;

	// Fill two nodes as start nodes, Gcost is already filled, Fcost is calculated, put them to open list
	nodes[StartNode1].openList = TRUE;
	nodes[StartNode1].Fcost = nodes[StartNode1].Gcost + vzd_by_degree(nodes[StartNode1].x, nodes[StartNode1].y, nodes[targetNode].x, nodes[targetNode].y);
	nodes[StartNode2].openList = TRUE;
	nodes[StartNode2].Fcost = nodes[StartNode2].Gcost + vzd_by_degree(nodes[StartNode2].x, nodes[StartNode2].y, nodes[targetNode].x, nodes[targetNode].y);
	numberOfOpenListItems = 2;
	if (StartNode1 == StartNode2)
	{
		numberOfOpenListItems = 1;
	}

	do
	{
		if (numberOfOpenListItems != 0)
		{

			j = -1; // no 0, wil be incremented in do loop

			// finding activeNode - the node with smalles Fcost, only for nodes on open and NOT on close list
			activeFcost = INT_MAX;
			for (i = 0; i < numberOfOpenListItems; i++)
			{
				do
				{
					j++;
				} while (nodes[j].openList != TRUE || nodes[j].closeList == TRUE);
				if (nodes[j].Fcost < activeFcost)
				{
					activeFcost = nodes[j].Fcost;
					activeNode = j;
				}
			}
			// we have the winner, Gcost is the distance
			if (activeNode == targetNode)
			{
				// clearing open and close Lists
				for (i = 0; i < nodesCount; i++)
				{
					nodes[i].openList = FALSE;
					nodes[i].closeList = FALSE;
				}
				*start_line_Node = nodes[activeNode].parent;
				while (StartNode2 != activeNode && StartNode1 != activeNode)
				{
					activeNode = nodes[activeNode].parent;
				} /*
				 if(StartNode2 == activeNode )
					 *end_line_Node = StartNode1; // opposite node is node on line part where route starts
				 else
					 *end_line_Node = StartNode2; //*/
				return nodes[targetNode].Gcost;
			}
			// put active node to close list
			nodes[activeNode].closeList = TRUE;
			numberOfOpenListItems--;
			// adding neghbors nodes
			for (neighborsActive = 0; neighborsActive < nodes[activeNode].neighborsCount; neighborsActive++)
			{ // fo through all neighbors
				neighborsNode = nodes[activeNode].neighborsID[neighborsActive];
				if (nodes[neighborsNode].closeList != TRUE)
				{ // only for not on close list
					if (nodes[neighborsNode].openList != TRUE)
					{ // if not on open lis - add it to open list and calculate Gcost and Fcost and set parent
						nodes[neighborsNode].openList = TRUE;
						numberOfOpenListItems++;
						nodes[neighborsNode].Gcost = nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
						nodes[neighborsNode].Fcost = nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x, nodes[neighborsNode].y, nodes[targetNode].x, nodes[targetNode].y);
						nodes[neighborsNode].parent = activeNode;
					}
					else
					{ // if already on open list - check if this new node has better G than previous
						if (nodes[neighborsNode].Gcost > nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive])
						{ // if yes callculate G, F, parent and replace
							nodes[neighborsNode].Gcost = nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
							nodes[neighborsNode].Fcost = nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x, nodes[neighborsNode].y, nodes[targetNode].x, nodes[targetNode].y);
							nodes[neighborsNode].parent = activeNode;
						}
					}
				}
			}
		}
		else
		{ // no way, buddy
			for (i = 0; i < nodesCount; i++)
			{
				nodes[i].openList = FALSE;
				nodes[i].closeList = FALSE;
			}
			return -1;
		}
	} while (1); // Do until path is found or deemed nonexistent

	return -1;
}

// points_drawn += draw_points_on_line(line, mm_Start, map_pts, first_length, def_length, draw_points, up_down);
int draw_points_on_line(WKBLINE *line, int first_point, MM_MAP_PT_W *map_pts, double draw_at, double def_length, int points, int up_down)
{
	int points_drawn = 0, i;

	double vzd_start, vzd_end, vzd;

	vzd_end = 0;
	for (i = 0; i < line->num_points - 1; i++)
	{
		vzd_start = vzd_end;
		vzd = vzd_by_degree(line->points[i].x, line->points[i].y, line->points[i + 1].x, line->points[i + 1].y);
		vzd_end += vzd;

		while (vzd_end > draw_at && points_drawn < points)
		{
			map_pts[first_point + (up_down * (points_drawn))].longitude = line->points[i].x + (line->points[i + 1].x - line->points[i].x) * ((double)(draw_at - vzd_start) / vzd);
			map_pts[first_point + (up_down * (points_drawn))].latitude = line->points[i].y + (line->points[i + 1].y - line->points[i].y) * ((double)(draw_at - vzd_start) / vzd);
			points_drawn++;
			draw_at += def_length;
		}
		if (points_drawn == points)
			break;
	}
	for (; points_drawn < points; points_drawn++)
	{
		map_pts[first_point + (up_down * points_drawn)].longitude = line->points[i].x;
		map_pts[first_point + (up_down * points_drawn)].latitude = line->points[i].y;
	}
}

int draw_points_on_lineA(WKBLINE *line, int mm_Start, int mm_End, MM_MAP_PT_W *map_pts)
{
	int drawing_part_part, points_drawn = 0, i;
	int from_start, points, first_point, last_point;
	double def_lenght = 0.0, draw_at, vzd_start, vzd_end, vzd;
	int up_down;

	points = mm_End - mm_Start - 1;
	def_lenght = fabs(map_pts[mm_Start].dist_to_start - map_pts[mm_End].dist_to_start) / (points + 1);
	if (map_pts[mm_Start].dist_to_start < map_pts[mm_End].dist_to_start)
	{
		first_point = mm_Start;
		last_point = mm_End;
		up_down = 1;
	}
	else
	{
		last_point = mm_Start;
		first_point = mm_End;
		up_down = -1;
	}

	draw_at = map_pts[first_point].dist_to_start + def_lenght;
	vzd_end = 0;
	for (i = 0; i < line->num_points - 1; i++)
	{
		vzd_start = vzd_end;
		vzd = vzd_by_degree(line->points[i].x, line->points[i].y, line->points[i + 1].x, line->points[i + 1].y);
		vzd_end += vzd;

		while (vzd_end > draw_at && points_drawn < points)
		{
			map_pts[first_point + (up_down * points_drawn + 1)].longitude = line->points[i].x + (line->points[i + 1].x - line->points[i].x) * ((double)(draw_at - vzd_start) / vzd);
			map_pts[first_point + (up_down * points_drawn + 1)].latitude = line->points[i].y + (line->points[i + 1].y - line->points[i].y) * ((double)(draw_at - vzd_start) / vzd);
			points_drawn++;
			draw_at += def_lenght;
		}
		if (points_drawn == points)
			break;
	}
	for (; points_drawn < points; points_drawn++)
	{
		map_pts[first_point + (up_down * points_drawn)].longitude = line->points[i].x;
		map_pts[first_point + (up_down * points_drawn)].latitude = line->points[i].y;
	}
}

int FindPointsOnPath(Node *nodes, int nodesCount, int mm_Start, int mm_End, int targetNode, MM_MAP_PT_W *map_pts)
{
#ifdef TESTING_COUNTS
	FindPath_count++;
#endif // TESTING_COUNTS

	int numberOfOpenListItems, i, j, activeFcost, neighborsNode, activeNode, neighborsActive, drawn_points = 0, points_in_turn;

	int points_drawn = 0, StartNode1, StartNode2, EndNode1, EndNode2, points, draw_points, up_down, mm_s;
	double way_length, draw_at, line_length, first_length, def_length, drawing_part_lenght = 0, rest_at; // rest_dist,
	WKBLINE *line;

	/*map_pts[j].longitude = tslots[j].points[c].x;
	map_pts[j].latitude = tslots[j].points[c].y;
	map_pts[j].geocoded = 1;
	map_pts[j].line_id = tslots[j].line_id[c];
	map_pts[j].start_Node = tslots[i].start_Node[c];
	map_pts[j].end_Node = tslots[i].end_Node[c];
	map_pts[j].dist_to_start=tslots[i].dist_to_start[c];
	map_pts[j].dist_start_end=tslots[i].dist_start_end[c];*/

	if (map_pts[mm_Start].line_id == map_pts[mm_End].line_id)
	{
		line = map_pts[mm_Start].line;
		return draw_points_on_lineA(line, mm_Start, mm_End, map_pts);
		// return 0;
	}

	StartNode1 = map_pts[mm_Start].start_Node;
	StartNode2 = map_pts[mm_Start].end_Node;
	EndNode1 = map_pts[mm_End].start_Node;
	EndNode2 = map_pts[mm_End].end_Node;
	// Fill two nodes as start nodes, Gcost is already filled, Fcost is calculated, put them to open list
	nodes[StartNode1].openList = TRUE;
	nodes[StartNode1].Fcost = nodes[StartNode1].Gcost + vzd_by_degree(nodes[StartNode1].x, nodes[StartNode1].y, nodes[targetNode].x, nodes[targetNode].y);
	nodes[StartNode2].openList = TRUE;
	nodes[StartNode2].Fcost = nodes[StartNode2].Gcost + vzd_by_degree(nodes[StartNode2].x, nodes[StartNode2].y, nodes[targetNode].x, nodes[targetNode].y);
	numberOfOpenListItems = 2;
	if (StartNode1 == StartNode2)
	{
		numberOfOpenListItems = 1;
	}

	do
	{
		if (numberOfOpenListItems != 0)
		{

			j = -1; // no 0, wil be incremented in do loop

			// finding activeNode - the node with smalles Fcost, only for nodes on open and NOT on close list
			activeFcost = INT_MAX;
			for (i = 0; i < numberOfOpenListItems; i++)
			{
				do
				{
					j++;
				} while (nodes[j].openList != TRUE || nodes[j].closeList == TRUE);
				if (nodes[j].Fcost < activeFcost)
				{
					activeFcost = nodes[j].Fcost;
					activeNode = j;
				}
			}
			// we have the winner, Gcost is the distance
			if (activeNode == targetNode)
			{
				// clearing open and close Lists
				for (i = 0; i < nodesCount; i++)
				{
					nodes[i].openList = FALSE;
					nodes[i].closeList = FALSE;
				}

				/////////////////////////
				way_length = nodes[targetNode].Gcost;
				points = mm_End - mm_Start - 1;
				def_length = way_length / (points + 1);
				way_length -= def_length;
				mm_s = mm_End - 1;

				for (i = 0; i < nodes[nodes[activeNode].parent].neighborsCount; i++)
				{
					if (nodes[nodes[activeNode].parent].neighborsID[i] == activeNode)
					{
						line_length = map_pts[mm_End].dist_start_end;

						if (nodes[activeNode].parent == EndNode1)
						{																//  start node
																						// OK
							draw_points = (map_pts[mm_End].dist_to_start / def_length); //-1
							draw_at = map_pts[mm_End].dist_to_start - (draw_points * def_length);
							up_down = 1;
							mm_s = mm_s - draw_points + 1;
							line = map_pts[mm_End].line; // nodes[nodes[activeNode].parent].line[i];
							points_drawn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
							// rest_dist = way_length - (def_length*draw_points);
							rest_at = draw_at;
							mm_s = mm_s - 1;
						}
						else
						{
							// OK
							draw_points = ((line_length - map_pts[mm_End].dist_to_start) / def_length);
							// draw_at =  line_length - map_pts[mm_End].dist_to_start + def_length;
							draw_at = map_pts[mm_End].dist_to_start + def_length;
							// nodes[nodes[activeNode].parent].neighborsDistance[i] - (def_length*(draw_points+1));
							up_down = -1;

							line = map_pts[mm_End].line; // nodes[nodes[activeNode].parent].line[i];

							points_drawn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
							mm_s = mm_s - points_drawn;
							// rest_dist = way_length - (def_length*draw_points);
							rest_at = (double)line_length - (def_length * (points_drawn - 1)) - draw_at;
						}
						break;
					}
				}
				activeNode = nodes[activeNode].parent;
				while ((StartNode2 != activeNode && StartNode1 != activeNode) && points_drawn < points)
				{

					for (i = 0; i < nodes[nodes[activeNode].parent].neighborsCount; i++)
					{
						if (nodes[nodes[activeNode].parent].neighborsID[i] == activeNode)
						{
							line_length = nodes[nodes[activeNode].parent].neighborsDistance[i];
							break;
						}
					}

					if ((int)((line_length + rest_at) / def_length) > (points - points_drawn))
						draw_points = points - points_drawn;
					else
						draw_points = (line_length + rest_at) / def_length;
					line = nodes[nodes[activeNode].parent].line[i];
					if (nodes[nodes[activeNode].parent].startNode[i] != 1)
					{
						up_down = 1; /////star
									 // dddddddddd
						mm_s = mm_s - draw_points + 1;
						draw_at = def_length - rest_at;
						points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
						mm_s = mm_s - 1;
					}
					else
					{
						up_down = -1;
						// ssss
						// mm_s = mm_s - draw_points;
						draw_at = line_length - (draw_points * def_length) + rest_at;
						points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
						mm_s = mm_s - points_in_turn;
					}
					// rest_dist = rest_dist - (def_length*draw_points);
					rest_at = line_length + rest_at - (def_length * points_in_turn);
					points_drawn += points_in_turn;
					activeNode = nodes[activeNode].parent;
				}

				if (points_drawn < points)
				{
					line_length = map_pts[mm_Start].dist_start_end;

					if (activeNode != StartNode1)
					{
						draw_points = points - points_drawn;
						draw_at = map_pts[mm_Start].dist_start_end - (def_length * (draw_points - 1)) - def_length + rest_at;
						up_down = 1;
						mm_s = mm_s - draw_points + 1;
						line = map_pts[mm_Start].line; // nodes[nodes[activeNode].parent].line[i];

						points_drawn += draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
					}
					else
					{
						draw_points = points - points_drawn;
						draw_at = def_length - rest_at;
						up_down = -1;
						// mm_s = mm_s - draw_points ;
						line = map_pts[mm_Start].line; // nodes[nodes[activeNode].parent].line[i];

						points_drawn += draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
					}
				}
				return points_drawn;
				//////////////////////////////////
			}
			// put active node to close list
			nodes[activeNode].closeList = TRUE;
			numberOfOpenListItems--;
			// adding neghbors nodes
			for (neighborsActive = 0; neighborsActive < nodes[activeNode].neighborsCount; neighborsActive++)
			{ // fo through all neighbors
				neighborsNode = nodes[activeNode].neighborsID[neighborsActive];
				if (nodes[neighborsNode].closeList != TRUE)
				{ // only for not on close list
					if (nodes[neighborsNode].openList != TRUE)
					{ // if not on open lis - add it to open list and calculate Gcost and Fcost and set parent
						nodes[neighborsNode].openList = TRUE;
						numberOfOpenListItems++;
						nodes[neighborsNode].Gcost = nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
						nodes[neighborsNode].Fcost = nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x, nodes[neighborsNode].y, nodes[targetNode].x, nodes[targetNode].y);
						nodes[neighborsNode].parent = activeNode;
					}
					else
					{ // if already on open list - check if this new node has better G than previous
						if (nodes[neighborsNode].Gcost > nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive])
						{ // if yes callculate G, F, parent and replace
							nodes[neighborsNode].Gcost = nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
							nodes[neighborsNode].Fcost = nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x, nodes[neighborsNode].y, nodes[targetNode].x, nodes[targetNode].y);
							nodes[neighborsNode].parent = activeNode;
						}
					}
				}
			}
		}
		else
		{ // no way, buddy
			for (i = 0; i < nodesCount; i++)
			{
				nodes[i].openList = FALSE;
				nodes[i].closeList = FALSE;
			}
			return -1;
		}
	} while (1); // Do until path is found or deemed nonexistent

	return -1;
}

#ifdef HEADING
static int snap_to_segment(WKBLINE *line, double gps_lon, double gps_lat, double *lon_snap, double *lat_snap, double *shortest_distance, double *dist_to_start, double degree_coef, int *heading)
#else
static int snap_to_segment(WKBLINE *line, double gps_lon, double gps_lat, double *lon_snap, double *lat_snap, double *shortest_distance, double *dist_to_start, double degree_coef)
#endif // HEADING
{
#ifdef TESTING_COUNTS
	snap_to_segment_count++;
#endif // TESTING_COUNTS

	double distance = 1000.0; ///< current distance between GPS location and candidate solution
	// double distance_to_mm = 0.0; /// distance from start to best candidate
	double d_mm = 0.0; /// distance from start to current part

	double p_lon, p_lat, /*lon_snap_degraded,*/ v_lon, v_lat, w_lon, w_lat, res_lon, res_lat;
	int i = 0;

	*shortest_distance = distance; ///< candidate distance

	if (line->num_points < 2)
	{
		return -1;
	}

	// For each pair of points
	for (i = 0; i < line->num_points - 1; i++)
	{
		p_lon = gps_lon * degree_coef;
		p_lat = gps_lat;
		v_lon = line->points[i].x * degree_coef;
		v_lat = line->points[i].y;
		w_lon = line->points[i + 1].x * degree_coef;
		w_lat = line->points[i + 1].y;
#ifdef TESTING_TIMES3
		clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES3
		pointOnSegment(p_lon, p_lat, v_lon, v_lat, w_lon, w_lat, &res_lon, &res_lat);
#ifdef TESTING_TIMES3
		clock_gettime(CLOCK_REALTIME, &tsAfter);
		TimeSpecFinal(&pointOnSegment_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES3
		res_lon = res_lon / degree_coef;

#ifdef TESTING_TIMES3
		clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES3
		distance = vzd_by_degree(gps_lon, gps_lat, res_lon, res_lat);
		if (distance < *shortest_distance)
		{
			// Found a better solution, use this one
			*lon_snap = res_lon;
			*lat_snap = res_lat;
			*shortest_distance = distance;
#ifdef HEADING
			*heading = 90 - (atan2(*lat_snap - v_lat, (*lon_snap * degree_coef) - v_lon) * 180 / M_PI); // heading from box has 0 no north and clockwise
																										// if (*lat_snap - v_lat < 0) *heading += 360;
																										// printf ("%f,%f %f,%f %i\n",v_lat,v_lon/degree_coef, *lat_snap, lon_snap_degraded/degree_coef,*heading );
#endif																									// HEADING
			*dist_to_start /*distance_to_mm*/ = d_mm + vzd_by_degree(line->points[i].x, line->points[i].y, res_lon, res_lat);
		}
		d_mm += vzd_by_degree(line->points[i].x, line->points[i].y, line->points[i + 1].x, line->points[i + 1].y);
#ifdef TESTING_TIMES3
		clock_gettime(CLOCK_REALTIME, &tsAfter);
		TimeSpecFinal(&distances_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES3
	}

	// *shortest_distance = vzd_by_degree(*lon_snap,*lat_snap,gps_lon,gps_lat);

	//*dist_to_start = distance_to_mm;

	return 1;
}

//Hidden Markov Model emissions
double get_emission_probability(double distance, double emis_param)
{
	// return 1 * (EMIS_COEF * exp (-0.5 * pow(distance / emis_param, 2)));
	/*double k=1;
	if(distance>emis_param)
		k=emis_param/distance;
	return k;*/
	return (1 / sqrt(2 * M_PI * emis_param)) * exp(-0.5 * pow(distance / emis_param, 2));
	// return (1 / sqrt(2 * M_PI * EMIS_PARAM)) * exp (-0.5 * pow(distance / EMIS_PARAM, 2));
}

double get_transition_probability(double dist_excess /*, double trans_param*/)
// double get_transition_probability(double dist,double route_dist, double trans_param)
{
#ifdef TESTING_COUNTS
	get_transition_probability_count++;
#endif // TESTING_COUNTS

	/*	double k=1;
		if(route_dist<1)route_dist = 1;
		if(dist<1)dist = 1;
		if(dist<route_dist){
			k=(dist+((route_dist-dist)/4))/route_dist;
		}else{
			k=route_dist/dist;
		}
		return k;*/
	// return 1 * (TRANS_COEF * exp (-(dist_excess) / trans_param ));
	return (1 / TRANS_PARAM) * exp(-(dist_excess) / TRANS_PARAM);
	// return (1 / trans_param) * exp (-(dist_excess) / trans_param );
}

/**********************************************************************************************************************
 * Fill up start and end time slots. Some GPS points can be abandoned, so than count of time slots will be less than
 * gps_count depending on number of abandoned points.
 * @in gps_pts			Array of GPS points.
 * @in gps_count			Count of GPS points.
 * @out tslot			Array of time slots.
 * @return				Total distance between all GPS points (degrees).
 **********************************************************************************************************************/
void init_time_slots(MM_GPS_PT_W *gps_pts, int gps_count, TIMESLOT *tslots)
{
	int i;

	for (i = 0; i < gps_count - 1; i++) // compute all distances
	{
		tslots[i].gps_pt = &gps_pts[i];

	/*	if ((gps_pts[i].latitude == gps_pts[i + 1].latitude) && (gps_pts[i].longitude == gps_pts[i + 1].longitude))
		{
			tslots[i].dist_to_next = 0;
		}
		else
		{
			tslots[i].dist_to_next = vzd_by_degree(gps_pts[i].longitude, gps_pts[i].latitude, gps_pts[i + 1].longitude, gps_pts[i + 1].latitude);
		}
	*/

	}

	tslots[gps_count - 1].gps_pt = &gps_pts[gps_count - 1]; // last time slot

	return;
}

/**********************************************************************************************************************
 * Create set of possible map matched points for all GPS points (one gps point is covered by one timeslot), and prepare
 * data for routing.
 * @in/out  tslot	Array of time slots, for which will be candidates searched.
 * @in count			Count of time slots.
 * @return			Return code.
 **********************************************************************************************************************/
MM_ERRORS create_candidates_set(TIMESLOT *tslots, int count, Node *nodes, int *nodes_sum, xdb_result_t *xdb_result)
{
#ifdef TESTING_COUNTS
	create_candidates_set_count++;
#endif // TESTING_COUNTS

#ifdef HEADING
	int heading;
#endif // HEADING
	int i, j;
	double mm_long, mm_lat, distance, dist_to_start, degree_coef;
	double start_lon, start_lat, end_lon, end_lat;
	int active_nodes, nodes_count;

	int db_index, way_id, way_length, way_length_rev, worst_dist, worst_id;
	WKBLINE *line;
	int StartNode, EndNode;

	nodes_count = 0;

	while (xdb_get_next_row(xdb_result))
	{
		db_index = 0;

		if (xdb_get_field_blob(xdb_result, db_index++, (void **)&line) == 0) // get one line from result
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_int(xdb_result, db_index++, &way_id) == 0)
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_int(xdb_result, db_index++, &way_length) == 0)
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_double(xdb_result, db_index++, &start_lon) == 0)
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_double(xdb_result, db_index++, &start_lat) == 0)
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_double(xdb_result, db_index++, &end_lon) == 0)
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_double(xdb_result, db_index++, &end_lat) == 0)
			return MM_ERR_UNKNOWN;
		if (xdb_get_field_int(xdb_result, db_index++, &way_length_rev) == 0)
			return MM_ERR_UNKNOWN;
			// if xdb_get_field_string will be used : method returns MM_ERR_UNKNOWN if value is NULL!!

#ifdef DEBUG
		if (way_id == 293232)
			printf("found\n");
#endif // DEBUG

		if (nodes_count == 0)
		{
			degree_coef = cos(start_lat * (0.0174533));
		}

#ifdef TESTING_TIMES2
		clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES2

		// fill nodes from lines data for routing- runs only once for all GPS points and all routing calculations
		active_nodes = nodes_count;
		for (i = 0; i < active_nodes; i++)
		{
			if (nodes[i].x == start_lon && nodes[i].y == start_lat)
			{
				for (j = 0; j < nodes_count; j++)
				{
					if (nodes[j].x == end_lon && nodes[j].y == end_lat)
					{
						if (way_length >= 0)
						{
							nodes[i].neighborsID[nodes[i].neighborsCount] = j;
							nodes[i].neighborsDistance[nodes[i].neighborsCount] = way_length;
							nodes[i].line[nodes[i].neighborsCount] = line;
							nodes[i].startNode[nodes[i].neighborsCount] = 1;
							nodes[i].neighborsCount++;
						}
						if (way_length_rev >= 0)
						{
							nodes[j].neighborsID[nodes[j].neighborsCount] = i;
							nodes[j].neighborsDistance[nodes[j].neighborsCount] = way_length_rev;
							nodes[j].line[nodes[j].neighborsCount] = line;
							nodes[j].startNode[nodes[j].neighborsCount] = 0;
							nodes[j].neighborsCount++;
						}
						StartNode = i;
						EndNode = j;
						break;
					}
				}
				if (j == nodes_count)
				{
					if (way_length >= 0)
					{
						nodes[i].neighborsID[nodes[i].neighborsCount] = nodes_count;
						nodes[i].neighborsDistance[nodes[i].neighborsCount] = way_length;
						nodes[i].line[nodes[i].neighborsCount] = line;
						nodes[i].startNode[nodes[i].neighborsCount] = 1;
						nodes[i].neighborsCount++;
					}
					nodes[nodes_count].x = end_lon;
					nodes[nodes_count].y = end_lat;
					if (way_length_rev >= 0)
					{
						nodes[nodes_count].neighborsCount = 1;
						nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount - 1] = i;
						nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount - 1] = way_length_rev;
						nodes[nodes_count].line[nodes[nodes_count].neighborsCount - 1] = line;
						nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount] = 0;
					}
					else
					{
						nodes[nodes_count].neighborsCount = 0;
					}
					StartNode = i;
					EndNode = nodes_count;

					nodes_count++;
				}
				break;
			}
		}
		if (i == active_nodes)
		{
			active_nodes = nodes_count;
			for (j = 0; j < active_nodes; j++)
			{
				if (nodes[j].x == end_lon && nodes[j].y == end_lat)
				{
					if (way_length >= 0)
					{
						nodes[nodes_count].x = start_lon;
						nodes[nodes_count].y = start_lat;
						nodes[nodes_count].neighborsCount = 1;
						nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount - 1] = j;
						nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount - 1] = way_length;
						nodes[nodes_count].line[nodes[nodes_count].neighborsCount - 1] = line;
						nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount] = 1;
					}
					else
					{
						nodes[nodes_count].neighborsCount = 0;
					}
					if (way_length_rev >= 0)
					{
						nodes[j].neighborsID[nodes[j].neighborsCount] = nodes_count;
						nodes[j].neighborsDistance[nodes[j].neighborsCount] = way_length_rev;
						nodes[j].line[nodes[j].neighborsCount] = line;
						nodes[j].startNode[nodes[j].neighborsCount] = 0;
						nodes[j].neighborsCount++;
					}
					StartNode = nodes_count;
					EndNode = j;
					nodes_count++;
					break;
				}
			}
		}
		if (i == active_nodes && j == active_nodes)
		{
			nodes[nodes_count].x = start_lon;
			nodes[nodes_count].y = start_lat;
			if (way_length >= 0)
			{
				nodes[nodes_count].neighborsCount = 1;
				nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount - 1] = nodes_count + 1;
				nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount - 1] = way_length;
				nodes[nodes_count].line[nodes[nodes_count].neighborsCount - 1] = line;
				nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount] = 1;
			}
			else
			{
				nodes[nodes_count].neighborsCount = 0;
			}
			StartNode = nodes_count;
			nodes_count++;
			nodes[nodes_count].x = end_lon;
			nodes[nodes_count].y = end_lat;
			if (way_length_rev >= 0)
			{
				nodes[nodes_count].neighborsCount = 1;
				nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount - 1] = nodes_count - 1;
				nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount - 1] = way_length_rev;
				nodes[nodes_count].line[nodes[nodes_count].neighborsCount - 1] = line;
				nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount] = 0;
			}
			else
			{
				nodes[nodes_count].neighborsCount = 0;
			}
			EndNode = nodes_count;
			nodes_count++;
		}

#ifdef TESTING_TIMES2
		clock_gettime(CLOCK_REALTIME, &tsAfter);
		TimeSpecFinal(&nodes_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES2

#ifdef TESTING_TIMES2
		clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES2

		// fill all time slots
		for (i = 0; i < count; i++)
		{
			// find the closest point on this line for all GPS points
			// TODO add condition not to do this part for not closed points - we have boundary box for each line
			// TODO handle error return from snap function
#ifdef DEBUG_SCORES
			// if (tslots[i].gps_pt->latitude == 51.45951033 || tslots[i].gps_pt->latitude == 51.45948887)
			// if (mm_lat == 51.459663 || mm_long == -2.590200)
			// if (mm_lat <= 51.459664 && mm_lat >= 51.459662 && mm_long >= -2.590198 && mm_long <= -2.590202/* && (tslots[i].gps_pt->latitude == 51.45951033 || tslots[i].gps_pt->latitude == 51.45948887)*/)
			//	printf("kuk\n");
#endif // DEBUG_SCORES

#ifdef HEADING
			snap_to_segment(line, tslots[i].gps_pt->longitude, tslots[i].gps_pt->latitude, &mm_long, &mm_lat, &distance, &dist_to_start, degree_coef, &heading);
#else
			snap_to_segment(line, tslots[i].gps_pt->longitude, tslots[i].gps_pt->latitude, &mm_long, &mm_lat, &distance, &dist_to_start, degree_coef);
#endif // HEADING
#ifdef DEBUG_SCORES
			// if (way_id == 2597539 && i==47)
			//	printf("dist_to_start:%f\n",dist_to_start);
#endif										  // DEBUG_SCORES
			if (distance < MAX_DIST_FROM_GPS) // is point in range?
			{
				if (tslots[i].count < MAX_CANDIDATES) // ? already found max candidates
				{
					tslots[i].dist_start_end[tslots[i].count] = way_length;
					tslots[i].points[tslots[i].count].x = mm_long;
					tslots[i].points[tslots[i].count].y = mm_lat;
					tslots[i].distances[tslots[i].count] = distance;
					tslots[i].dist_to_start[tslots[i].count] = dist_to_start;
					tslots[i].start_Node[tslots[i].count] = StartNode;
					tslots[i].end_Node[tslots[i].count] = EndNode;
					tslots[i].line_id[tslots[i].count] = way_id;
					tslots[i].line[tslots[i].count] = line;
#ifdef HEADING
					tslots[i].heading[tslots[i].count] = heading;
#endif // HEADING
					if (way_length >= 0 && way_length_rev >= 0)
						tslots[i].one_way[tslots[i].count] = 0;
					else if (way_length < 0)
						tslots[i].one_way[tslots[i].count] = -1;
					else
						tslots[i].one_way[tslots[i].count] = 1;
					tslots[i].count++;
				}
				else
				{
					worst_dist = 0;

					for (j = 0; j < tslots[i].count; j++) // max candidates found, ? is one worse than the new one
					{
						if (tslots[i].distances[j] > worst_dist)
						{
							worst_dist = tslots[i].distances[j];
							worst_id = j;
						}
					}

					tslots[i].dist_start_end[worst_id] = way_length;
					tslots[i].points[worst_id].x = mm_long;
					tslots[i].points[worst_id].y = mm_lat;
					tslots[i].distances[worst_id] = distance;
					tslots[i].dist_to_start[worst_id] = dist_to_start;
					tslots[i].start_Node[worst_id] = StartNode;
					tslots[i].end_Node[worst_id] = EndNode;
					tslots[i].line_id[worst_id] = way_id;
					tslots[i].line[worst_id] = line;
					if (way_length >= 0 && way_length_rev >= 0)
						tslots[i].one_way[worst_id] = 0;
					else if (way_length < 0)
						tslots[i].one_way[worst_id] = -1;
					else
						tslots[i].one_way[worst_id] = 1;
				}
			}

#ifdef TESTING_TIMES2
			clock_gettime(CLOCK_REALTIME, &tsAfter);
			TimeSpecFinal(&snap_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES2

			*nodes_sum = nodes_count;
			return MM_OK;
		}
	}
}

void mm_close(MM_WS *ws)
{
	xdb_close(&ws->xdb);
	free(ws);
}

MM_ERRORS mm_init(MM_INIT *init_data, MM_WS **workspace)
{
	xdb_t *xdb;

	if (NULL == (*workspace = (MM_WS *)malloc(sizeof(MM_WS)))) // memory is freed in mm_close method
	{
		return MM_NO_MEMORY;
	}

	xdb = &((*workspace)->xdb);

	// Init DB.
	xdb_init2(xdb, init_data->host, init_data->user, init_data->passwd, init_data->db, init_data->port);

	return MM_OK;
}

void map_maptch_part(TIMESLOT *tslots, MM_MAP_PT_W *map_pts, Node *nodes, int gpscount, int nodes_count, int start) // ORIGINAL
{
	int i, l, r, c, i_break, j, start_line_Node = -1, end_line_Node = -2;
	long double tmp_score, max_r_score, score[MAX_CANDIDATES];
	double dist_excess, prob_r_l, route_dist;
	// double emis_param, trans_param;
	// int k;
	double dist_from_point = DBL_MAX;
#ifdef DEBUG
	int iii, debug_count;
	BOOL startDebug = FALSE;
	double search_lat = 50.86795986;
#endif // DEBUG
	   // emis_param = 20; // emission parameter - its average value of difference between GPS point and real position
	   // trans_param = REV_LN2; // from theory

#ifdef TESTING_COUNTS
	temp_count++;
#endif // TESTING_COUNTS

#ifdef DEBUG_SCORES
	if (((tslots[start].gps_pt->latitude < (search_lat + 0.00001) && tslots[start].gps_pt->latitude > (search_lat - 0.00001))))
		printf("gps found, i:%i\n", start);
#endif // DEBUG_SCORES

	if (gpscount == 1) // only one GPS point, find the closest point on lines
	{
		for (i = 0; i < tslots[start].count; i++)
		{
			if (dist_from_point > tslots[start].distances[i])
			{
				map_pts[start].longitude = tslots[start].points[i].x;
				map_pts[start].latitude = tslots[start].points[i].y;
				map_pts[start].geocoded = 0;
				dist_from_point = tslots[start].distances[i];
			}
		}
		return;
	}

	i_break = start;
	for (i = start; i < gpscount - 1 + start; i++) // i = timeslot from, i+1 = timeslot to
	{

#ifdef DEBUG_SCORES

		if (((tslots[i].gps_pt->latitude < (search_lat + 0.00001) && tslots[i].gps_pt->latitude > (search_lat - 0.00001))) ||
			((tslots[i + 1].gps_pt->latitude < (search_lat + 0.00001) && tslots[i + 1].gps_pt->latitude > (search_lat - 0.00001))))
		{

			if (tslots[i].gps_pt->latitude < (search_lat + 0.00001) && tslots[i].gps_pt->latitude > (search_lat - 0.00001))
			{
				printf("gps found, i:%i\n", i);
			}
			else
			{
				printf("gps found, i:%i\n", i + 1);
			}
			debug_count = 30;
			startDebug = TRUE;
		}
		if (startDebug == TRUE)
		{
			printf("I%i:GPS: %f,%f \n", i, tslots[i].gps_pt->latitude, tslots[i].gps_pt->longitude);

			for (iii = 0; iii < tslots[i].count; iii++)
			{
				printf("I%i:Line[%i] ID: %i Candidate: %f,%f\n", i, iii, tslots[i].line_id[iii], tslots[i].points[iii].y, tslots[i].points[iii].x);
			}

			printf("I%i:GPS: %f,%f \n", i + 1, tslots[i + 1].gps_pt->latitude, tslots[i + 1].gps_pt->longitude);

			for (iii = 0; iii < tslots[i + 1].count; iii++)
			{
				printf("I%i:Line[%i] ID: %i Candidate: %f,%f\n", i + 1, iii, tslots[i + 1].line_id[iii], tslots[i + 1].points[iii].y, tslots[i + 1].points[iii].x);
			}

			debug_count--;

			if (debug_count == 0)
				startDebug = FALSE;
		}
#endif // DEBUG_SCORES

		c = -1; // init / no parent

		for (l = 0; l < tslots[i + 1].count; l++) // l = actual candidate in timeslot[i+1]
		{
			max_r_score = 0;

			score[l] = get_emission_probability(tslots[i + 1].distances[l], EMIS_PARAM); // count emisson probability
#ifdef HEADING
			/*double xxx = fmod(abs(tslots[i+1].heading[l] - tslots[i+1].gps_pt->heading),90.0);
			xxx = 1-(xxx/100.0);
			xxx = pow(xxx,3);*/
#ifdef DEBUG_SCORES
			if (startDebug == TRUE)
			{
				printf("Before heading : score[l]:%LE. %f %i %i \n", score[l], pow((1 - (fmod(abs(tslots[i + 1].heading[l] - tslots[i + 1].gps_pt->heading), 90.0) / 100.0)), 12), tslots[i + 1].heading[l], tslots[i + 1].gps_pt->heading);
			}
#endif // DEBUG
			score[l] *= pow((1 - (fmod(abs(tslots[i + 1].heading[l] - tslots[i + 1].gps_pt->heading), 90.0) / 100.0)), 12);
#endif // HEADING
#ifdef DEBUG_SCORES
			if (startDebug == TRUE)
			{
				printf("score[l]:%LE.\n", score[l]);
			}
#endif // DEBUG

			for (r = 0; r < tslots[i].count; r++) // r = candidate in timeslot[i]
			{
				/*
				#ifdef	DEBUG
					if(i==0 && l == 13 && r==0)
					{
						printf("ILR\n");
					}
				#endif	//DEBUG
				*/
				if (i == i_break /*start*/ && l == 0)
				{
					tslots[i].scores[r] = get_emission_probability(tslots[i].distances[r], EMIS_PARAM); // count emisson probability fot tslot [0], only for the first run
#ifdef HEADING
					tslots[i].scores[r] *= pow((1 - (fmod(abs(tslots[i].heading[r] - tslots[i].gps_pt->heading), 90.0) / 100.0)), 12);

#endif // HEADING
				}

				// calculate road distance from r to l candidate
				if (tslots[i].line_id[r] == tslots[i + 1].line_id[l]) // both on same line
				{
					if (tslots[i].one_way[r] == 0 || abs(tslots[i].dist_to_start[r] - tslots[i + 1].dist_to_start[l]) < MINMOVE)
					{
						route_dist = abs(tslots[i].dist_to_start[r] - tslots[i + 1].dist_to_start[l]);
					}

					else if (tslots[i].one_way[r] == 1)
					{
						if (tslots[i].dist_to_start[r] <= tslots[i + 1].dist_to_start[l])
							route_dist = abs(tslots[i].dist_to_start[r] - tslots[i + 1].dist_to_start[l]);
						else
							route_dist = MAX_D;
					}
					else
					{
						if (tslots[i].dist_to_start[r] >= tslots[i + 1].dist_to_start[l])
							route_dist = abs(tslots[i].dist_to_start[r] - tslots[i + 1].dist_to_start[l]);
						else
							route_dist = MAX_D;
					}
					if (tslots[i].dist_to_start[r] <= tslots[i + 1].dist_to_start[l])
					{
						// tslots[i+1].last_Node[l] = tslots[i+1].end_Node[l];
						end_line_Node = tslots[i].end_Node[r];
						start_line_Node = tslots[i + 1].start_Node[l];
					}
					else
					{
						// tslots[i+1].last_Node[l] = tslots[i+1].start_Node[l];
						end_line_Node = tslots[i].start_Node[r];
						start_line_Node = tslots[i + 1].end_Node[l];
					}
				}
				else
				{
					if (tslots[i].start_Node[r] == tslots[i + 1].start_Node[l]) // one common node / start-start
					{
						if (tslots[i].one_way[r] != 1 && tslots[i + 1].one_way[l] != -1)
							route_dist = tslots[i].dist_to_start[r] + tslots[i + 1].dist_to_start[l];
						else
							route_dist = MAX_D;
						end_line_Node = tslots[i].start_Node[r];
						start_line_Node = tslots[i + 1].start_Node[l];
					}
					else
					{
						if (tslots[i].start_Node[r] == tslots[i + 1].end_Node[l]) // one common node / start-end
						{
							if (tslots[i].one_way[r] != 1 && tslots[i + 1].one_way[l] != 1)
								route_dist = tslots[i].dist_to_start[r] + tslots[i + 1].dist_start_end[l] - tslots[i + 1].dist_to_start[l];
							else
								route_dist = MAX_D;
							end_line_Node = tslots[i].start_Node[r];
							start_line_Node = end_line_Node; // tslots[i+i].end_Node[l];
						}
						else
						{
							if (tslots[i].end_Node[r] == tslots[i + 1].start_Node[l]) // one common node / end-start
							{
								if (tslots[i].one_way[r] != -1 && tslots[i + 1].one_way[l] != -1)
									route_dist = tslots[i].dist_start_end[r] - tslots[i].dist_to_start[r] + tslots[i + 1].dist_to_start[l];
								else
									route_dist = MAX_D;
								end_line_Node = tslots[i].end_Node[r];
								start_line_Node = tslots[i + 1].start_Node[l];
							}
							else
							{
								if (tslots[i].end_Node[r] == tslots[i + 1].end_Node[l]) // one common node / end-end
								{
									if (tslots[i].one_way[r] != -1 && tslots[i + 1].one_way[l] != 1)
										route_dist = tslots[i].dist_start_end[r] - tslots[i].dist_to_start[r] + tslots[i + 1].dist_start_end[l] - tslots[i + 1].dist_to_start[l];
									else
										route_dist = MAX_D;
									end_line_Node = tslots[i].end_Node[r];
									start_line_Node = end_line_Node; // tslots[i+i].end_Node[l];
								}
								else
								{ // have to use routing

									nodes[tslots[i].start_Node[r]].Gcost = tslots[i].dist_to_start[r];							   // fill distance from candidate to the first start node
									nodes[tslots[i].end_Node[r]].Gcost = tslots[i].dist_start_end[r] - tslots[i].dist_to_start[r]; // fill distance from candidate to the second start node
									if (tslots[i + 1].one_way[l] != 1)
									{
										nodes[tslots[i + 1].end_Node[l]].neighborsID[nodes[tslots[i + 1].end_Node[l]].neighborsCount] = nodes_count; // add new neighbors to node which is on line where target
										nodes[tslots[i + 1].end_Node[l]].neighborsDistance[nodes[tslots[i + 1].end_Node[l]].neighborsCount] = tslots[i + 1].dist_start_end[l] - tslots[i + 1].dist_to_start[l];
										nodes[tslots[i + 1].end_Node[l]].neighborsCount++;
									}
									if (tslots[i + 1].one_way[l] != -1)
									{
										nodes[tslots[i + 1].start_Node[l]].neighborsID[nodes[tslots[i + 1].start_Node[l]].neighborsCount] = nodes_count; // add new neighbors to node which is on line where target
										nodes[tslots[i + 1].start_Node[l]].neighborsDistance[nodes[tslots[i + 1].start_Node[l]].neighborsCount] = tslots[i + 1].dist_to_start[l];
										nodes[tslots[i + 1].start_Node[l]].neighborsCount++;
									}
									route_dist = FindPath(nodes, nodes_count + 1,
														  (tslots[i].one_way[r] == 1) ? tslots[i].end_Node[r] : tslots[i].start_Node[r],
														  (tslots[i].one_way[r] == -1) ? tslots[i].start_Node[r] : tslots[i].end_Node[r],
														  nodes_count, &start_line_Node, &end_line_Node);
									/*if(tslots[i+1].end_Node[l]==end_line_Node)
										end_line_Node=tslots[i+1].start_Node[l];
									else
										end_line_Node=tslots[i+1].end_Node[l];*/

									// start_line_Node = -1;
									// end_line_Node = -2;

									if (tslots[i + 1].one_way[l] != 1)
									{
										nodes[tslots[i + 1].end_Node[l]].neighborsCount--; // delete adde neighbors for next routing
									}
									if (tslots[i + 1].one_way[l] != -1)
									{
										nodes[tslots[i + 1].start_Node[l]].neighborsCount--;
									}
								}
							}
						}
					}
				}
				if (route_dist < 0)
				{
					prob_r_l = 0; // 1E-100;	//no wey
				}
				else
				{
					// double prob_ofset = get_emission_probability(abs(tslots[i].distances[r]-(tslots[i+1].distances[l])),5);
					// prob_r_l *= prob_ofset;
					dist_excess = abs(tslots[i].gps_pt->dist_to_next - route_dist);
					prob_r_l = get_transition_probability(dist_excess /*, trans_param*/);
					if (end_line_Node == tslots[i].last_Node[r])
					{
						prob_r_l *= 1E-10;
					}
					// prob_r_l = get_transition_probability(tslots[i].dist_to_next, route_dist, trans_param);
					// if (prob_r_l==0) prob_r_l = 1E-100;
				}

				tmp_score = prob_r_l * tslots[i].scores[r]; // calculate score	!11
#ifdef DEBUG_R
				if (startDebug == TRUE)
				{
					printf("I:%i,R:%2i TRANS_R_L:%E * SCORE_R:%LE = SCORE:%LE. %f %f\n", i, r, prob_r_l, tslots[i].scores[r], tmp_score, tslots[i].dist_to_next, route_dist);
				}
#endif // DEBUG
	   // if(tslots[i].scores[r]>0 && tmp_score == 0 && prob_r_l > 0)
	   //	tmp_score = 1E-300;

				if (max_r_score < tmp_score) // find the best score
				{
					max_r_score = tmp_score;
					c = r;
					tslots[i + 1].last_Node[l] = start_line_Node;
				}
			}
			tslots[i + 1].scores[l] = score[l] * max_r_score; // store the best score and parent
			// if( max_r_score>0 && tslots[i+1].scores[l] == 0)
			// tslots[i+1].scores[l] = 1E-300;
			tslots[i + 1].parents[l] = c;
			// if (tslots[i+1].scores[l] == 0)
			//	c = -1;
#ifdef DEBUG_SCORES
			if (startDebug == TRUE)
			{
				printf("I+1:%i,L:%2i BEST R SCORE:%LE TOTAL SCORE:%LE PARENT:%i.\n", i + 1, l, max_r_score, tslots[i + 1].scores[l], c);
			}
#endif // DEBUG
		}
		if (c == -1 || i == gpscount - 2 + start) // no parent found or last run of cycle for i
		{

			if (c == -1 && i != gpscount - 2 + start)
			{
				i--;
			}

			tmp_score = 0;

			for (j = 0; j < tslots[i + 1].count; j++) // find the best score in the last timeslot
			{
				if (tmp_score < tslots[i + 1].scores[j])
				{
					tmp_score = tslots[i + 1].scores[j];
					c = j;
				}
			}

			if (c == -1)
			{
				for (j = i + 1; j >= /*start+*/ i_break; j--)
				{
					map_pts[j].longitude = tslots[j].gps_pt->longitude;
					map_pts[j].latitude = tslots[j].gps_pt->latitude;
					map_pts[j].geocoded = 0;
				}
			}
			else
			{
				for (j = i + 1; j >= /*start+*/ i_break; j--) // go back and find the parents
				{
					map_pts[j].longitude = tslots[j].points[c].x;
					map_pts[j].latitude = tslots[j].points[c].y;
					map_pts[j].geocoded = 1;
					map_pts[j].line_id = tslots[j].line_id[c];
					map_pts[j].start_Node = tslots[j].start_Node[c];
					map_pts[j].end_Node = tslots[j].end_Node[c];
					map_pts[j].dist_to_start = tslots[j].dist_to_start[c];
					map_pts[j].dist_start_end = tslots[j].dist_start_end[c];
					map_pts[j].line = tslots[j].line[c];
					map_pts[j].one_way = tslots[j].one_way[c];

#ifdef DEBUG_FIND_MM
					if (51.056054 >= map_pts[j].latitude && map_pts[j].latitude >= 51.056052)
					{ // Lat: 51.010078 Lng: -2.194237
						startDebug = TRUE;
						debug_count = 30;
					}
					if (startDebug == TRUE)
					{
						printf("Original coords[%i/%i/%i]: %2.8f,%2.9f\n", i_break, j, i + 1, tslots[j].gps_pt->latitude, tslots[j].gps_pt->longitude);
						printf("Matched  coords[%i/%i/%i]: %2.8f,%2.9f\n", i_break, j, i + 1, map_pts[j].latitude, map_pts[j].longitude);
						if (debug_count != 0)
							debug_count--;
						else
							startDebug = FALSE;
						if (j == i_break)
							startDebug = FALSE;
					}
#endif // DEBUG_FIND_MM

#ifdef DEBUG_START_END
					if (j == i + 1)
					{
						printf("[%2.8f,%2.9f],", map_pts[j].latitude, map_pts[j].longitude);
					}
					if (j == i_break)
					{
						printf("[%2.8f,%2.9f],\n", map_pts[j].latitude, map_pts[j].longitude);
					}
#endif // DEBUG_START_END

					c = tslots[j].parents[c];
				}
			}
			i_break = i + 2;
			i++;
		}

		/*for(j=0;j<tslots[i+1].count;j++){
			if(tslots[i+1].scores[j]<1E-200){
				for(k=0;k<tslots[i+1].count;k++){
					tslots[i+1].scores[k]*=1E20;
				}
				break;
			}
		}*/
	}

	map_pts[tslots[gpscount + start - 1].gps_pt->index].longitude = map_pts[gpscount + start - 1].longitude;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].latitude = map_pts[gpscount + start - 1].latitude;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].line_id = map_pts[gpscount + start - 1].line_id;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].dist_to_start = map_pts[gpscount + start - 1].dist_to_start;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].dist_start_end = map_pts[gpscount + start - 1].dist_start_end;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].line = map_pts[gpscount + start - 1].line;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].start_Node = map_pts[gpscount + start - 1].start_Node;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].end_Node = map_pts[gpscount + start - 1].end_Node;
	map_pts[tslots[gpscount + start - 1].gps_pt->index].geocoded = map_pts[gpscount + start - 1].geocoded;

	for (i = gpscount + start - 1; i > start; i--)
	{
		map_pts[tslots[i - 1].gps_pt->index].longitude = map_pts[i - 1].longitude;
		map_pts[tslots[i - 1].gps_pt->index].latitude = map_pts[i - 1].latitude;
		map_pts[tslots[i - 1].gps_pt->index].line_id = map_pts[i - 1].line_id;
		map_pts[tslots[i - 1].gps_pt->index].dist_to_start = map_pts[i - 1].dist_to_start;
		map_pts[tslots[i - 1].gps_pt->index].dist_start_end = map_pts[i - 1].dist_start_end;
		map_pts[tslots[i - 1].gps_pt->index].line = map_pts[i - 1].line;
		map_pts[tslots[i - 1].gps_pt->index].start_Node = map_pts[i - 1].start_Node;
		map_pts[tslots[i - 1].gps_pt->index].end_Node = map_pts[i - 1].end_Node;
		map_pts[tslots[i - 1].gps_pt->index].geocoded = map_pts[i - 1].geocoded;
		if (((tslots[i].gps_pt->index) - (tslots[i - 1].gps_pt->index)) > 1)
		{ // yes if there are points between index i and i+1

			// for(j=tslots[i-1].gps_pt->index+1; j<(tslots[i].gps_pt->index);j++){
			// map_pts[j].longitude = map_pts[i].longitude;
			// map_pts[j].latitude =map_pts[i].latitude;

			if (map_pts[i - 1].geocoded == 0 || map_pts[i].geocoded == 0)
			{
				for (j = 1; j < (tslots[i].gps_pt->index - tslots[i - 1].gps_pt->index); j++)
				{
					map_pts[tslots[i - 1].gps_pt->index + j].longitude = map_pts[i - 1].longitude;
					map_pts[tslots[i - 1].gps_pt->index + j].latitude = map_pts[i - 1].latitude;
				}
			}
			else
			{

				nodes[map_pts[i - 1].start_Node].Gcost = map_pts[i - 1].dist_to_start;								 // tslots[i].start_Node[r]].Gcost=tslots[i].dist_to_start[r];  //fill distance from candidate to the first start node
				nodes[map_pts[i - 1].end_Node].Gcost = map_pts[i - 1].dist_start_end - map_pts[i - 1].dist_to_start; // fill distance from candidate to the second start node

				if (map_pts[i].one_way != 1)
				{
					nodes[map_pts[i].end_Node].neighborsID[nodes[map_pts[i].end_Node].neighborsCount] = nodes_count; // add new neighbors to node which is on line where target
					nodes[map_pts[i].end_Node].neighborsDistance[nodes[map_pts[i].end_Node].neighborsCount] = map_pts[i].dist_start_end - map_pts[i].dist_to_start;
					nodes[map_pts[i].end_Node].neighborsCount++;
				}

				if (map_pts[i].one_way != -1)
				{
					nodes[map_pts[i].start_Node].neighborsID[nodes[map_pts[i].start_Node].neighborsCount] = nodes_count; // add new neighbors to node which is on line where target
					nodes[map_pts[i].start_Node].neighborsDistance[nodes[map_pts[i].start_Node].neighborsCount] = map_pts[i].dist_to_start;
					nodes[map_pts[i].start_Node].neighborsCount++;
				}

				route_dist = FindPointsOnPath(nodes, nodes_count + 1, tslots[i - 1].gps_pt->index, tslots[i].gps_pt->index, nodes_count, map_pts);

				if (map_pts[i].one_way != 1)
				{
					nodes[map_pts[i].end_Node].neighborsCount--; // delete adde neighbors for next routing
				}
				if (map_pts[i].one_way != -1)
				{
					nodes[map_pts[i].start_Node].neighborsCount--;
				}
			}
		}
	}
}

MM_ERRORS mm_map_match_recursive(MM_WS *ws, MM_REQUEST_W *mm_request, MM_RESPONSE_W *mm_response)
{
#ifdef TESTING_COUNTS
	mm_map_match_count++;
#endif // TESTING_COUNTS

	int i, nodes_count, gpscount, start;
	MM_ERRORS ret_code;
	Node *nodes;
	TIMESLOT *tslots;
	MM_GPS_PT_W *gps_pts;
	MM_MAP_PT_W *map_pts;

	char query[4096];
	xdb_t *xdb;
	xdb_result_t *xdb_result;
	int rows_count;

	double min_long = DBL_MAX;
	double min_lat = DBL_MAX;
	double max_long = -DBL_MAX;
	double max_lat = -DBL_MAX;

	const char *query_ptr = query;

	gps_pts = mm_request->gps_pts;
	gpscount = mm_request->gps_pts_size;
	map_pts = mm_response->map_pts;

	if (gpscount > MAX_GPS_COUNT)
	{
		MM_REQUEST_W split_mm_request;
		MM_RESPONSE_W split_mm_response;
		int split_count;
		split_count = gpscount / 2;

		split_mm_request.gps_pts = gps_pts;
		split_mm_request.gps_pts_size = split_count;
		split_mm_response.map_pts = map_pts;
		mm_map_match_recursive(ws, &split_mm_request, &split_mm_response);

		split_mm_request.gps_pts = gps_pts + split_count;
		split_mm_response.map_pts = map_pts + split_count;
		split_mm_request.gps_pts_size = gpscount - split_count;
		mm_map_match_recursive(ws, &split_mm_request, &split_mm_response);

		return MM_OK;
	}

	ret_code = MM_OK; // initial set-up
#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif							   // TESTING_TIMES1
								   // FIND BOUNDARY VALUES
	for (i = 0; i < gpscount; i++) // count boundary of points
	{
		if (gps_pts[i].latitude > max_lat)
			max_lat = gps_pts[i].latitude;
		if (gps_pts[i].latitude < min_lat)
			min_lat = gps_pts[i].latitude;
		if (gps_pts[i].longitude > max_long)
			max_long = gps_pts[i].longitude;
		if (gps_pts[i].longitude < min_long)
			min_long = gps_pts[i].longitude;
	}

	// load data from DB
	sprintf(query, "SELECT AsWKB(%s),%s,%s,start_lon, start_lat,end_lon,end_lat,distance_reversed FROM %s WHERE MBRIntersects("
				   "PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
			NEW_LINE_ATTRIBUTE,
			WAY_ID_ATTRIBUTE,
			WAY_LENGTH_ATTRIBUTE,
			NEW_STREETS_TABLE,
			min_long - GPS_DISCREPANCY, max_lat + GPS_DISCREPANCY, // top left
			max_long + GPS_DISCREPANCY, max_lat + GPS_DISCREPANCY, // top right
			max_long + GPS_DISCREPANCY, min_lat - GPS_DISCREPANCY, // bottom right
			min_long - GPS_DISCREPANCY, min_lat - GPS_DISCREPANCY, // bottom left
			min_long - GPS_DISCREPANCY, max_lat + GPS_DISCREPANCY, // top left
			NEW_LINE_ATTRIBUTE,
			MAX_DB_LINES);

	xdb = &(ws->xdb);
#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&boundary_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES1

#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES1

	xdb_result = xdb_query(xdb, query_ptr);

#ifdef DEBUG_SELECT
	printf("Count of rows in result: %i   %s; \n", xdb_result->row_count, query);
#endif // DEBUG_SELECT

#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&db_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES1

	if (xdb_result == NULL)
	{
		for (i = 0; i < gpscount; i++) // copy GPS points to result
		{
			map_pts[i].longitude = gps_pts[i].longitude;
			map_pts[i].latitude = gps_pts[i].latitude;
		}
		return MM_NO_DB_RESPONSE;
	}

	rows_count = xdb_result->row_count;

	if (rows_count == 0)
	{
		for (i = 0; i < gpscount; i++) // copy GPS points to result
		{
			map_pts[i].longitude = gps_pts[i].longitude;
			map_pts[i].latitude = gps_pts[i].latitude;
		}
		xdb_free_result(xdb_result); // ADDED
		return MM_NO_LINES;
	}

	if ((rows_count >= MAX_DB_LINES) && gpscount > 1)
	{
		xdb_free_result(xdb_result);

		MM_REQUEST_W part_mm_request;
		MM_RESPONSE_W part_mm_response;
		int part_count;
		part_count = gpscount / 2;

		part_mm_request.gps_pts = gps_pts;
		part_mm_request.gps_pts_size = part_count;
		part_mm_response.map_pts = map_pts;
		mm_map_match_recursive(ws, &part_mm_request, &part_mm_response);

		part_mm_request.gps_pts = gps_pts + part_count;
		part_mm_response.map_pts = map_pts + part_count;
		part_mm_request.gps_pts_size = gpscount - part_count;
		mm_map_match_recursive(ws, &part_mm_request, &part_mm_response);

		return MM_OK;
	}

#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif															// TESTING_TIMES1
	nodes = (Node *)calloc((rows_count * 2) + 1, sizeof(Node)); // accocate nodes for max each line has 2 points + one as target

	if (NULL == (tslots = (TIMESLOT *)calloc(gpscount, sizeof(TIMESLOT)))) // memory for time slots
	{
		// printf("Can\'t allocate memory for time slots.");
		return MM_NO_MEMORY;
	}

	for (i = 0; i < gpscount; i++) // no neighbors yet
	{
		tslots[i].count = 0;
	}

	// set *gps_pt and compute dist_to_next for all tslots
	init_time_slots(gps_pts, gpscount, tslots);
#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&init_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES1

#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES1
	   // fill nodes and tslots
	ret_code = create_candidates_set(tslots, gpscount, nodes, &nodes_count, xdb_result);

#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&create_candidates_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES1

	if (ret_code != MM_OK) // error
	{
		for (i = 0; i < gpscount; i++) // copy GPS points to result
		{
			map_pts[i].longitude = gps_pts[i].longitude;
			map_pts[i].latitude = gps_pts[i].latitude;
		}
		return ret_code;
	}
#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif // TESTING_TIMES1
	for (i = 0; i < gpscount; i++)
	{
		if (tslots[i].count == 0)
		{
			map_pts[i].longitude = gps_pts[i].longitude;
			map_pts[i].latitude = gps_pts[i].latitude;
		}
		else
		{
			start = i;
			for (i = start + 1; i < gpscount; i++)
			{
				if (tslots[i].count == 0)
				{
					map_maptch_part(tslots, map_pts, nodes, i - start, nodes_count, start);
					map_pts[i].longitude = gps_pts[i].longitude;
					map_pts[i].latitude = gps_pts[i].latitude;
					break;
				}
			}
			if (i == gpscount)
				map_maptch_part(tslots, map_pts, nodes, i - start, nodes_count, start);
		}
	}
#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&mm_part_time, &tsBefore, &tsAfter);
#endif // TESTING_TIMES1
	xdb_free_result(xdb_result);
	if (nodes)
		free(nodes);

	return ret_code;
}

#ifdef TESTING_COUNTS
void debug_print_counts()
{
	printf("pointOnSegment_count : %i\n", pointOnSegment_count);
	printf("HaversineDistance_count : %i\n", HaversineDistance_count);
	printf("FindPath_count : %i\n", FindPath_count);
	printf("snap_to_segment_count : %i\n", snap_to_segment_count);
	printf("get_transition_probability_count : %i\n", get_transition_probability_count);
	printf("create_candidates_set_count : %i\n", create_candidates_set_count);
	printf("mm_map_match_count : %i\n", mm_map_match_count);
}
#endif // TESTING_COUNTS

#ifdef TESTING_TIMES
void debug_print_times()
{
#ifdef TESTING_TIMES1
	printf("db_time: %i\n", db_time);
	printf("create_candidates_time: %i\n", create_candidates_time);
	printf("boundary_time: %i\n", boundary_time);
	printf("init_time: %i\n", init_time);
	printf("mm_part_time: %i\n", mm_part_time);
#endif // TESTING_TIMES1

#ifdef TESTING_TIMES2
	printf("nodes_time: %i\n", nodes_time);
	printf("snap_time: %i\n", snap_time);
#endif // TESTING_TIMES2

#ifdef TESTING_TIMES3
	printf("pointOnSegment_time: %i\n", pointOnSegment_time);
	printf("distances_time: %i\n", distances_time);
#endif // TESTING_TIMES3

#ifdef TESTING_TIMES4
	printf("test_time: %i\n", test_time);
	printf("test_time2: %i\n", test_time2);
	printf("test_time3: %i\n", test_time3);
#endif // TESTING_TIMES4
}
#endif // TESTING_TIMES

MM_ERRORS mm_map_match(MM_WS *ws, MM_REQUEST *mm_request, MM_RESPONSE *mm_response)
{

	MM_ERRORS result;
	MM_REQUEST_W mm_request_recursive;
	MM_RESPONSE_W mm_response_recursive;
	int i, from_index, gpscount, pts_skipped = 0;
	MM_GPS_PT_W *gps_pts;
	MM_MAP_PT_W *mm_pts;

	gpscount = mm_request->gps_pts_size;

	gps_pts = (MM_GPS_PT_W *)calloc(mm_request->gps_pts_size, sizeof(MM_GPS_PT_W));
	mm_pts = (MM_MAP_PT_W *)calloc(mm_request->gps_pts_size, sizeof(MM_MAP_PT_W));

	// gps_indexes = (int*)calloc(mm_request->gps_pts_size, sizeof(int));

	gps_pts[0].latitude = mm_request->gps_pts[0].latitude;
	gps_pts[0].longitude = mm_request->gps_pts[0].longitude;
	gps_pts[0].heading = mm_request->gps_pts[0].heading;
	gps_pts[0].dist_to_next = mm_request->gps_pts[0].dist_to_next;
	gps_pts[0].index = 0;
	gpscount = 1;

	for (i = 1; i < mm_request->gps_pts_size; i++) // start from second point
	{
		// distance between two points is big enough - use point
		// double aa = gps_pts[gpscount-1].latitude - mm_request->gps_pts[i].latitude;
		// aa = fabs(aa);

		if (i == (mm_request->gps_pts_size) - 1 || (fabs(gps_pts[gpscount - 1].latitude - mm_request->gps_pts[i].latitude) > MAX_GPS_DIFF) ||
			(fabs(gps_pts[gpscount - 1].longitude - mm_request->gps_pts[i].longitude) > MAX_GPS_DIFF) || pts_skipped > MAX_GPS_PTS_SKIP)
		{
			if (((fabs(mm_request->gps_pts[i - 1].latitude - mm_request->gps_pts[i].latitude) > MAX_GPS_DIFF) || (fabs(mm_request->gps_pts[i - 1].longitude - mm_request->gps_pts[i].longitude) > MAX_GPS_DIFF)) && gps_pts[gpscount - 1].index != (i - 1))
			{
				gps_pts[gpscount].latitude = mm_request->gps_pts[i - 1].latitude;
				gps_pts[gpscount].longitude = mm_request->gps_pts[i - 1].longitude;
				gps_pts[gpscount].heading = mm_request->gps_pts[i - 1].heading;
				gps_pts[gpscount].dist_to_next = mm_request->gps_pts[i - 1].dist_to_next;
				gps_pts[gpscount].index = i - 1;
				gpscount++;
			}
			gps_pts[gpscount].latitude = mm_request->gps_pts[i].latitude;
			gps_pts[gpscount].longitude = mm_request->gps_pts[i].longitude;
			gps_pts[gpscount].heading = mm_request->gps_pts[i].heading;
			gps_pts[gpscount].dist_to_next = mm_request->gps_pts[i].dist_to_next;
			gps_pts[gpscount].index = i;
			gpscount++;
			pts_skipped = 0;
		}
		else
		{
			pts_skipped++;
		}
	}

	mm_request_recursive.gps_pts = gps_pts;
	mm_request_recursive.gps_pts_size = gpscount;
	mm_response_recursive.map_pts = mm_pts;

#ifdef DEBUG
	FILE *fw = NULL;
	char vypis_gps[2048], tempchar[2048];
	sprintf(vypis_gps, "[");

	fw = fopen("/home/adminuser/workspace/mmnew/Debug/src/original_filtered.csv", "w");

	for (i = 0; i < gpscount; i++)
	{
		sprintf(tempchar, "[%f,%f],", gps_pts[i].latitude, gps_pts[i].longitude);
		strcat(vypis_gps, tempchar);

		if (i % 100)
		{
			fputs(vypis_gps, fw);
			sprintf(vypis_gps, "");
		}
	}

	fputs(vypis_gps, fw);
	fprintf(fw, "]\n");
	fflush(fw);
	fclose(fw);
#endif // DEBUG

	result = mm_map_match_recursive(ws, &mm_request_recursive, &mm_response_recursive);

	for (i = mm_request->gps_pts_size - 1; i >= 0; i--) // set from last to first point
	{
		mm_response->map_pts[i].latitude = mm_response_recursive.map_pts[i].latitude;
		mm_response->map_pts[i].longitude = mm_response_recursive.map_pts[i].longitude;
	}
	/*
	gpscount--;							//this variable will be now used to get indexes
	from_index = gps_pts[gpscount].index;	//set last index

	for (i = mm_request->gps_pts_size - 1; i >= 0; i--)		//set from last to first point
	{
		if (i < from_index)
		{
			gpscount--;
			from_index = gps_pts[gpscount].index;
		}
		mm_response->map_pts[i].latitude = mm_response->map_pts[gpscount].latitude;
		mm_response->map_pts[i].longitude = mm_response->map_pts[gpscount].longitude;
	}*/

	free(gps_pts);
	free(mm_pts);
	// free(gps_indexes);

	return result;

	// return mm_map_match_recursive(ws, mm_request, mm_response);
}
