//#define ROBOT_DEVICE	      "/dev/myrobot"
//#define STATION_DEVICE	      "/dev/mystation"
#define ROBOT_DEVICE	      "/dev/ttyUSB1"
#define STATION_DEVICE	      "/dev/ttyUSB2"
#define STATION_MAX		      	15
#define STATION_START		  	2

#define LOOP_uSLEEP_TIME		500000
uint16_t isfullscreen	   =  1;
uint8_t  stationignore[STATION_MAX];
//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...)    fprintf(stderr, fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)    /* Don't do anything in release builds */
#endif
