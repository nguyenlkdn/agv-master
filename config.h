//#define ROBOT_DEVICE	      "/dev/myrobot"
//#define STATION_DEVICE	      "/dev/mystation"
#define ROBOT_DEVICE	      "/dev/ttyUSB1"
#define STATION_DEVICE	      "/dev/ttyUSB0"
#define STATION_MAX		      15
#define STATION_START		  2
uint16_t STATION1_ENABLE   =  0;
uint16_t STATION2_ENABLE   =  1;
uint16_t STATION3_ENABLE   =  1;
uint16_t STATION4_ENABLE   =  1;
uint16_t STATION5_ENABLE   =  1;
uint16_t isfullscreen	   =  1;
uint8_t  stationignore[STATION_MAX];
//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...)    fprintf(stderr, fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)    /* Don't do anything in release builds */
#endif
