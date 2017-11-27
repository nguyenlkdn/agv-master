#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <curses.h>
#include <errno.h>
#include <modbus.h>
#include <time.h>
#include <pthread.h>
#include <gtk/gtk.h>

#include "config.h"
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
//fonts color
#define FBLACK      "\033[30;"
#define FRED        "\033[31;"
#define FGREEN      "\033[32;"
#define FYELLOW     "\033[33;"
#define FBLUE       "\033[34;"
#define FPURPLE     "\033[35;"
#define D_FGREEN    "\033[6;"
#define FWHITE      "\033[7;"
#define FCYAN       "\x1b[36m"

//background color
#define BBLACK      "40m"
#define BRED        "41m"
#define BGREEN      "42m"
#define BYELLOW     "43m"
#define BBLUE       "44m"
#define BPURPLE     "45m"
#define D_BGREEN    "46m"
#define BWHITE      "47m"

//end color
#define NONE        "\033[0m"

enum {TCP, RTU};

#define STATION_MAX         5
#define ROBOT_MAX           1

uint16_t STATION1_WRITING  =  0;
uint16_t STATION2_WRITING  =  0;
uint16_t STATION3_WRITING  =  0;
uint16_t STATION4_WRITING  =  0;
uint16_t STATION5_WRITING  =  0;

#define STATION_TIMEOUT_uS        0
#define STATION_TIMEOUT_S         1
#define STATION_WRITE_TIMEOUT_uS  0
#define STATION_WRITE_TIMEOUT_S   1
#define ROBOT_WRITE_TIMEOUT_S     1
#define ROBOT_WRITE_TIMEOUT_uS    0
#define ROBOT_READ_TIMEOUT_S      1
#define ROBOT_READ_TIMEOUT_uS     0

/*
  User Interface Defination
*/
uint16_t UserCallingBuffer[10];
typedef struct _UserCallingHistory{
  uint16_t stationid;
  uint64_t processed_duration;
  time_t datetime;
  uint16_t status;
  uint16_t counter;
}UserCallingHistory;

enum{
  STOP=0,
  PROC=1
};


UserCallingHistory History[100];
/*

*/
uint8_t  station1_processed=0;
uint8_t  station2_processed=0;
uint8_t  station3_processed=0;
uint8_t  station4_processed=0;
uint8_t  station5_processed=0;

uint32_t station1_counter=0;
uint32_t station2_counter=0;
uint32_t station3_counter=0;
uint32_t station4_counter=0;
uint32_t station5_counter=0;

uint64_t station1_read_err=0;
uint16_t station2_read_err=0;
uint16_t station3_read_err=0;
uint16_t station4_read_err=0;
uint16_t station5_read_err=0;

uint64_t station1_write_err=0;
uint16_t station2_write_err=0;
uint16_t station3_write_err=0;
uint16_t station4_write_err=0;
uint16_t station5_write_err=0;

int16_t robotRegister_received[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t robotRegister_sent[MODBUS_TCP_MAX_ADU_LENGTH];

int16_t station1Register_received[MODBUS_TCP_MAX_ADU_LENGTH];
int16_t station2Register_received[MODBUS_TCP_MAX_ADU_LENGTH];
int16_t station3Register_received[MODBUS_TCP_MAX_ADU_LENGTH];
int16_t station4Register_received[MODBUS_TCP_MAX_ADU_LENGTH];
int16_t station5Register_received[MODBUS_TCP_MAX_ADU_LENGTH];

uint16_t station1Register_sent[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station2Register_sent[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station3Register_sent[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station4Register_sent[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station5Register_sent[MODBUS_TCP_MAX_ADU_LENGTH];

uint16_t station1Register_sent_previous[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station2Register_sent_previous[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station3Register_sent_previous[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station4Register_sent_previous[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t station5Register_sent_previous[MODBUS_TCP_MAX_ADU_LENGTH];

uint16_t station1_confimation=0;
uint16_t station2_confimation=0;
uint16_t station3_confimation=0;
uint16_t station4_confimation=0;
uint16_t station5_confimation=0;
void GUIInit(int argc, char *argv[]);
void *stationThread(void *vargp);
void *robotThread(void *vargp);
void *userThread();
void *userInterface(void *vargp);
void initThread();
void robotInit();
void stationInit();
char *getTime();

/*

*/
static void callback1( GtkWidget *widget, gpointer data);
static void callback2( GtkWidget *widget, gpointer data);
static void callback3( GtkWidget *widget, gpointer data);
static void callback4( GtkWidget *widget, gpointer data);
static void callback5( GtkWidget *widget, gpointer data);
/*

*/
modbus_t *ctx;
modbus_t *modbus_rtu_robot_ctx;
modbus_t *modbus_rtu_station_ctx;
modbus_mapping_t *mb_mapping;
modbus_mapping_t *modbus_rtu_robot_mb_mapping;
modbus_mapping_t *modbus_rtu_station_mb_mapping;
modbus_mapping_t *modbus_rtu_station1_mb_mapping;
modbus_mapping_t *modbus_rtu_station2_mb_mapping;
modbus_mapping_t *modbus_rtu_station3_mb_mapping;
modbus_mapping_t *modbus_rtu_station4_mb_mapping;
modbus_mapping_t *modbus_rtu_station5_mb_mapping;
int16_t robot_status = 0;
uint16_t robot_control = 0;
/*
*/
int main(int argc, char *argv[])
{
  initThread();
  robotInit();
  stationInit();
  pthread_t stationThread_id, userThread_id, robotThread_id, userInterface_id;
  pthread_create(&userThread_id, NULL, userThread, NULL);
  pthread_create(&userInterface_id, NULL, userInterface, NULL);
  //pthread_create(&robotThread_id, NULL, robotThread, NULL);
  //pthread_create(&stationThread_id, NULL, stationThread, NULL);
  //pthread_join(robotThread_id, NULL);
  pthread_join(userInterface_id, NULL);
  pthread_join(userThread_id, NULL);
  //pthread_join(stationThread_id, NULL);
  return 0;
}

void *userThread()
{
  GUIInit(0, NULL);
  char button[50];
  while (1)
  {
    //printf("%s %s\n", __FUNCTION__, "geting user control");
    scanf("%s" , button) ;
    printf("Pressed key: %s\n", button);
    if(strcmp(button, "call")==0)
    {
      //printf("Select station id: ");
      //int id=0;
      //scanf("%d", &id);
      //robotRegister_sent[0] = (uint16_t)id;
    }
    else if(strcmp(button, "send")==0)
    {
      printf("Type station [2, 3, 4, 5] will be processed: ");
      uint16_t error=0;
      char key[5];
      //key = getch();
      while(++error < 5)
      {
        scanf("%s", key);
        if(strcmp(key, "S2")==0)
        {
          printf("[SUCCESS] Robot was being sent to Station 2\n");
          robotRegister_sent[0] = 2;
          robot_control = 2;
        }
        else if(strcmp(key, "S3")==0)
        {
          printf("[SUCCESS] Robot was being sent to Station 3\n");
          robotRegister_sent[0] = 3;
          robot_control = 3;
        }
        else if(strcmp(key, "S4")==0)
        {
          printf("[SUCCESS] Robot was being sent to Station 4\n");
          robotRegister_sent[0] = 4;
          robot_control = 4;
        }
        else if(strcmp(key, "S5")==0)
        {
          printf("[SUCCESS] Robot was being sent to Station 5\n");
          robotRegister_sent[0] = 5;
          robot_control = 5;
        }
        else
        {
          printf("\nInvalid command [2, 3, 4, 5]\n");
          robot_control = 0;
        }

        //scanf("%c" , &key);
        // switch(key)
        // {
        //   case '1':
        //     printf("[SUCCESS] Robot was being sent to Station 1!\n");
        //   break;
        //   case '2':
        //     printf("[SUCCESS] Robot was being sent to Station 2!\n");
        //   break;
        //   case '3':
        //     printf("[SUCCESS] Robot was being sent to Station 3!\n");
        //   break;
        //   case '4':
        //     printf("[SUCCESS] Robot was being sent to Station 4!\n");
        //   break;
        //   case '5':
        //     printf("[SUCCESS] Robot was being sent to Station 5!\n");
        //   break;
        //   default:
        //     printf("\nInvalid command[2, 3, 4, 5]\n");
        //   break;
        // }
      }
    }
    else
    {
      printf("Unknow Command, please type only the (call/send) commands\n");
    }
    usleep(1000000);
  }
}

// A normal C function that is executed as a thread 
// when its name is specified in pthread_create()
void *stationThread(void *vargp)
{
  uint8_t stationid=0;
  while (1)
  {
    //printf("%s %s\n", __FUNCTION__, "Processing Station
    int rc;
    //modbus_flush(modbus_rtu_station_ctx);
    if(STATION1_ENABLE == 1)
    {
      stationid=1;
      modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
      modbus_set_slave(modbus_rtu_station_ctx, stationid);
      modbus_flush(modbus_rtu_station_ctx);
      rc = modbus_read_registers(modbus_rtu_station_ctx, 1, 5, station1Register_received);
      if(rc == -1)
      {
        memset(station1Register_received, -1, sizeof(station1Register_received));
        station1_read_err++;
        printf("Reading from Station 1: TIMEOUT\n");
        STATION1_WRITING=0;
        //printf("[ERROR] Cannot read from station: %d\n", stationid);
      }
      else
      {
        usleep(500000);
        STATION1_WRITING = 1;
        // printf("Reading from Station 1: ");
        // int i;
        // for(i=0;i<rc;i++)
        // {
        //   printf("%3d", station1Register_received[i]);
        // }
        // printf("\n");
      }
      usleep(500000);
    }

    if(STATION2_ENABLE == 1)
    {
      stationid=2;
      modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
      modbus_set_slave(modbus_rtu_station_ctx, stationid);
      modbus_flush(modbus_rtu_station_ctx);
      rc = modbus_read_registers(modbus_rtu_station_ctx, 1, 5, station2Register_received);
      if(rc == -1)
      {
        memset(station2Register_received, -1, sizeof(station2Register_received));
        station1_read_err++;
        printf("Reading from Station 2: TIMEOUT\n");
        STATION2_WRITING=0;
        //printf("[ERROR] Cannot read from station: %d\n", stationid);
      }
      else
      {
        usleep(500000);
        STATION2_WRITING = 1;
        // printf("Reading from Station 1: ");
        // int i;
        // for(i=0;i<rc;i++)
        // {
        //   printf("%3d", station1Register_received[i]);
        // }
        // printf("\n");
      }
      usleep(500000);
    }
    if(STATION3_ENABLE == 1)
    {
      stationid=3;
      modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
      modbus_set_slave(modbus_rtu_station_ctx, stationid);
      rc = modbus_read_registers(modbus_rtu_station_ctx, 1, 5, station3Register_received);
      if(rc == -1)
      {
        printf("Reading from Station 3: TIMEOUT\n");
        memset(station3Register_received, -1, sizeof(station3Register_received));
        station3_read_err++;
        //printf("[ERROR] Cannot read from station: %d\n", stationid);
      }
      else
      {
        STATION3_WRITING = 1;
      }
      usleep(500000);
    }
    if(STATION4_ENABLE == 1)
    {
      stationid=4;
      modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
      modbus_set_slave(modbus_rtu_station_ctx, stationid);
      rc = modbus_read_registers(modbus_rtu_station_ctx, 1, 5, station4Register_received);
      if(rc == -1)
      {
        memset(station4Register_received, -1, sizeof(station4Register_received));
        station4_read_err++;
        printf("Reading from Station 4: TIMEOUT\n");
      }
      else
      {
        STATION4_WRITING = 1;
        // printf("Reading from Station 4: ");
        // int i;
        // for(i=0;i<rc;i++)
        // {
        //   printf("%3d", station4Register_received[i]);
        // }
        // printf("\n");
      }
      usleep(500000);

    }
    if(STATION5_ENABLE == 1)
    {
      stationid=5;
      modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
      modbus_set_slave(modbus_rtu_station_ctx, stationid);
      modbus_flush(modbus_rtu_station_ctx);
      rc = modbus_read_registers(modbus_rtu_station_ctx, 1, 5, station5Register_received);
      if(rc == -1)
      {
        memset(station5Register_received, -1, sizeof(station5Register_received));
        station5_read_err++;
        STATION5_WRITING=0;
        printf("Reading from Station 5: TIMEOUT\n");
        //printf("[ERROR] Cannot read from station: %d\n", stationid);
      }
      else
      {
        STATION5_WRITING=1;
        usleep(500000);
        // printf("Reading from Station 5: ");
        // int i;
        // for(i=0;i<rc;i++)
        // {
        //   printf("%3d", station5Register_received[i]);
        // }
        // printf("\n");
      }
    }

/*
  
*/
    if(STATION1_WRITING)
    {
      int issend=0;
      int i;
      for(i=0;i<10;i++)
      {
        if(station1Register_sent[i] != station1Register_sent_previous[i])
        {
          issend = 1;
          break;
        }
      }
      if(issend == 1)
      {
        // printf("Writing to Station 1: ");
        // int i;
        // for(i=0;i<10;i++)
        // {
        //   printf("%3d", station1Register_sent[i]);
        // }
        // printf("\n");
        modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_WRITE_TIMEOUT_S, STATION_WRITE_TIMEOUT_uS);
        modbus_set_debug(modbus_rtu_station_ctx, TRUE);
        modbus_set_slave(modbus_rtu_station_ctx, 1);
        modbus_set_debug(modbus_rtu_station_ctx, FALSE);
        rc = modbus_write_registers(modbus_rtu_station_ctx, 20, 10, station1Register_sent);
        if(rc != -1)
        {
          for(i=0;i<10;i++)
          {
            station1Register_sent_previous[i] = station1Register_sent[i];
            STATION1_WRITING=0;
          }
        }

      }
    }

    if(STATION2_WRITING)
    {
      int issend=0;
      int i;
      for(i=0;i<10;i++)
      {
        if(station2Register_sent[i] != station2Register_sent_previous[i])
        {
          issend = 1;
          break;
        }
      }
      if(issend == 1)
      {
        printf("Writing to Station 2\n");
        modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_WRITE_TIMEOUT_S, STATION_WRITE_TIMEOUT_uS);
        modbus_set_slave(modbus_rtu_station_ctx, 2);
        modbus_write_registers(modbus_rtu_station_ctx, 20, 10, station2Register_sent);
        for(i=0;i<10;i++)
        {
          station2Register_sent_previous[i] = station2Register_sent[i];
        }
      }
    }

    if(STATION3_WRITING)
    {
      int issend=0;
      int i;
      for(i=0;i<10;i++)
      {
        if(station3Register_sent[i] != station3Register_sent_previous[i])
        {
          issend = 1;
          break;
        }
      }
      if(issend == 1)
      {
        printf("Writing to Station 3\n");
        modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_WRITE_TIMEOUT_S, STATION_WRITE_TIMEOUT_uS);
        modbus_set_slave(modbus_rtu_station_ctx, 3);
        modbus_write_registers(modbus_rtu_station_ctx, 20, 10, station3Register_sent);
        for(i=0;i<10;i++)
        {
          station3Register_sent_previous[i] = station3Register_sent[i];
        }
      }
    }

    if(STATION4_WRITING)
    {
      int issend=0;
      int i;
      for(i=0;i<10;i++)
      {
        if(station4Register_sent[i] != station4Register_sent_previous[i])
        {
          issend = 1;
          break;
        }
      }
      if(issend == 1)
      {
        printf("Writing to Station 4\n");
        modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_WRITE_TIMEOUT_S, STATION_WRITE_TIMEOUT_uS);
        modbus_set_slave(modbus_rtu_station_ctx, 4);
        modbus_write_registers(modbus_rtu_station_ctx, 20, 10, station4Register_sent);
        for(i=0;i<10;i++)
        {
          station4Register_sent_previous[i] = station4Register_sent[i];
        }
        STATION4_WRITING = 0;
      }
    }

    if(STATION5_WRITING)
    {
      int issend=0;
      int i;
      for(i=0;i<10;i++)
      {
        if(station5Register_sent[i] != station5Register_sent_previous[i])
        {
          issend = 1;
          station5Register_sent_previous[i] = station5Register_sent[i];
        }
      }
      if(issend == 1)
      {
        // printf("Writing to Station 5: ");
        // int i;
        // for(i=0;i<10;i++)
        // {
        //   printf("%3d", station5Register_sent[i]);
        // }
        // printf("\n");
        modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_WRITE_TIMEOUT_S, STATION_WRITE_TIMEOUT_uS);
        modbus_set_slave(modbus_rtu_station_ctx, 5);
        modbus_write_registers(modbus_rtu_station_ctx, 20, 10, station5Register_sent);
      }
    }
  }
  return NULL; 
}

void *robotThread(void *vargp)
{
  uint16_t position=0, control=0;
  int rc;
  int rewrite = 1;
  int16_t robotRegister_sent_previous[5];
  memset(robotRegister_sent_previous, 0, sizeof(robotRegister_sent_previous));
  uint8_t resend = 0;
  while (1)
  {
    if(
           (robotRegister_received[1] != robotRegister_sent[0])
        || (rewrite == 1)
        || (robotRegister_sent_previous[1] != robotRegister_sent[1]) 
        //|| (robotRegister_sent_previous[2] != robotRegister_sent[2])
      )
    {
      //printf("Write to robot\n");
      //position = robotRegister_sent[0];
      memcpy(robotRegister_sent_previous, robotRegister_sent, sizeof(robotRegister_sent_previous));
      modbus_flush(modbus_rtu_robot_ctx);
      modbus_set_response_timeout(modbus_rtu_robot_ctx, ROBOT_WRITE_TIMEOUT_S, ROBOT_WRITE_TIMEOUT_uS);
      //modbus_set_debug(modbus_rtu_robot_ctx, TRUE);
      rc = modbus_write_registers(modbus_rtu_robot_ctx, 0, 5, robotRegister_sent);
      //modbus_set_debug(modbus_rtu_robot_ctx, FALSE);
      if(rc != 5)
      {
        rewrite = 1;
        //printf("Rewrite to robot!!!\n");
      }
      else
      {
        //printf("[OK] Sending succesffuly!!\n");
        //usleep(200000);
        rewrite = 0;
      }
    }
    else
    {
      //modbus_read_registers(modbus_rtu_robot_ctx, 0, 2, robotRegister_received);
    }
    
    modbus_flush(modbus_rtu_robot_ctx);
    modbus_set_response_timeout(modbus_rtu_robot_ctx, ROBOT_READ_TIMEOUT_S, ROBOT_READ_TIMEOUT_uS);
    rc = modbus_read_registers(modbus_rtu_robot_ctx, 0, 3, robotRegister_received);
    if(rc != -1)
    {
      // if(robotRegister_received[0] == 0)
      // {
      //   robotRegister_sent[2] = robot_status;
      // }
      // else
      // {
      //   robot_status = robotRegister_sent[0];
      // }
      // printf("Stored Robot Location: %d\n", robotRegister_received[0]);

      // int i;
      // printf("Data: ");
      // for(i=0;i<rc;i++)
      // {
      //   printf("%3d", robotRegister_received[i]);
      // }
      // printf("\n");
      //usleep(200000);
    }
    else
    {
      printf("[Error] Read from robot Timeout!\n");
    }
    //printf("Robot Location: %d\n", robot_status);
    if(robotRegister_received[0] == 0)
    {
      robotRegister_sent[2] = robot_status;
    }
    usleep(200000);
  }
  return NULL;
}
void *userInterface(void *vargp)
{
  uint8_t hascalling = 0;
  uint8_t transfer_ok = 1;
  int16_t station1request;
  int16_t station2request;
  int16_t station3request;
  int16_t station4request;
  int16_t station5request;
  int16_t station1control1;
  int16_t station1control2;
  int16_t station2control1;
  int16_t station2control2;
  int16_t station3control1;
  int16_t station3control2;
  int16_t station4control1;
  int16_t station4control2;
  int16_t station5control1;
  int16_t station5control2;

  uint8_t mode = 0;
  while(1)
  {
    if(robot_control == 0)
    {
      hascalling = 0;
      if(STATION1_ENABLE == 1)
      {
        station1request = station1Register_received[0];
        if(station1request != -1)
        {
          if((station1request == 1))
          {
            hascalling = 1;
            if((robotRegister_sent[0] != 1))
            {
              printf("[OK] [%6d] Station 1 requests robot at => %s", ++station1_counter, getTime());
              robotRegister_sent[0] = 1;
              station1_processed = 1;
            }
          }
          else
          {
            if((robotRegister_sent[0] == 1) && (station1_processed == 1))
            {
              station1Register_sent[0]=0;
              station1_processed = 0;
              robotRegister_sent[0]=0;
              printf("[OK] Station 1 has canceled requests at => %s", getTime());
            }
          }
        }
      }

      if(STATION2_ENABLE == 1)
      {
        station2request = station2Register_received[0];

        if((station2request == 1))
        {
          hascalling = 2;
          if((robotRegister_sent[0] != 2))
          {
            printf("[OK] [%6d] Station 2 requests robot at => %s", ++station2_counter, getTime());
            robotRegister_sent[0] = 2; 
            station2_processed=1;
          }
        }
        else
        {
          if((robotRegister_sent[0] == 2) && (station2_processed == 1))
          {
            printf("[OK] Station 2 has canceled requests at => %s", getTime());
            robotRegister_sent[0]=4;
            station2Register_sent[0]=0;
            station2_processed=0;
          }
        }

      }
      
      if(STATION3_ENABLE == 1)
      {
        station3request = station3Register_received[0];

        if(station3request != -1)
        {
          if((station3Register_received[0] == 1))
          {
            hascalling = 3;
            if((robotRegister_sent[0] != 3))
            {
              printf("[OK] [%6d] Station 3 requests robot at => %s", ++station3_counter, getTime());
              robotRegister_sent[0] = 3;
              station3_processed = 1;
            }
          }
          else
          {
            if((robotRegister_sent[0] == 3) && (station3_processed == 1))
            {
              station3Register_sent[0]=0;
              station3_processed = 0;
              robotRegister_sent[0] = 4;
              printf("[OK] Station 3 has canceled requests at => %s", getTime());
            }
          }
        }
      }
      
      if(STATION4_ENABLE == 1)
      {
        station4request = station4Register_received[0];
        if(station4request != -1)
        {
          if((station4request == 1))
          {
            hascalling = 1;
            if((robotRegister_sent[0] != 4))
            {
              printf("[OK] [%6d] Station 4 requests robot at => %s", ++station4_counter, getTime());
              robotRegister_sent[0] = 4;
              station4_processed = 1;
            }
          }
          else
          {
            if((robotRegister_sent[0] == 4) && (station4_processed == 1))
            {
              station4Register_sent[0]=0;
              station4_processed = 0;
              robotRegister_sent[0]=1;
              printf("[OK] Station 4 has canceled requests at => %s", getTime());
            }
          }
        }
      }
      
      if(STATION5_ENABLE == 1)
      {
        station5request = station5Register_received[0];
        if(station5request != -1)
        {
          if((station5request == 1))
          {
            hascalling = 5;
            if((robotRegister_sent[0] != 5))
            {
              printf("[OK] [%6d] Station 5 requests robot at => %s", ++station1_counter, getTime());
              robotRegister_sent[0] = 5;
              station5_processed = 1;
              //station5Register_sent[0]=5;
            }
          }
          else
          {
            if((robotRegister_sent[0] == 5) && (station5_processed == 1))
            {
              station5Register_sent[0]=0;
              station5_processed = 0;
              robotRegister_sent[0]=4;
              printf("[OK] Station 5 has canceled requests at => %s", getTime());
            }
          }
        }
      }
    }
    else
    {
      printf("Robot in Manual Mode under control of the Station 1\n");
      while(robot_control != robotRegister_received[0] && (robot_control != 0))
      {
        printf("Robot is going to station %d under control of Station 1\n", robot_control);
        sleep(1);
      }

      if(robot_control == 0)
      {
        printf("Canceled Requesting\n");
      }
      else
      {
        printf("Robot in Auto Mode at station: %d\n", robot_control);
      }
      robot_control = 0;
      robotRegister_sent[0] = 0;
    }

    int16_t robotlocation = robotRegister_received[0];
    if(robotlocation != 0)
    {
      robot_status = robotlocation;
    }
    switch(robotlocation)
    {
      case 1:
        if(station1_processed == 1)
        {
          station1Register_sent[0]=robotlocation;
          station2Register_sent[0]=0;
          station3Register_sent[0]=0;
          station4Register_sent[0]=0;
          station5Register_sent[0]=0;
          printf("[OK] Robot come to Station %d at %s", robotlocation, getTime());
          while(1)
          {
            station1request = station1Register_received[0];
            station1control1 = station1Register_received[2];
            station1control2 = station1Register_received[3];
            if((station1control1 == 1) && (station1control1 != -1) && (robotRegister_sent[1] != 1))
            {
              printf("%s[WARN] Increased the Carier at station %d\n%s", KYEL, robotlocation, NONE);
              robotRegister_sent[1]=1;
            }
            else if ((station1control2 == 1) && (robotRegister_sent[1] != 0))
            {
              printf("%s[WARN] Decreased the Carier\n%s", KGRN, NONE);
              robotRegister_sent[1]=0;
            }

            if((station1request == 0) && station1request != -1)
            {
              if(robotRegister_sent[1] == 1)
              {
                printf("%s\r[ERROR] Please DECREASE the Carier firstly%s", KRED, NONE);
              }
              else
              {
                printf("[OK] Robot finished at Station %d\n", robotlocation);
                break;
              }
            }
          }
          station1Register_sent[0]=0;
          robotRegister_sent[0]=4;
          station1_processed = 0;
        }
      break;
      case 2:
        if(station2_processed == 1)
        {
          station1Register_sent[0]=0;
          station2Register_sent[0]=robotlocation;
          station3Register_sent[0]=0;
          station4Register_sent[0]=0;
          station5Register_sent[0]=0;
          printf("[OK] Robot come to Station %d at %s", robotlocation, getTime());
          while(1)
          {
            station2request = station2Register_received[0];
            station2control1 = station2Register_received[2];
            station2control2 = station2Register_received[3];
            if((station2control1 == 1) && (robotRegister_sent[1] != 1))
            {
              printf("%s[WARN] Increased the Carier at station %d\n%s", KYEL, robotlocation, NONE);
              robotRegister_sent[1]=1;
            }
            else if ((station2control2 == 1) && (robotRegister_sent[1] != 0))
            {
              printf("%s[WARN] Decreased the Carier\n%s", KGRN, NONE);
              robotRegister_sent[1]=0;
            }
            if((station2request == 0) && station2request != -1)
            {
              if(robotRegister_sent[1] == 1)
              {
                printf("%s\r[WARNING] Please DECREASE the Carier firstly%s", KRED, NONE);
              }
              else
              {
                printf("[OK] Robot finished at Station %d\n", robotlocation);
                break;
              }
            }
          }
          sleep(1);
          station2Register_sent[0]=0;
          robotRegister_sent[0]=4;
          station2_processed = 0;
        }
      break;
      case 3:
        if(station3_processed == 1)
        {
          station1Register_sent[0]=0;
          station2Register_sent[0]=0;
          station3Register_sent[0]=robotlocation;
          station4Register_sent[0]=0;
          station5Register_sent[0]=0;
          printf("[OK] Robot come to Station %d at %s", robotlocation, getTime());
          while(1)
          {
            station3request = station3Register_received[0];
            station3control1 = station3Register_received[2];
            station3control2 = station3Register_received[3];
            if((station3control1 == 1) && (station3control1 != -1) && (robotRegister_sent[1] != 1))
            {
              printf("%s[WARN] Increased the Barier at station %d\n%s", KYEL, robotlocation, NONE);
              robotRegister_sent[1]=1;
            }
            else if ((station3control2 == 1) && (station3control2 != -1) && (robotRegister_sent[1] != 0))
            {
              printf("%s[WARN] Decreased the Barier\n%s", KGRN, NONE);
              robotRegister_sent[1]=0;
            }
            if((station3Register_received[0] == 0) && station3Register_received[0] != -1)
            {
              if(robotRegister_sent[1] == 1)
              {
                printf("%s\r[ERROR] Please DECREASE the Barier firstly%s", KRED, NONE);
              }
              else
              {
                printf("[OK] Robot finished at Station %d\n", robotlocation);
                break;
              }

            }
          }
          sleep(1);
          station3Register_sent[0]=0;
          robotRegister_sent[0]=4;
          station3_processed = 0;
        }
      break;
      case 4:
        if(station4_processed == 1)
        {
          station1Register_sent[0]=0;
          station2Register_sent[0]=0;
          station3Register_sent[0]=0;
          station4Register_sent[0]=robotlocation;
          station5Register_sent[0]=0;
          printf("[OK] Robot come to Station %d at %s", robotlocation, getTime());
          while(1)
          {
            station4request = station4Register_received[0];
            station4control1 = station4Register_received[2];
            station4control2 = station4Register_received[3];
            if((station4control1 == 1) && (station4control1 != -1) && (robotRegister_sent[1] != 1))
            {
              printf("%s[WARN] Increased the Barier at station %d\n%s", KYEL, robotlocation, NONE);
              robotRegister_sent[1]=1;
            }
            else if ((station4control2 == 1) && (station4control2 != -1) && (robotRegister_sent[1] != 0))
            {
              printf("%s[WARN] Decreased the Barier\n%s", KGRN, NONE);
              robotRegister_sent[1]=0;
            }
            if((station4request == 0) && station4request != -1)
            {
              if(robotRegister_sent[1] == 1)
              {
                printf("%s\r[ERROR] Please DECREASE the Barier firstly%s", KRED, NONE);
              }
              else
              {
                printf("[OK] Robot finished at Station %d\n", robotlocation);
                break;
              }

            }
          }
          sleep(1);
          station4Register_sent[0]=0;
          robotRegister_sent[0]=1;
          station4_processed = 0;
        }
      break;
      case 5:
      if(station5_processed == 1)
      {
        station1Register_sent[0]=0;
        station2Register_sent[0]=0;
        station3Register_sent[0]=0;
        station4Register_sent[0]=0;
        station5Register_sent[0]=robotlocation;
        STATION5_WRITING = 1;
        printf("[OK] Robot come to Station %d at %s", robotlocation, getTime());
        while(1)
        {
          station5request=station5Register_received[0];
          station5control1=station5Register_received[2];
          station5control2=station5Register_received[3];
          if((station5control1 == 1) && (robotRegister_sent[1] != 1))
          {
            printf("%s[WARN] Increased the Barier at station %d\n%s", KYEL, robotlocation, NONE);
            robotRegister_sent[1]=1;
          }
          else if ((station5control2 == 1) && (robotRegister_sent[1] != 0))
          {
            printf("%s[WARN] Decreased the Barier\n%s", KGRN, NONE);
            robotRegister_sent[1]=0;
          }
          if((station5request == 0) && station5request != -1)
          {
            if(robotRegister_sent[1] == 1)
            {
              printf("%s\r[ERROR] Please DECREASE the Barier firstly%s", KRED, NONE);
            }
            else
            {
              printf("[OK] Robot finished at Station %d\n", robotlocation);
              break;
            }

          }
        }
        sleep(1);
        station5Register_sent[0]=0;
        robotRegister_sent[0]=4;
        station5_processed = 0;
      }
      break;
      default:
        //printf("[WARNING] Unknow robot location!!!\n");
      break;
    }
    // int i;
    // printf("Robot Data: ");
    // for(i=0;i<3;i++)
    // {
    //   printf("%3d", robotRegister_received[i]);
    // }
    // printf("\n");
    //printf("Robot is in Station: %d\n", robotRegister_received[0]);
  }
}
////////////// Init Common ////////////
///////////////////////////////////////
///////////////////////////////////////
void initThread()
{
  memset(robotRegister_sent, 0, sizeof(robotRegister_sent));
  memset(robotRegister_received, 0, sizeof(robotRegister_received));

  memset(UserCallingBuffer, 0, sizeof(UserCallingBuffer));
  memset(History, 0, sizeof(History));

  memset(station1Register_received, 0, sizeof(station1Register_received));
  memset(station2Register_received, 0, sizeof(station2Register_received));
  memset(station3Register_received, 0, sizeof(station3Register_received));
  memset(station4Register_received, 0, sizeof(station4Register_received));
  memset(station5Register_received, 0, sizeof(station5Register_received));

  memset(station1Register_sent_previous, 0, sizeof(station1Register_sent_previous));
  memset(station2Register_sent_previous, 0, sizeof(station2Register_sent_previous));
  memset(station3Register_sent_previous, 0, sizeof(station3Register_sent_previous));
  memset(station4Register_sent_previous, 0, sizeof(station4Register_sent_previous));
  memset(station5Register_sent_previous, 0, sizeof(station5Register_sent_previous));
}

void robotInit()
{
  modbus_rtu_robot_ctx = modbus_new_rtu(ROBOT_DEVICE, 115200, 'N', 8, 1);
  //modbus_set_debug(modbus_rtu_robot_ctx, TRUE);
  modbus_set_slave(modbus_rtu_robot_ctx, 1);
  modbus_set_response_timeout(modbus_rtu_robot_ctx, 1, 0);
  if(modbus_connect(modbus_rtu_robot_ctx) == -1)
   {
    fprintf(stderr, "Serial connection failed: %s\n", modbus_strerror(errno));
    modbus_free(modbus_rtu_robot_ctx);
   }

  modbus_rtu_robot_mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, 0,
                                 MODBUS_MAX_READ_REGISTERS, 0);
  if(modbus_rtu_robot_mb_mapping == NULL)
   {
     fprintf(stderr, "Failed to allocate the mapping: %s\n",
               modbus_strerror(errno));
    free(modbus_rtu_robot_mb_mapping);
   }
}

void stationInit()
{
  modbus_rtu_station_ctx = modbus_new_rtu(STATION_DEVICE, 4800, 'E', 8, 1);
  //modbus_set_debug(modbus_rtu_station_ctx, TRUE);
  //modbus_set_slave(modbus_rtu_station_ctx, 1);
  //modbus_set_response_timeout(modbus_rtu_station_ctx, 1, 500000);
  if(modbus_connect(modbus_rtu_station_ctx) == -1)
   {
    fprintf(stderr, "Serial connection failed: %s\n", modbus_strerror(errno));
    modbus_free(modbus_rtu_station_ctx);
   }

  modbus_rtu_station_mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, 0,
                                 MODBUS_MAX_READ_REGISTERS, 0);
  if(modbus_rtu_station_mb_mapping == NULL)
   {
     fprintf(stderr, "Failed to allocate the mapping: %s\n",
               modbus_strerror(errno));
    free(modbus_rtu_station_mb_mapping);
   }
}

char* getTime()
{
  time_t current_time;
  char* c_time_string;
  current_time = time(NULL);

  if (current_time == ((time_t)-1))
  {
      fprintf(stderr, "Failure to obtain the current time.\n");
  }
  else
  {
    c_time_string = ctime(&current_time);

    if (c_time_string == NULL)
    {
        fprintf(stderr, "Failure to convert the current time.\n");
    }
  }
  return c_time_string;
}
/*
 * button_was_clicked
 * 
 * event handler called when the button is clicked.
 */
void button_was_clicked (GtkWidget *widget, gpointer gdata)
{
  if(!strcmp(gdata, "ReCall Robot"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), 
                           (GtkCallback) callback1, gdata);
  }
  else if (!strcmp(gdata, "Station 2"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), 
                           (GtkCallback) callback2, gdata);
  }
  else if (!strcmp(gdata, "Station 3"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), 
                           (GtkCallback) callback3, gdata);
  }
  else if (!strcmp(gdata, "Station 4"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), 
                           (GtkCallback) callback4, gdata);
  }
  else if (!strcmp(gdata, "Station 5"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), 
                           (GtkCallback) callback5, gdata);
  }
  else
  {
    printf("Unknow button");
  }

  // switch(data)
  // {

  //   gtk_container_foreach (GTK_CONTAINER (widget), 
  //                          (GtkCallback) callback2, gdata);
  // }
}
void GUIInit(int argc, char *argv[])
{
  GtkWidget *window;
  GtkWidget *table;
  GtkWidget *title;
  GtkWidget *wins;
  
  GtkWidget *halign;
  GtkWidget *halign2;
  GtkWidget *valign;

  GtkWidget *actBtn;
  GtkWidget *clsBtn;
  GtkWidget *hlpBtn;
  GtkWidget *okBtn;
  GtkWidget *actstation1;
  GtkWidget *actstation2;
  GtkWidget *actstation3;
  GtkWidget *actstation4;
  GtkWidget *actstation5;

  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  //gtk_widget_set_size_request (window, 800, 480);
  gtk_window_fullscreen(window);
  //gtk_widget_realize (window);
  //GdkCursor* Cursor = gdk_cursor_new(GDK_BLANK_CURSOR);
  //gdk_window_set_cursor(window, GDK_BLANK_CURSOR);
  gtk_window_set_title(GTK_WINDOW(window), "AGV Robot Controller");
  //gdk_window_set_cursor((window),Cursor);
  gtk_container_set_border_width(GTK_CONTAINER(window), 50);

  table = gtk_table_new(12, 6, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(table), 10);
  gtk_table_set_row_spacing(GTK_TABLE(table), 0, 10);

  title = gtk_label_new("Console: ");
  halign = gtk_alignment_new(0, 0, 0, 0);
  gtk_container_add(GTK_CONTAINER(halign), title);
  gtk_table_attach(GTK_TABLE(table), halign, 0, 1, 0, 1, 
      GTK_FILL, GTK_FILL, 0, 0);

  wins = gtk_text_view_new();
  GtkTextBuffer *consoletext = gtk_text_buffer_new (NULL);
  //gtk_text_buffer_insert_at_cursor (consoletext, "Truong Dinh Nguyen", sizeof("Truong Dinh Nguyen"));
  wins = gtk_text_view_new_with_buffer (consoletext);

  gtk_text_view_set_editable(GTK_TEXT_VIEW(wins), FALSE);
  gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(wins), TRUE);
  gtk_table_attach(GTK_TABLE(table), wins,  0, 2, 1, 3, 
      GTK_FILL | GTK_EXPAND, GTK_FILL | GTK_EXPAND, 1, 1);

  // actBtn = gtk_button_new_with_label("Activate");
  // gtk_widget_set_size_request(actBtn, 70, 30);
  // gtk_table_attach(GTK_TABLE(table), actBtn, 3, 4, 1, 2, 
  //     GTK_FILL, GTK_SHRINK, 1, 1);

  actstation1 = gtk_button_new_with_label("ReCall Robot");
  gtk_widget_set_size_request(actstation1, 70, 30);
  gtk_table_attach_defaults(GTK_TABLE(table), actstation1, 5, 6, 1, 2);
  g_signal_connect (GTK_OBJECT(actstation1), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ReCall Robot");

  actstation2 = gtk_button_new_with_label("Station 2");
  gtk_widget_set_size_request(actstation2, 70, 30);
  gtk_table_attach_defaults(GTK_TABLE(table), actstation2, 6, 7, 1, 2);
  g_signal_connect (GTK_OBJECT(actstation2), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 2");

  actstation3 = gtk_button_new_with_label("Station 3");
  gtk_widget_set_size_request(actstation3, 70, 30);
  gtk_table_attach_defaults(GTK_TABLE(table), actstation3, 7, 8, 1, 2);
  g_signal_connect (GTK_OBJECT(actstation3), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 3");

  actstation4 = gtk_button_new_with_label("Station 4");
  gtk_widget_set_size_request(actstation4, 70, 30);
  gtk_table_attach_defaults(GTK_TABLE(table), actstation4, 8, 9, 1, 2);
  g_signal_connect (GTK_OBJECT(actstation4), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 4");

  actstation5 = gtk_button_new_with_label("Station 5");
  gtk_table_attach_defaults(GTK_TABLE(table), actstation5, 9, 10, 1, 2);
  g_signal_connect (GTK_OBJECT(actstation5), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 5");

  // actstation1 = gtk_button_new_with_label("Station 1");
  // gtk_widget_set_size_request(actstation1, 70, 30);
  // gtk_table_attach(GTK_TABLE(table), actstation1, 2, 3, 0, 1, 
  //     GTK_FILL, GTK_SHRINK, 1, 1);

  // actstation2 = gtk_button_new_with_label("Station 2");
  // gtk_widget_set_size_request(actstation2, 70, 30);
  // gtk_table_attach(GTK_TABLE(table), actstation2, 2, 3, 0, 2, 
  //     GTK_FILL, GTK_SHRINK, 1, 1);

  // actstation3 = gtk_button_new_with_label("Station 3");
  // gtk_widget_set_size_request(actstation3, 70, 30);
  // gtk_table_attach(GTK_TABLE(table), actstation3, 2, 3, 1, 6, 
  //     GTK_FILL, GTK_SHRINK, 1, 1);

  // valign = gtk_alignment_new(0, 0, 0, 0);
  // clsBtn = gtk_button_new_with_label("Close");
  
  // gtk_widget_set_size_request(clsBtn, 50, 30);
  // gtk_container_add(GTK_CONTAINER(valign), clsBtn);
  // gtk_table_set_row_spacing(GTK_TABLE(table), 1, 3);
  // gtk_table_attach(GTK_TABLE(table), valign, 3, 4, 2, 3, 
  //     GTK_FILL, GTK_FILL | GTK_EXPAND, 1, 1);

  halign2 = gtk_alignment_new(0, 1, 0, 0);
  hlpBtn = gtk_button_new_with_label("Help");
  gtk_container_add(GTK_CONTAINER(halign2), hlpBtn);
  gtk_widget_set_size_request(hlpBtn, 70, 30);
  gtk_table_set_row_spacing(GTK_TABLE(table), 3, 5);
  gtk_table_attach(GTK_TABLE(table), halign2, 0, 1, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  // okBtn = gtk_button_new_with_label("OK");
  // gtk_widget_set_size_request(okBtn, 70, 30);
  // gtk_table_attach(GTK_TABLE(table), okBtn, 3, 4, 4, 5, 
  //     GTK_FILL, GTK_FILL, 0, 0);

  gtk_container_add(GTK_CONTAINER(window), table);

  g_signal_connect(G_OBJECT(window), "destroy",
        G_CALLBACK(gtk_main_quit), G_OBJECT(window));

  gtk_widget_show_all(window);
  gtk_main();

}

/* Our usual callback function */
static void callback1( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    robotRegister_sent[0] = 0;
    robot_control = 0;
    gtk_label_set (GTK_LABEL(widget), "ReCall Robot");
  }
  else
  {
    gtk_label_set (GTK_LABEL(widget), "Calling");
    robotRegister_sent[0] = 1;
    robot_control = 1;
  }
  //gtk_label_get(GTK_LABEL(widget), button_label);
  g_print ("%s was pressed, %s\n", (char *) data, (char*) button_label);

}

/* Our usual callback function */
static void callback2( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    robotRegister_sent[0] = 0;
    robot_control = 0;
    gtk_label_set (GTK_LABEL(widget), "Station 2");
  }
  else
  {
    gtk_label_set (GTK_LABEL(widget), "Calling");
    robotRegister_sent[0] = 2;
    robot_control = 2;
  }
}

/* Our usual callback function */
static void callback3( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    robotRegister_sent[0] = 0;
    robot_control = 0;
    gtk_label_set (GTK_LABEL(widget), "Station 3");
  }
  else
  {
    gtk_label_set (GTK_LABEL(widget), "Calling");
    robotRegister_sent[0] = 3;
    robot_control = 3;
  }
}

/* Our usual callback function */
static void callback4( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    robotRegister_sent[0] = 0;
    robot_control = 0;
    gtk_label_set (GTK_LABEL(widget), "Station 4");
  }
  else
  {
    gtk_label_set (GTK_LABEL(widget), "Calling");
    robotRegister_sent[0] = 4;
    robot_control = 4;
  }
}
/* Our usual callback function */
static void callback5( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    robotRegister_sent[0] = 0;
    robot_control = 0;
    gtk_label_set (GTK_LABEL(widget), "Station 5");
  }
  else
  {
    gtk_label_set (GTK_LABEL(widget), "Calling");
    robotRegister_sent[0] = 5;
    robot_control = 5;
  }
}

