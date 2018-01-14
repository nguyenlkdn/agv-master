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
//#define RobotModbus_DEBUG
//#define Station1Modbus_DEBUG

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

*/
int16_t stationRead_reg[STATION_MAX][6];
int16_t stationWrite_reg[STATION_MAX][6];

#define font "Sans 60"
PangoFontDescription *font_desc;
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

uint8_t callingallowed = 0;
uint8_t station1allowedfrommaster = 0;
uint8_t station2allowedfrommaster = 0;
uint8_t station3allowedfrommaster = 0;
uint8_t station4allowedfrommaster = 0;
uint8_t station5allowedfrommaster = 0;

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
uint16_t robot_read_err = 0;
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
uint16_t robotworking = 0;
uint16_t station1_confimation=0;
uint16_t station2_confimation=0;
uint16_t station3_confimation=0;
uint16_t station4_confimation=0;
uint16_t station5_confimation=0;

uint16_t station1_isaccepted=1;
uint16_t station2_isaccepted=1;
uint16_t station3_isaccepted=1;
uint16_t station4_isaccepted=1;
uint16_t station5_isaccepted=1;

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

uint16_t staion4_locking = 0;
uint16_t stored_robot_location = 0;

char TEXT[255];
uint16_t stored_request=0;
/*

*/
GtkTextBuffer *consoletxt;
GtkTextIter iter;
GtkWidget *wins;

/*

*/
void GUIInit(int argc, char *argv[]);
void *stationThread(void *vargp);
void *robotThread(void *vargp);
void *userThread();
void *userInterface(void *vargp);
void initThread();
void robotInit();
void stationInit();
char *getTime();
void printtoconsole(char* text);
void stationresponding(uint16_t id);
/*

*/
static void callback1( GtkWidget *widget, gpointer data);
static void callback2( GtkWidget *widget, gpointer data);
static void callback3( GtkWidget *widget, gpointer data);
static void callback4( GtkWidget *widget, gpointer data);
static void callback5( GtkWidget *widget, gpointer data);
static void callback6( GtkWidget *widget, gpointer data);

static void recallback1( GtkWidget *widget, gpointer data);
static void recallback2( GtkWidget *widget, gpointer data);
static void recallback3( GtkWidget *widget, gpointer data);
static void recallback4( GtkWidget *widget, gpointer data);
static void recallback5( GtkWidget *widget, gpointer data);
static void recallback6( GtkWidget *widget, gpointer data);
static void allowcallinghandler( GtkWidget *widget, gpointer data);
/*

*/
modbus_t *ctx;
modbus_t *modbus_rtu_robot_ctx;
modbus_t *modbus_rtu_station_ctx;
modbus_t *modbus_rtu_station_zigbee_ctx;
modbus_mapping_t *mb_mapping;
modbus_mapping_t *modbus_rtu_robot_mb_mapping;
modbus_mapping_t *modbus_rtu_station_mb_mapping;
modbus_mapping_t *modbus_rtu_station_zigbee_mb_mapping;
modbus_mapping_t *modbus_rtu_station1_mb_mapping;
modbus_mapping_t *modbus_rtu_station2_mb_mapping;
modbus_mapping_t *modbus_rtu_station3_mb_mapping;
modbus_mapping_t *modbus_rtu_station4_mb_mapping;
modbus_mapping_t *modbus_rtu_station5_mb_mapping;

GtkWidget *btnstation1;
GtkWidget *actstation1;
GtkWidget *actstation2;
GtkWidget *actstation3;
GtkWidget *actstation4;
GtkWidget *actstation5;
GtkWidget *actstation6;
GtkWidget *actstation7;
GtkWidget *actstation8;
GtkWidget *actstation9;
GtkWidget *actstation10;
GtkWidget *btnallowcalling;

GtkWidget *image;
int16_t robot_status = 0;
int16_t robot_sensor = 0;
uint16_t robot_control = 0;

GtkWidget *colorseldlg = NULL;
GtkWidget *drawingarea = NULL;
GdkColor color;

GtkWidget *robotbattery;
GtkWidget *robotlocation;
GtkWidget *robotstatus;
GtkWidget *robotconnection;
GtkWidget *robotspeedmax;

GtkWidget *station1status;
GtkWidget *station2status;
GtkWidget *station3status;
GtkWidget *station4status;
GtkWidget *station5status;
GtkWidget *station6status;
GtkWidget *station7status;
GtkWidget *station8status;
GtkWidget *station9status;
GtkWidget *station10status;



static void color_changed_cb( GtkWidget         *widget,
                              GtkColorSelection *colorsel )
{
  GdkColor ncolor;

  gtk_color_selection_get_current_color (colorsel, &ncolor);
  gtk_widget_modify_bg (drawingarea, GTK_STATE_NORMAL, &ncolor);       
}

static gboolean area_event( GtkWidget *widget,
                            GdkEvent  *event,
                            gpointer   client_data )
{
  gint handled = FALSE;
  gint response;
  GtkColorSelection *colorsel;

  /* Check if we've received a button pressed event */

  if (event->type == GDK_BUTTON_PRESS)
    {
      handled = TRUE;

       /* Create color selection dialog */
      if (colorseldlg == NULL)
        colorseldlg = gtk_color_selection_dialog_new ("Select background color");

      /* Get the ColorSelection widget */
      colorsel = GTK_COLOR_SELECTION (GTK_COLOR_SELECTION_DIALOG (colorseldlg)->colorsel);

      gtk_color_selection_set_previous_color (colorsel, &color);
      gtk_color_selection_set_current_color (colorsel, &color);
      gtk_color_selection_set_has_palette (colorsel, TRUE);

      /* Connect to the "color_changed" signal, set the client-data
       * to the colorsel widget */
      g_signal_connect (colorsel, "color_changed",
                        G_CALLBACK (color_changed_cb), (gpointer) colorsel);

      /* Show the dialog */
      response = gtk_dialog_run (GTK_DIALOG (colorseldlg));

      if (response == GTK_RESPONSE_OK)
        gtk_color_selection_get_current_color (colorsel, &color);
      else 
        gtk_widget_modify_bg (drawingarea, GTK_STATE_NORMAL, &color);

      gtk_widget_hide (colorseldlg);
    }

  return handled;
}

void RequestingProcess(void);
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
  pthread_create(&stationThread_id, NULL, stationThread, NULL);
  //pthread_join(robotThread_id, NULL);
  pthread_join(userInterface_id, NULL);
  pthread_join(userThread_id, NULL);
  pthread_join(stationThread_id, NULL);
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
  int32_t rc;
  while (1)
  {
    /*
      List of station will be ignored
    */
    DEBUG_PRINT("Ignored %d: ", STATION_MAX);
    for(rc=1;rc<=STATION_MAX;rc++)
    {
      stationid = rc;
      if(stationignore[rc] == 0)
      {
        DEBUG_PRINT("%3d", rc);
      }
    }
    DEBUG_PRINT("\n");
    ////////////////////////////////////
    /*
      Reading Stations
    */
    int32_t stationscan;
    for(stationscan=STATION_START;stationscan<=STATION_MAX;stationscan++)
    {
      if(stationignore[stationscan] == 1)
      {
        modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
        modbus_set_slave(modbus_rtu_station_ctx, stationscan);
        //modbus_flush(modbus_rtu_station_ctx);
        //modbus_set_debug(modbus_rtu_station_ctx, TRUE);
        ////////////// Reading //////////////////////
        rc = modbus_read_registers(modbus_rtu_station_ctx, 0, 5, stationRead_reg[stationscan]);
        if(rc == -1)
        {
          memset(stationRead_reg[stationscan], -1, sizeof(stationRead_reg[stationscan]));
          DEBUG_PRINT("STATION %d: READ TIMEOUT!\n", stationscan);
        }
        else
        {
          int i;
          DEBUG_PRINT("[%d] STATION %d READs OK, with Data: ", rc, stationscan);
          for(i=0;i<5;i++)
          {
            DEBUG_PRINT("%6d", stationRead_reg[stationscan][i], i);
          }
          DEBUG_PRINT("\n");
          usleep(500000);
          ///////////////// Writing ////////////////////
          rc = modbus_write_registers(modbus_rtu_station_ctx, 0, 5, stationWrite_reg[stationscan]);
          if(rc == -1)
          {
            DEBUG_PRINT("STATION %d: WRITE TIMEOUT!\n", stationscan);
          }
          else
          {
            DEBUG_PRINT("STATION %d: WRITE OK!\n", stationscan);
          }
        }
      }
    }
    ////////////////////////////////////
    /*
      Checking Calling
    */
    for(stationscan=STATION_START;stationscan<=STATION_MAX;stationscan++)
    {
      if(stationRead_reg[stationscan][0] == 1)
      {
        printf("%d HAS Requeting\n", stationscan);
        stationresponding(stationscan);
      }
    }
    ////////////////////////////////////
    usleep(500000);
  }
  return NULL; 
}

void *robotThread(void *vargp)
{
  uint16_t position=0, control=0;
  int rc;
  int rewrite = 1;
  int16_t robotRegister_sent_previous[5];
  memset(robotRegister_sent_previous, -1, sizeof(robotRegister_sent_previous));
  uint8_t resend = 1;
  while (1)
  {
    for(rc=0;rc<5;rc++)
    {
      if(robotRegister_sent_previous[rc] != robotRegister_sent[rc])
      {
        printf("Has new packages so that writing to robot\n");
        resend = 1;
        break;
      }
    }

    if(robotRegister_received[1] != robotRegister_sent[0])
    {
      resend = 1;
    }

    if(
      ((resend == 1) || (rewrite == 1))
      )
    {
      printf("Write to robot: %d %d %d %d %d\n", robotRegister_sent[0], robotRegister_sent[1], robotRegister_sent[2], robotRegister_sent[3], robotRegister_sent[4], robotRegister_sent[5]);
      modbus_flush(modbus_rtu_robot_ctx);
      modbus_set_response_timeout(modbus_rtu_robot_ctx, ROBOT_WRITE_TIMEOUT_S, ROBOT_WRITE_TIMEOUT_uS);
      //modbus_set_debug(modbus_rtu_robot_ctx, TRUE);
      rc = modbus_write_registers(modbus_rtu_robot_ctx, 0, 5, robotRegister_sent);
      //modbus_set_debug(modbus_rtu_robot_ctx, FALSE);
      if(rc != 5)
      {
        rewrite = 1;
        printf("Robot Writing Failed so that re-writing\n");
      }
      else
      {
        memcpy(robotRegister_sent_previous, robotRegister_sent, sizeof(robotRegister_sent_previous));
        printf("Robot Writing: OK\n");
        //robotRegister_sent[2] = 0;
        rewrite = 0;
        resend = 0;
        usleep(200000);
      }
    }

    modbus_flush(modbus_rtu_robot_ctx);
    modbus_set_response_timeout(modbus_rtu_robot_ctx, ROBOT_READ_TIMEOUT_S, ROBOT_READ_TIMEOUT_uS);
    //modbus_set_debug(modbus_rtu_robot_ctx, TRUE);
    rc = modbus_read_registers(modbus_rtu_robot_ctx, 0, 3, robotRegister_received);
    //modbus_set_debug(modbus_rtu_robot_ctx, FALSE);
    if(rc != -1)
    {
      #ifdef RobotModbus_DEBUG
        printf("Robot Reading: OK\n");
      #endif
      usleep(200000);
    }
    else
    {
      robot_read_err++;
      printf("Robot Reading: Timeout %d\n", rc);
    }
    snprintf(TEXT, sizeof(TEXT), "%d", robotRegister_received[0]);
    gtk_entry_set_text (GTK_ENTRY (robotlocation), TEXT);

    snprintf(TEXT, sizeof(TEXT), "%d", robotRegister_received[2]);
    gtk_entry_set_text (GTK_ENTRY (robotstatus), TEXT);


    if(robotRegister_received[0] == 0)
    {
      robotRegister_sent[2] = robot_status;
      robotRegister_sent[3] = robot_sensor;
      printf("Robot was reseted\n");
    }
    else
    {
      robot_status = robotRegister_received[0];
      if(robotRegister_received[2] != 3)
      {
        robot_sensor = robotRegister_received[2];
      }
      robotRegister_sent[2] = 0;
    }

    if(robotRegister_received[0] == 0)
    {
      station1_isaccepted = 0;
      station2_isaccepted = 1;
      station3_isaccepted = 1;
      station4_isaccepted = 0;
      station5_isaccepted = 1;

    }
    else
    {
      if(robotRegister_received[0] == 1)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation1), 
                               (GtkCallback) recallback1, "Recall Robot");
        if(callingallowed == 0)
        {
          station1_isaccepted = 0;
          station2_isaccepted = 0;
          station3_isaccepted = 0;
          station4_isaccepted = 0;
          station5_isaccepted = 0;
        }
        else
        {
          if(callingallowed == 1)
          {
            station1_isaccepted = 0;
            station2_isaccepted = 1;
            station3_isaccepted = 1;
            station4_isaccepted = 0;
            station5_isaccepted = 1;
          }
          else if (callingallowed == 2)
          {
            station2_isaccepted = 1;

          }
          else if (callingallowed == 3)
          {
            station3_isaccepted = 1;
            station3allowedfrommaster = 1;
          }
          else if (callingallowed == 4)
          {
            station4_isaccepted = 1;

          }
          else if (callingallowed == 5)
          {
            station5_isaccepted = 1;
          }
          //callingallowed = 0;
        }

        //gtk_label_set (GTK_LABEL(actstation1), "Recall Robot");
      }
      else if(robotRegister_received[0] == 2)
      {
        station1_isaccepted = 0;
        station2_isaccepted = 0;
        station3_isaccepted = 0;
        station4_isaccepted = 0;
        station5_isaccepted = 0;
        gtk_container_foreach (GTK_CONTAINER (actstation2), 
                               (GtkCallback) recallback2, "Station 2");
      }
      else if(robotRegister_received[0] == 3)
      {
        if(callingallowed == 0)
        {
          station1_isaccepted = 0;
          station2_isaccepted = 1;
          station3_isaccepted = 0;
          station4_isaccepted = 0;
          station5_isaccepted = 0;
        }

        gtk_container_foreach (GTK_CONTAINER (actstation3), 
                               (GtkCallback) recallback3, "Station 3");
      }
      else if(robotRegister_received[0] == 4)
      {
        station1_isaccepted = 0;
        station2_isaccepted = 0;
        station3_isaccepted = 0;
        station4_isaccepted = 0;
        station5_isaccepted = 0;
        if(robotRegister_sent[0] == 4)
        {
          station4Register_sent[0] = 4;
        }
        else
        {
          station4Register_sent[0] = 0;
        }
        gtk_container_foreach (GTK_CONTAINER (actstation4), 
                               (GtkCallback) recallback4, "Station 4");
      }
      else if(robotRegister_received[0] == 5)
      {
        if(callingallowed == 0)
        {
          station1_isaccepted = 0;
          station2_isaccepted = 1;
          station3_isaccepted = 1;
          station4_isaccepted = 0;
          station5_isaccepted = 0;
        }

        gtk_container_foreach (GTK_CONTAINER (actstation5), 
                               (GtkCallback) recallback5, "Station 5");
      }
    }

    usleep(500000);
  }
  return NULL;
}
void *userInterface(void *vargp)
{
  uint8_t hascalling = 0;
  uint8_t transfer_ok = 1;

  uint8_t mode = 0;
  hascalling = 0;
  uint16_t robot_loc = 0;
  while(1)
  {
    if(robot_control == 0)
    {
      RequestingProcess();
      ControllProcess();
    }
    else
    {
      printf("Robot in Manual Mode under control of the Station 1\n");
      stored_robot_location = robot_control;
      uint8_t hasnew_calling = 0;
      uint8_t robotstatus = 0;
      while(robot_control != robotRegister_received[0] && (robot_control != 0))
      {
        //printf("Robot is going to station %d/%d under control of Station 1\n", robot_control, robotRegister_received[0]);
        RequestingProcess();
        ControllProcess();
      }
      printf("Finished with master\n");
      if(robot_control == 0)
      {
        printf("Master was canceled requesting\n");
        robotRegister_sent[0] = 1;
      }
      else
      {
        robot_control = 0;
      }
    }

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

  memset(station1Register_sent_previous, -1, sizeof(station1Register_sent_previous));
  memset(station2Register_sent_previous, -1, sizeof(station2Register_sent_previous));
  memset(station3Register_sent_previous, -1, sizeof(station3Register_sent_previous));
  memset(station4Register_sent_previous, -1, sizeof(station4Register_sent_previous));
  memset(station5Register_sent_previous, -1, sizeof(station5Register_sent_previous));
}

void robotInit()
{
  modbus_rtu_robot_ctx = modbus_new_rtu(ROBOT_DEVICE, 115200, 'N', 8, 1);
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
  modbus_rtu_station_ctx = modbus_new_rtu(STATION_DEVICE, 38400, 'N', 8, 1);
  if(modbus_connect(modbus_rtu_station_ctx) == -1)
   {
    fprintf(stderr, "%s connection failed: %s\n", STATION_DEVICE, modbus_strerror(errno));
    modbus_free(modbus_rtu_station_ctx);
    return;
   }
  modbus_rtu_station_mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, 0,
                                 MODBUS_MAX_READ_REGISTERS, 0);
  if(modbus_rtu_station_mb_mapping == NULL)
   {
     fprintf(stderr, "Failed to allocate the mapping: %s\n",
               modbus_strerror(errno));
    free(modbus_rtu_station_mb_mapping);
   }
   memset(stationignore, 0, sizeof(stationignore));
   stationignore[12]=1;
   int i;
   for(i=0;i<STATION_MAX;i++)
   {
    memset(stationRead_reg[i], -1, 5);
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
  stored_request = robotRegister_sent[0];
  if(!strcmp(gdata, "Reset Stations"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), (GtkCallback) callback6, gdata);
    return;
  }

  if(robotworking>0)
  {
    snprintf(TEXT, sizeof(TEXT), "Robot is BUSY at station %d, please wait a while!\n", robotworking);
    printtoconsole(TEXT);
    return;
  }

  if(!strcmp(gdata, "Recall Robot"))
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
  else if (!strcmp(gdata, "Calling Denied"))
  {
    gtk_container_foreach (GTK_CONTAINER (widget), 
                           (GtkCallback) allowcallinghandler, gdata);
  }
  else
  {

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
  
  GtkWidget *halign;
  GtkWidget *halign2;
  GtkWidget *valign;

  GtkWidget *actBtn;
  GtkWidget *clsBtn;
  GtkWidget *hlpBtn;
  GtkWidget *okBtn;

  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    // drawingarea = gtk_drawing_area_new ();

    // color.red = 0;
    // color.blue = 65535;
    // color.green = 0;
    // gtk_widget_modify_bg (drawingarea, GTK_STATE_NORMAL, &color);       

    // //gtk_widget_set_size_request (GTK_WIDGET (drawingarea), 200, 200);

    // gtk_widget_set_events (drawingarea, GDK_BUTTON_PRESS_MASK);

    // g_signal_connect (GTK_OBJECT (drawingarea), "event", 
    //             GTK_SIGNAL_FUNC (area_event), (gpointer) drawingarea);
    
    // /* Add drawingarea to window, then show them both */

    // gtk_container_add (GTK_CONTAINER (window), drawingarea);

    // // gtk_widget_show (drawingarea);
    // // gtk_widget_show (window);

  //gtk_widget_set_size_request (window, 720, 480);
  if(isfullscreen == 1)
  {
    gtk_window_fullscreen(GTK_WINDOW(window));
  }
  PangoFontDescription *df;

  df = pango_font_description_from_string("Monospace");

  pango_font_description_set_size(df,20*PANGO_SCALE);

  gtk_window_set_title(GTK_WINDOW(window), "AGV Robot Controller");
  gtk_container_set_border_width(GTK_CONTAINER(window), 12);
  GdkColor red = {0x0000, 47575, 65535, 64858};
  gtk_widget_modify_bg(GTK_CONTAINER(window), GTK_STATE_NORMAL, &red);

  table = gtk_table_new(10, 8, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(table), 2);
  gtk_table_set_row_spacing(GTK_TABLE(table), 0, 2);

  wins = gtk_text_view_new();
  //gtk_widget_set_size_request(table, 1920, 1080);

  consoletxt = gtk_text_view_get_buffer(GTK_TEXT_VIEW(wins));
  gtk_text_buffer_create_tag(consoletxt, "gap",
        "pixels_above_lines", 30, NULL);
  gtk_text_buffer_create_tag(consoletxt, "lmarg", 
      "left_margin", 5, NULL);
  gtk_text_buffer_create_tag(consoletxt, "blue_fg", 
      "foreground", "blue", NULL); 
  gtk_text_buffer_create_tag(consoletxt, "gray_bg", 
      "background", "gray", NULL); 
  gtk_text_buffer_create_tag(consoletxt, "italic", 
      "style", PANGO_STYLE_ITALIC, NULL);
  gtk_text_buffer_create_tag(consoletxt, "bold", 
      "weight", PANGO_WEIGHT_BOLD, NULL);
  gtk_text_buffer_get_iter_at_offset(consoletxt, &iter, 0);

  title = gtk_label_new("Console Logs ");
  gtk_widget_modify_font(title, df);
  halign = gtk_alignment_new(0, 0, 0, 0);
  gtk_container_add(GTK_CONTAINER(halign), title);
  gtk_table_attach(GTK_TABLE(table), halign, 1, 3, 9, 10, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Status ");
  gtk_widget_modify_font(title, df);
  gtk_table_attach(GTK_TABLE(table), title, 3, 5, 2, 3, 
      GTK_FILL, GTK_FILL, 0, 0);

  // Robot Status labels
  ////////////////////////////////////
  title = gtk_label_new("Robot Battery: ");
  //gtk_widget_modify_font(title, df);
  gtk_widget_set_size_request(title, 150, 50);
  gtk_table_attach(GTK_TABLE(table), title, 3, 4, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotbattery = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotbattery), 50);
  gtk_entry_set_text (GTK_ENTRY (robotbattery), "none");
  gtk_table_attach(GTK_TABLE(table), robotbattery, 4, 5, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Location: ");
  gtk_table_attach(GTK_TABLE(table), title, 3, 4, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotlocation = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotlocation), 50);
  gtk_entry_set_text (GTK_ENTRY (robotlocation), "none");
  gtk_table_attach(GTK_TABLE(table), robotlocation, 4, 5, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Status: ");
  gtk_table_attach(GTK_TABLE(table), title, 3, 4, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotstatus = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotstatus), 50);
  gtk_entry_set_text (GTK_ENTRY (robotstatus), "none");
  gtk_table_attach(GTK_TABLE(table), robotstatus, 4, 5, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Connection: ");
  gtk_table_attach(GTK_TABLE(table), title, 3, 4, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotconnection = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotconnection), 50);
  gtk_entry_set_text (GTK_ENTRY (robotconnection), "none");
  gtk_table_attach(GTK_TABLE(table), robotconnection, 4, 5, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Speed Max: ");
  gtk_table_attach(GTK_TABLE(table), title, 3, 4, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotspeedmax = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotspeedmax), 50);
  gtk_entry_set_text (GTK_ENTRY (robotspeedmax), "none");
  gtk_table_attach(GTK_TABLE(table), robotspeedmax, 4, 5, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station Status ");
  gtk_widget_modify_font(title, df);
  //halign = gtk_alignment_new(0, 0, 0, 0);
  //gtk_container_add(GTK_CONTAINER(halign), title);
  gtk_table_attach(GTK_TABLE(table), title, 5, 9, 2, 3, 
      GTK_FILL, GTK_FILL, 0, 0);

  // station Status labels
  ////////////////////////////////////
  title = gtk_label_new("   Station 1: ");
  gtk_table_attach(GTK_TABLE(table), title, 5, 6, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  station1status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station1status), 50);
  gtk_entry_set_text (GTK_ENTRY (station1status), "none");
  gtk_table_attach(GTK_TABLE(table), station1status, 6, 7, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 2: ");
  gtk_table_attach(GTK_TABLE(table), title, 7, 8, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);
  station2status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station2status), 50);
  gtk_entry_set_text (GTK_ENTRY (station2status), "none");
  gtk_table_attach(GTK_TABLE(table), station2status, 8, 9, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 3: ");
  gtk_table_attach(GTK_TABLE(table), title, 5, 6, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  station3status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station3status), 50);
  gtk_entry_set_text (GTK_ENTRY (station3status), "none");
  gtk_table_attach(GTK_TABLE(table), station3status, 6, 7, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 4: ");
  gtk_table_attach(GTK_TABLE(table), title, 7, 8, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  station4status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station4status), 50);
  gtk_entry_set_text (GTK_ENTRY (station4status), "none");
  gtk_table_attach(GTK_TABLE(table), station4status, 8, 9, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 5: ");
  gtk_table_attach(GTK_TABLE(table), title, 5, 6, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  station5status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station5status), 50);
  gtk_entry_set_text (GTK_ENTRY (station5status), "none");
  gtk_table_attach(GTK_TABLE(table), station5status, 6, 7, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 6: ");
  gtk_table_attach(GTK_TABLE(table), title, 7, 8, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  station6status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station6status), 50);
  gtk_entry_set_text (GTK_ENTRY (station6status), "none");
  gtk_table_attach(GTK_TABLE(table), station6status, 8, 9, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 7: ");
  gtk_table_attach(GTK_TABLE(table), title, 5, 6, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  station7status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station7status), 50);
  gtk_entry_set_text (GTK_ENTRY (station7status), "none");
  gtk_table_attach(GTK_TABLE(table), station7status, 6, 7, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 8: ");
  gtk_table_attach(GTK_TABLE(table), title, 7, 8, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  station8status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station8status), 50);
  gtk_entry_set_text (GTK_ENTRY (station8status), "none");
  gtk_table_attach(GTK_TABLE(table), station8status, 8, 9, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 9: ");
  gtk_table_attach(GTK_TABLE(table), title, 5, 6, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  station9status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station9status), 50);
  gtk_entry_set_text (GTK_ENTRY (station9status), "none");
  gtk_table_attach(GTK_TABLE(table), station9status, 6, 7, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 10: ");
  gtk_table_attach(GTK_TABLE(table), title, 7, 8, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  station10status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station10status), 50);
  gtk_entry_set_text (GTK_ENTRY (station10status), "none");
  gtk_table_attach(GTK_TABLE(table), station10status, 8, 9, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);
  // g_signal_connect (robotbattery, "activate",
  //       G_CALLBACK (enter_callback),
  //       robotbattery);
  ////////////////////////////////////



  gtk_text_view_set_editable(GTK_TEXT_VIEW(wins), FALSE);
  gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(wins), FALSE);
  gtk_widget_set_size_request(wins, 400, 400);
  gtk_table_attach(GTK_TABLE(table), wins,        1, 3, 10, 11, 
      GTK_FILL | GTK_SHRINK, GTK_FILL | GTK_SHRINK, 1, 1);

  title = gtk_label_new("Control Panel");
  gtk_widget_modify_font(title, df);

  halign = gtk_alignment_new(0, 0, 0, 0);
  gtk_container_add(GTK_CONTAINER(halign), title);
  gtk_table_attach(GTK_TABLE(table), halign, 1, 3, 1, 2, 
      GTK_FILL, GTK_FILL, 0, 0);

  btnstation1 = gtk_button_new_with_label("Station 1");
  gtk_widget_set_size_request(btnstation1, 300, 100);
  gtk_widget_modify_font(btnstation1, df);

  gtk_table_attach(GTK_TABLE(table), btnstation1, 1, 2, 2, 3, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(btnstation1), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 1");

  actstation3 = gtk_button_new_with_label("Station 3");
  gtk_widget_set_size_request(actstation3, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation3, 1, 2, 3, 4, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation3), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 3");

  actstation5 = gtk_button_new_with_label("Station 5");
  gtk_widget_set_size_request(actstation5, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation5, 1, 2, 4, 5, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation5), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 5");

  actstation7 = gtk_button_new_with_label("Station 7");
  gtk_widget_set_size_request(actstation7, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation7, 1, 2, 5, 6, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation7), "clicked",
          G_CALLBACK (area_event), (gpointer) "Station 7");

  actstation9 = gtk_button_new_with_label("Station 9");
  gtk_widget_set_size_request(actstation9, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation9, 1, 2, 6, 7, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation9), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 9");

  actstation2 = gtk_button_new_with_label("Station 2");
  gtk_widget_set_size_request(actstation2, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation2, 2, 3, 2, 3, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation2), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 2");

  actstation4 = gtk_button_new_with_label("Station 4");
  gtk_widget_set_size_request(actstation4, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation4, 2, 3, 3, 4, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation4), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 4");

  actstation6 = gtk_button_new_with_label("Station 6");
  gtk_widget_set_size_request(actstation6, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation6, 2, 3, 4, 5, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation6), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 6");

  actstation8 = gtk_button_new_with_label("Station 8");
  gtk_widget_set_size_request(actstation8, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation8, 2, 3, 5, 6, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation8), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 8");

  actstation10 = gtk_button_new_with_label("Station 10");
  gtk_widget_set_size_request(actstation10, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation10, 2, 3, 6, 7, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation10), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Station 10");

  actstation1 = gtk_button_new_with_label("Recall Robot");
  gtk_widget_set_size_request(actstation1, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation1, 1, 3, 7, 8, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation1), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Recall Robot");

  btnallowcalling = gtk_button_new_with_label("Get yarn dyed");
  gtk_widget_set_size_request(btnallowcalling, 300, 100);
  gtk_table_attach(GTK_TABLE(table), btnallowcalling, 1, 3, 8, 9, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(btnallowcalling), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "Calling Denied");

  /*
    Robot Status
  */
  image = gtk_image_new ();
  gtk_image_set_from_file (GTK_IMAGE(image), "/home/agv/coast.png");
  halign = gtk_alignment_new(0, 0, 0, 0);
  gtk_container_add(GTK_CONTAINER(halign), image);
  //column 1
  gtk_table_attach(GTK_TABLE(table), halign, 4, 9, 8, 11,
          GTK_FILL, GTK_FILL, 0, 0);


  // actstation2 = gtk_button_new_with_label("Station 2");
  // gtk_widget_set_size_request(actstation2, 100, 100);
  // gtk_table_attach(GTK_TABLE(table), actstation2, 90, 91, 1, 2, 
  //         GTK_FILL, GTK_FILL, 0, 0);
  // g_signal_connect (GTK_OBJECT(actstation2), "clicked",
  //         G_CALLBACK (button_was_clicked), (gpointer) "Station 2");

  // actstation3 = gtk_button_new_with_label("Station 3");
  // gtk_widget_set_size_request(actstation3, 100, 100);
  // gtk_table_attach(GTK_TABLE(table), actstation3, 91, 92, 1, 2, 
  //         GTK_FILL, GTK_FILL, 0, 0);
  // g_signal_connect (GTK_OBJECT(actstation3), "clicked",
  //         G_CALLBACK (button_was_clicked), (gpointer) "Station 3");

  // actstation4 = gtk_button_new_with_label("Station 4");
  // gtk_widget_set_size_request(actstation4, 100, 100);
  // gtk_table_attach(GTK_TABLE(table), actstation4, 90, 91, 2, 3, 
  //         GTK_FILL, GTK_FILL, 0, 0);
  // g_signal_connect (GTK_OBJECT(actstation4), "clicked",
  //         G_CALLBACK (button_was_clicked), (gpointer) "Station 4");

  // actstation5 = gtk_button_new_with_label("Station 5");
  // gtk_widget_set_size_request(actstation5, 100, 100);
  // gtk_table_attach(GTK_TABLE(table), actstation5, 91, 92, 2, 3, 
  //         GTK_FILL, GTK_FILL, 0, 0);
  // g_signal_connect (GTK_OBJECT(actstation5), "clicked",
  //         G_CALLBACK (button_was_clicked), (gpointer) "Station 5");

  // image = gtk_image_new ();
  // gtk_image_set_from_file (GTK_IMAGE(image), "/home/agv/logo.png");

  // //column 1
  // gtk_table_attach(GTK_TABLE(table), image, 190, 192, 1, 91,
  //         GTK_FILL, GTK_FILL, 0, 0);

  // halign2 = gtk_alignment_new(0, 1, 0, 0);
  // hlpBtn = gtk_button_new_with_label("Help");
  // gtk_container_add(GTK_CONTAINER(halign2), hlpBtn);
  // gtk_widget_set_size_request(hlpBtn, 70, 30);
  // gtk_table_set_row_spacing(GTK_TABLE(table), 3, 5);
  // gtk_table_attach(GTK_TABLE(table), halign2, 0, 1, 92, 93, 
  //     GTK_FILL, GTK_FILL, 0, 0);

  // actstation6 = gtk_button_new_with_label("Reset Stations");
  // gtk_widget_set_size_request(actstation6, 140, 30);
  // gtk_table_attach(GTK_TABLE(table), actstation6, 1, 3, 92, 93, 
  //         GTK_FILL, GTK_FILL, 0, 0);
  // g_signal_connect (GTK_OBJECT(actstation6), "clicked",
  //         G_CALLBACK (button_was_clicked), (gpointer) "Reset Stations");

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
    gtk_label_set (GTK_LABEL(widget), "Recall Robot");
  }
  else
  {
    if(robotRegister_received[0] == 1)
    {
      snprintf (TEXT, sizeof(TEXT), "ROBOT still in STATION 1\n");
      printtoconsole(TEXT);
    }
    else
    {
      gtk_label_set (GTK_LABEL(widget), "Calling");
      snprintf (TEXT, sizeof(TEXT), "Re-called ROBOT to STATION 1\n");
      printtoconsole(TEXT);
      robotRegister_sent[0] = 1;
      robot_control = 1;
      station1Register_sent[1] = 1;
    }
  }
  //gtk_label_get(GTK_LABEL(widget), button_label);
  g_print ("%s was pressed, %s\n", (char *) data, (char*) button_label);

}

/* Our usual callback function */
static void recallback1( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    gtk_label_set (GTK_LABEL(widget), "Recall Robot");
  }
}

/* Our usual callback function */
static void allowcallinghandler( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Send yarn NOT DYED") == 0)
  {
    callingallowed = 0;
    gtk_label_set (GTK_LABEL(widget), "Get yarn dyed");
    snprintf (TEXT, sizeof(TEXT), "DENIED calling from all Stations\n");
    printtoconsole(TEXT);
  }
  else
  {
    callingallowed = 1;
    gtk_label_set (GTK_LABEL(widget), "Send yarn NOT DYED");
    snprintf (TEXT, sizeof(TEXT), "ALLOWED calling from all Stations\n");
    printtoconsole(TEXT);
  }
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
    snprintf (TEXT, sizeof(TEXT), "CANCELED the requesting ROBOT to STATION 2\n");
    printtoconsole(TEXT);
  }
  else
  {
    if(robotRegister_received[0] == 2)
    {
      snprintf (TEXT, sizeof(TEXT), "ROBOT still in STATION 2\n");
      printtoconsole(TEXT);
    }
    else
    {
      gtk_label_set (GTK_LABEL(widget), "Calling");
      snprintf (TEXT, sizeof(TEXT), "ROBOT was being sent to STATION 2\n");
      printtoconsole(TEXT);
      callingallowed = 2;
      robotRegister_sent[0] = 2;
      robot_control = 2;
      station2Register_sent[1] = 1;
    }
  }
}
/* Our usual callback function */
static void recallback2( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    gtk_label_set (GTK_LABEL(widget), "Station 2");
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
    snprintf (TEXT, sizeof(TEXT), "CANCELED the requesting ROBOT to STATION 3\n");
    printtoconsole(TEXT);
  }
  else
  {
    if(robotRegister_received[0] == 3)
    {
      snprintf (TEXT, sizeof(TEXT), "ROBOT still in STATION 3\n");
      printtoconsole(TEXT);
    }
    else
    {
      gtk_label_set (GTK_LABEL(widget), "Calling");
      snprintf (TEXT, sizeof(TEXT), "ROBOT was being sent to STATION 3\n");
      printtoconsole(TEXT);
      callingallowed = 3;
      robotRegister_sent[0] = 3;
      robot_control = 3;
      station3Register_sent[1] = 1;
    }
  }
}
/* Our usual callback function */
static void recallback3( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    gtk_label_set (GTK_LABEL(widget), "Station 3");
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
    snprintf (TEXT, sizeof(TEXT), "CANCELED the requesting ROBOT to STATION 4\n");
    printtoconsole(TEXT);
  }
  else
  {
    if(robotRegister_received[0] == 4)
    {
      snprintf (TEXT, sizeof(TEXT), "ROBOT still in STATION 4\n");
      printtoconsole(TEXT);
    }
    else
    {
      gtk_label_set (GTK_LABEL(widget), "Calling");
      snprintf (TEXT, sizeof(TEXT), "ROBOT was being sent to STATION 4\n");
      printtoconsole(TEXT);
      callingallowed = 4;
      robotRegister_sent[0] = 4;
      robot_control = 4;
      station4Register_sent[1] = 1;
    }
  }
}
/* Our usual callback function */
static void recallback4( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    gtk_label_set (GTK_LABEL(widget), "Station 4");
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
    snprintf (TEXT, sizeof(TEXT), "CANCELED the requesting ROBOT to STATION 5\n");
    printtoconsole(TEXT);
  }
  else
  {
    if(robotRegister_received[0] == 5)
    {
      snprintf (TEXT, sizeof(TEXT), "ROBOT still in STATION 5\n");
      printtoconsole(TEXT);
    }
    else
    {
      gtk_label_set (GTK_LABEL(widget), "Calling");
      snprintf (TEXT, sizeof(TEXT), "ROBOT was being sent to STATION 5\n");
      printtoconsole(TEXT);
      callingallowed = 5;
      robotRegister_sent[0] = 5;
      robot_control = 5;
      station5Register_sent[1] = 1;
    }
  }
}
/* Our usual callback function */
static void recallback5( GtkWidget *widget,
                      gpointer   data )
{
  const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  if(strcmp(button_label, "Calling") == 0)
  {
    gtk_label_set (GTK_LABEL(widget), "Station 5");
  }
}

/* Our usual callback function */
static void callback6( GtkWidget *widget,
                      gpointer   data )
{
  snprintf(TEXT, sizeof(TEXT), "This function is being developed!!!\n");
  printtoconsole(TEXT);
  // printf("Reset all stations\n");
  // memset(station1Register_received, 0, sizeof(station1Register_received));
  // memset(station2Register_received, 0, sizeof(station2Register_received));
  // memset(station3Register_received, 0, sizeof(station3Register_received));
  // memset(station4Register_received, 0, sizeof(station4Register_received));
  // memset(station5Register_received, 0, sizeof(station5Register_received));

  // memset(station1Register_sent, 0, sizeof(station1Register_sent));
  // memset(station2Register_sent, 0, sizeof(station2Register_sent));
  // memset(station3Register_sent, 0, sizeof(station3Register_sent));
  // memset(station4Register_sent, 0, sizeof(station4Register_sent));
  // memset(station5Register_sent, 0, sizeof(station5Register_sent));

  // station1Register_sent[4] = 1;
  // station2Register_sent[4] = 1;
  // station3Register_sent[4] = 1;
  // station4Register_sent[4] = 1;
  // station5Register_sent[4] = 1;

  // memset(station1Register_sent_previous, -1, sizeof(station1Register_sent_previous));
  // memset(station2Register_sent_previous, -1, sizeof(station2Register_sent_previous));
  // memset(station3Register_sent_previous, -1, sizeof(station3Register_sent_previous));
  // memset(station4Register_sent_previous, -1, sizeof(station4Register_sent_previous));
  // memset(station5Register_sent_previous, -1, sizeof(station5Register_sent_previous));
}

void
quick_message (GtkWindow *parent, gchar *message)
{
 GtkWidget *dialog, *label, *content_area;
 GtkDialogFlags flags;

 // Create the widgets
 flags = GTK_DIALOG_DESTROY_WITH_PARENT;
 dialog = gtk_dialog_new_with_buttons ("Message",
                                       parent,
                                       flags,
                                       ("_OK"),
                                       GTK_RESPONSE_NONE,
                                       NULL);
 content_area = gtk_dialog_get_content_area (GTK_DIALOG (dialog));
 label = gtk_label_new (message);

 // Ensure that the dialog box is destroyed when the user responds

 g_signal_connect_swapped (dialog,
                           "response",
                           G_CALLBACK (gtk_widget_destroy),
                           dialog);

 // Add the label, and show everything we’ve added

 gtk_container_add (GTK_CONTAINER (content_area), label);
 gtk_widget_show_all (dialog);
}

void printtoconsole(char* text)
{
  GtkTextIter start;
  GtkTextIter end;
  // gtk_text_buffer_get_bounds(consoletxt, &start, &end);
  // gtk_text_buffer_delete(consoletxt, &start, &end);
  gtk_text_buffer_insert(consoletxt, &iter, text, -1);
  gtk_text_buffer_get_end_iter(consoletxt, &end);
  gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(wins), &end, 0.0, FALSE, 0.0,0.0);
}

uint32_t count=0;
void RequestingProcess(void)
{
  station1request = station1Register_received[0];
  station2request = station2Register_received[0];
  station3request = station3Register_received[0];
  station4request = station4Register_received[0];
  station5request = station5Register_received[0];
  //printf("station 2 calling status: %d\n", callingallowed);

  if(++count == 10000000)
  {
    count = 0;
    //printf("%d %d %d %d %d\n", station1request, station2request, station3request, station4request, station5request);

  }
  if((station2request != -1))
  {
    if(
      (station3request == 0 || station3request == -1 || station3_isaccepted == 0) &&
      (station5request == 0 || station5request == -1 || station5_isaccepted == 0)
      )
    {
      if(
          (station2_isaccepted == 1)
        )

      {
        if(
            (station2request == 1)
          )
        {
          robotworking                  = 2;
          robotRegister_sent[0]         = 2; 
          station2Register_sent[2]      = 2;
          if((station2_processed == 0))
          {
            station2_processed          = 1;
            snprintf(TEXT, sizeof(TEXT), "[OK] [%6d] Station 2 requests robot at => %s", ++station2_counter, getTime());
            printtoconsole(TEXT);
            printf("[OK] [%6d] Station 2 requests robot at => %s", station2_counter, getTime());
          }
        }
        else
        {
          robotworking = 0;
          station2Register_sent[0]  = 0;
          station2Register_sent[2]  = 0;
          if((station2_processed == 1))
          {
            station2_processed        = 0;
            robotRegister_sent[0]     = 4;
            station4Register_sent[1]    = 1;

            printf("[OK] Station 2 has canceled requests at => %s", getTime());
            snprintf(TEXT, sizeof(TEXT), "[OK] Station 2 has canceled requests at => %s", getTime());
            printtoconsole(TEXT);
          }
        }
      }
    }
  }

  if(
    (station5request == 0) || (station5request == -1) || (station5_isaccepted == 0) || (station3allowedfrommaster == 1)
    )
  {
    if(
        (station3_isaccepted == 1)
      )
    {
      if((station3request != -1))
      {
        if((station3request == 1))
        {
          robotworking = 3;
          robotRegister_sent[0]         = 3;
          station3Register_sent[2]      = 3;
          if((station3_processed == 0))
          {
            station3_processed          = 1;
            snprintf(TEXT, sizeof(TEXT), "[OK] [%6d] Station 3 requests robot at => %s", ++station3_counter, getTime());
            printtoconsole(TEXT);
            printf("[OK] [%6d] Station 3 requests robot at => %s", station3_counter, getTime());
          }
        }
        else if(station3request == 0)
        {
          station3Register_sent[0]      = 0;
          station3Register_sent[1]      = 0;
          station3Register_sent[2]      = 0;
          robotworking                  = 0;
          station4Register_sent[1]      = 1;
          station3allowedfrommaster     = 0;
          if((station3_processed == 1))
          {
            station3_processed = 0;
            robotRegister_sent[0]         = 4;
            printf("[OK] Station 3 has canceled requests at => %s", getTime());
            snprintf(TEXT, sizeof(TEXT), "[OK] Station 3 has canceled requests at => %s", getTime());
            printtoconsole(TEXT);
          }
        }
      }
    }
  }
  
  if((station5request != -1))
  {
    if(
        (station5_isaccepted == 1)
      )
    {
      if((station5request == 1))
      {
        robotworking = 5;
        robotRegister_sent[0]         = 5;
        station5Register_sent[2]      = 5;
        if((station5_processed == 0))
        {
          station5_processed          = 1;
          snprintf(TEXT, sizeof(TEXT), "[OK] [%6d] Station 5 requests robot at => %s", ++station5_counter, getTime());
          printtoconsole(TEXT);
          printf("[OK] [%6d] Station 5 requests robot at => %s", station5_counter, getTime());
        }
      }
      else if(
              (station2request == 0) &&
              (station3request == 0)
              )
      {
        station5Register_sent[0]      = 0;
        station5Register_sent[2]      = 0;
        robotworking = 0;
        if((station5_processed == 1))
        {
          station5_processed          = 0;
          robotRegister_sent[0]       = 4;
          station4Register_sent[1]    = 1;
          printf("[OK] Station 5 has canceled requests at => %s", getTime());
          snprintf(TEXT, sizeof(TEXT), "[OK] Station 5 has canceled requests at => %s", getTime());
          printtoconsole(TEXT);
        }
      }
    }
  }
}

void ControllProcess(void)
{

  int16_t robotlocation = robotRegister_received[0];
  switch(robotlocation)
  {
    case 1:
      staion4_locking = 0;
    break;
    case 2:
      staion4_locking = 0;
      if(station2_processed == 1)
      {
        robotworking = 2;
        station1Register_sent[0]=0;
        station2Register_sent[0]=robotlocation;
        station2Register_sent[1]=2;
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
              snprintf(TEXT, sizeof(TEXT), "[OK] Robot was finished at station %d at %s", robotlocation, getTime());
              printtoconsole(TEXT);
              break;
            }
          }
        }
        sleep(1);
        STATION1_ENABLE   =  0;
        STATION2_ENABLE   =  1;
        STATION3_ENABLE   =  1;
        STATION4_ENABLE   =  1;
        STATION5_ENABLE   =  1;
        robotworking = 0;
        station2Register_sent[0]=0;
        station2Register_sent[1]=0;
        station2Register_sent[2]=0;
        robotRegister_sent[0]=4;
        station2_processed = 0;
      }
    break;
    case 3:
      staion4_locking = 0;
      if(station3_processed == 1)
      {
        robotworking = 3;
        station1Register_sent[0]=0;
        station2Register_sent[0]=0;
        station3Register_sent[0]=robotlocation;
        station3Register_sent[1]=2;
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
              snprintf(TEXT, sizeof(TEXT), "[OK] Robot was finished at station %d at %s", robotlocation, getTime());
              printtoconsole(TEXT);
              break;
            }

          }
        }
        sleep(1);
        STATION1_ENABLE   =  0;
        STATION2_ENABLE   =  1;
        STATION3_ENABLE   =  1;
        STATION4_ENABLE   =  1;
        STATION5_ENABLE   =  1;
        robotworking = 0;
        station3Register_sent[0]=0;
        station3Register_sent[1]=0;
        station3Register_sent[2]=0;
        robotRegister_sent[0]=4;
        station3_processed = 0;
      }
    break;
    case 4:
      if(staion4_locking == 0)
      {
        robotworking = 4;
        station4Register_sent[1] = 2;
        while(station4Register_received[3] == 0 || (station4Register_received[0] == -1))
        {
          printf("Wating Station 4 confirms\n");
          sleep(1);
        }
        robotRegister_sent[0] = 1;
        snprintf(TEXT, sizeof(TEXT), "Station 4 was confirmed at %s", getTime());
        printtoconsole(TEXT);
        printf("Station 4 was confirmed\n");
        station4Register_sent[0] = 0;
        station4Register_sent[1] = 0;
        station4Register_sent[2] = 0;
        robotworking = 0;
        staion4_locking = 1;
      }
    break;
    case 5:
      staion4_locking = 0;
    if(station5_processed == 1)
    {
      robotworking = 5;
      station1Register_sent[0]=0;
      station2Register_sent[0]=0;
      station3Register_sent[0]=0;
      station4Register_sent[0]=0;
      station5Register_sent[0]=robotlocation;
      station5Register_sent[1]=2;
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
            snprintf(TEXT, sizeof(TEXT), "[OK] Robot was finished at station %d at %s", robotlocation, getTime());
            printtoconsole(TEXT);
            break;
          }

        }
      }
      sleep(1);
      STATION1_ENABLE   =  0;
      STATION2_ENABLE   =  1;
      STATION3_ENABLE   =  1;
      STATION4_ENABLE   =  1;
      STATION5_ENABLE   =  1;
      robotworking = 0;
      station5Register_sent[0]=0;
      station5Register_sent[1]=0;
      station5Register_sent[2]=0;
      robotRegister_sent[0]=4;
      station5_processed = 0;
    }
    break;
    default:
      //printf("[WARNING] Unknow robot location!!!\n");
    break;
  }
}

void stationresponding(uint16_t id)
{
  switch(id)
  {
    case 2:

    break;
    case 3:

    break;
    case 4:

    break;
    case 5:

    break;
    case 6:

    break;
    case 7:

    break;
    case 8:

    break;
    case 9:

    break;
    case 10:

    break;
    case 11:

    break;
    case 12:
      stationWrite_reg[12][1] = 12;
    break;
    case 13:

    break;
    case 14:

    break;
    default:
      DEBUG_PRINT("[ERROR] Invalid Station ID\n");
    break;
  }
}
