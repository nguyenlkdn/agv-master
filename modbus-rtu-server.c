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

#define STATION_TIMEOUT_uS        0
#define STATION_TIMEOUT_S         1

#define STATION_WRITE_TIMEOUT_uS  0
#define STATION_WRITE_TIMEOUT_S   1

#define ROBOT_WRITE_TIMEOUT_S     1
#define ROBOT_WRITE_TIMEOUT_uS    0

#define ROBOT_READ_TIMEOUT_S      1
#define ROBOT_READ_TIMEOUT_uS     0
/*
/////// Global Variables
*/
#define font "Sans 60"
PangoFontDescription *font_desc;
char TEXT[255];
uint8_t callingallowed = 1;
uint16_t UserCallingBuffer[10];
GtkTextBuffer *consoletxt;
GtkTextIter iter;
GtkWidget *wins;

/*
////////// Station Variables ///////////////////
*/
int16_t stationRead_reg[STATION_MAX][5];
int16_t stationWrite_reg[STATION_MAX][5];
int16_t stationstatus[STATION_MAX];
int16_t under_control_ofMaster=0;
int16_t IS_GUI_INITED = 0;
/*
////////// Robot Variables ///////////////////
*/
int16_t robotRegister_received[MODBUS_TCP_MAX_ADU_LENGTH];
uint16_t robotRegister_sent[MODBUS_TCP_MAX_ADU_LENGTH];
/*

*/
typedef struct _CallingHistory
{
	int16_t data[100];
	int16_t windex;
	int16_t rindex;
}CallingHistory;
CallingHistory History;
/*

*/
void GUIInit(int argc, char *argv[]);
void *stationThread(void *vargp);
void *robotThread(void *vargp);
void *userThread();
void *testing();
void initThread();
void robotInit();
void stationInit();
char *getTime();
void printtoconsole(char* text);

// Jan 14
uint16_t stationresponding(uint16_t id, int32_t usleeptime);
uint16_t stationderesponding(uint16_t id);

uint16_t stationcontroller(uint16_t id);
int16_t stationwriting(int16_t id, int16_t* regs, int32_t usleeptime);
int16_t stationreading(int16_t id, int16_t *regs, int32_t usleeptime);

static void allowcallinghandler( GtkWidget *widget, gpointer data);
static void callback( GtkWidget *widget, gpointer data);

static void recallback( GtkWidget *widget, gpointer data);
static void button_was_clicked (GtkWidget *widget, gpointer gdata);

void attachcalling(int16_t data);
int16_t detachcalling(void);
void quickSort(void);
int16_t isemtpyHistory(void);
void deleteHistory(int16_t data);
int16_t checkingHistory();
int16_t robotlocationGet(void);

///////////////////////////////////////
/*

*/

/*

*/
modbus_t *ctx;
modbus_t *modbus_rtu_robot_ctx;
modbus_t *modbus_rtu_station_ctx;
modbus_t *modbus_rtu_station_zigbee_ctx;
modbus_mapping_t *mb_mapping;
modbus_mapping_t *modbus_rtu_robot_mb_mapping;
modbus_mapping_t *modbus_rtu_station_mb_mapping;

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
GtkWidget *actstation11;
GtkWidget *actstation12;
GtkWidget *actstation13;
GtkWidget *actstation14;
GtkWidget *actstation15;
GtkWidget *btnallowcalling;

GtkWidget *image;
int16_t robot_status = 0;
int16_t robot_sensor = 0;
uint16_t robot_control = 0;

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
GtkWidget *station11status;
GtkWidget *station12status;
GtkWidget *station13status;
GtkWidget *station14status;
GtkWidget *station15status;
/*
*/
int main(int argc, char *argv[])
{
  initThread();
  robotInit();
  stationInit();
  pthread_t stationThread_id, userThread_id, robotThread_id;
  pthread_t testing_id;
  //robotRegister_sent[0] = 15;
  pthread_create(&userThread_id, NULL, userThread, NULL);
  pthread_create(&robotThread_id, NULL, robotThread, NULL);
  pthread_create(&stationThread_id, NULL, stationThread, NULL);

  //pthread_create(&testing_id, NULL, testing, NULL);

  pthread_join(userThread_id, NULL);
  pthread_join(robotThread_id, NULL);
  pthread_join(stationThread_id, NULL);

  //pthread_join(testing_id, NULL);
  return 0;
}

void *userThread()
{
  GUIInit(0, NULL);
}

void *testing()
{
	int pos;
  while(1)
  {
  	if(isemtpyHistory() == 0)
  	{
  		pos = checkingHistory();
  		printf("Getting Calling: %d\n", pos);
  		//deleteHistory(pos);
  	}
  	else
  	{
  		printf("No any calling\n");
  	}
  	sleep(1);
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
    if(robotRegister_received[0] == 15 && robotRegister_sent[0] == 15)
    {
      DEBUG_PRINT("Robot was come to Station 15\n");

      // Turn of the Led add the station 15
      stationWrite_reg[15][1] = 2;
      stationWrite_reg[15][0] = 15;
      stationwriting(15, stationWrite_reg[15], 500000);

      // Wating the Station 15 Confirm or Recalling from Master
      while(stationRead_reg[15][3] == 0 || (stationRead_reg[15][3] == -1))
      {
      	usleep(1000000);
        stationreading(15, stationRead_reg[15], 200000);
        printf("[AGV INFO] Wating Station 15 confirms\n");
        if(robotRegister_sent[0] != 15 )
        {
        	snprintf(TEXT, sizeof(TEXT), "[WARNING] Robot was recalling without confirm of ST 15");
        	printtoconsole(TEXT);
        	break;
        }
      }

      // Default at the station 15 will send robot to Station 1
      robotRegister_sent[0] = 1;

      // Print to log on GUI
      snprintf(TEXT, sizeof(TEXT), "Finished at ST 15 %s", getTime());
      printtoconsole(TEXT);

      // Clear Status of ST 15
      stationWrite_reg[15][0] = 0;
      stationWrite_reg[15][1] = 0;
      stationWrite_reg[15][2] = 0;
      stationwriting(15, stationWrite_reg[15], 500000);
    }

    else if(isemtpyHistory() == 0)
    {
    	while(1)
    	{
    		// Get the reqeusting from history
    		if(checkingHistory() != -1)
    		{

    			// Get the history from Buffer
    			under_control_ofMaster = checkingHistory();
    			printf("[AGV INFO] Under control of Master at the %d\n", under_control_ofMaster);

    			// Request robot to stations
    			robotRegister_sent[0] = under_control_ofMaster;
    	  		
    	  		// Confirm with station
    			stationWrite_reg[under_control_ofMaster][1] = 1;
    			stationwriting(under_control_ofMaster, stationWrite_reg[under_control_ofMaster], 500000);

    	  		// Checking robot come station or not
    	  		if(stationresponding(under_control_ofMaster, 500000) == 1)
    	  		{
    	  			// Turn on the led
    	  			stationWrite_reg[under_control_ofMaster][1] = 2;
   					stationwriting(under_control_ofMaster, stationWrite_reg[under_control_ofMaster], 500000);
    	  		  	
    	  		  	// Allow station increase/decrease the carier
    	  		  	stationcontroller(under_control_ofMaster);

    	  		  	// Clear the requesting
    	  		  	deleteHistory(under_control_ofMaster);
    	  		  	under_control_ofMaster = 0;
    	  		  	break;
    	  		}
    	  		else
    	  		{
    	  			if(robotRegister_received[0] > under_control_ofMaster)
    	  			{
                printf("[AGV INFO] Removed %d due to robot ignore that station\n", under_control_ofMaster);
    	  				// Clear History if robot ignore RFID
    	  				deleteHistory(under_control_ofMaster);

    	  				// Clear The status of station 
    	  				stationWrite_reg[under_control_ofMaster][0] = 0;
    	  				stationWrite_reg[under_control_ofMaster][1] = 0;
    	  				stationWrite_reg[under_control_ofMaster][2] = 0;
    	  				stationwriting(under_control_ofMaster, stationWrite_reg[under_control_ofMaster], 500000);

                break;
    	  			}
    	  		}
    		}
    		usleep(LOOP_uSLEEP_TIME);
    	}
    }
    else if (callingallowed == 1)
    {
      /*
        List of station will be ignored
      */
    	#ifdef DEBUG
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
	    #endif
      ////////////////////////////////////
      /*
        Reading Stations
      */
      int32_t stationscan;
      uint16_t come_to_valid_point;

      if(robotRegister_received[0] > 2)
      {
        stationscan = robotRegister_received[0];
      }
      else
      {
        stationscan=STATION_START;
      }

      printf("[AGV INFO] Searching the requesting %d -> %d\n", stationscan, STATION_MAX);

      for(;stationscan<=STATION_MAX;stationscan++)
      {
        if(stationignore[stationscan] == 1)
        {
          modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
          modbus_set_slave(modbus_rtu_station_ctx, stationscan);
          modbus_set_debug(modbus_rtu_station_ctx, FALSE);

          ////////////// Reading //////////////////////
          rc = modbus_read_registers(modbus_rtu_station_ctx, 0, 5, stationRead_reg[stationscan]);
          if(rc == -1)
          {
            memset(stationRead_reg[stationscan], -1, sizeof(stationRead_reg[stationscan]));
            printf("[AGV ERROR] STATION %d: READ TIMEOUT!\n", stationscan);
          }
          else
          {
            int i;
            #ifdef DEBUG
	            DEBUG_PRINT("[%d] STATION %d READs OK, with Data: ", rc, stationscan);
	            for(i=0;i<5;i++)
	            {
	              DEBUG_PRINT("%6d", stationRead_reg[stationscan][i], i);
	            }
	            DEBUG_PRINT("\n");
            #endif
            // Sleep between Reading & Writing Thread
            usleep(500000);
            
            ////////////////////////////////////
            // /*
            //   Checking Calling
            // */
            if(stationRead_reg[stationscan][0] == 1)
            {
              stationstatus[stationscan] = 1;
              come_to_valid_point = stationresponding(stationscan, 500000);
              if(come_to_valid_point == 1)
              {
                stationcontroller(stationscan);
              }
              break;
            }
            else
            {
              stationderesponding(stationscan);
            }
            ////////////////////////////////////
          }
        }
      }
    }

    DEBUG_PRINT("%s %d %d\n", __FUNCTION__, robotRegister_received[0], robotRegister_sent[0]);

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
        //printf("Has new packages so that writing to robot: %d \n", robotRegister_sent[0]);
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
      DEBUG_PRINT("Write to robot: %d %d %d %d %d\n", robotRegister_sent[0], robotRegister_sent[1], robotRegister_sent[2], robotRegister_sent[3], robotRegister_sent[4], robotRegister_sent[5]);
      modbus_flush(modbus_rtu_robot_ctx);
      modbus_set_response_timeout(modbus_rtu_robot_ctx, ROBOT_WRITE_TIMEOUT_S, ROBOT_WRITE_TIMEOUT_uS);
      //modbus_set_debug(modbus_rtu_robot_ctx, TRUE);
      rc = modbus_write_registers(modbus_rtu_robot_ctx, 0, 5, robotRegister_sent);
      //modbus_set_debug(modbus_rtu_robot_ctx, FALSE);
      if(rc != 5)
      {
        rewrite = 1;
        DEBUG_PRINT("[AGV ERROR] Robot Writing Failed so that re-writing\n");
      }
      else
      {
        memcpy(robotRegister_sent_previous, robotRegister_sent, sizeof(robotRegister_sent_previous));
        DEBUG_PRINT("Robot Writing: OK\n");
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
        DEBUG_PRINT("Robot Reading: OK\n");
      #endif
      usleep(200000);
    }
    else
    {
      printf("[AGV ERROR] Robot Reading: Timeout %d\n", rc);
    }
    if(robotRegister_received[0] != 0)
    {
      snprintf(TEXT, sizeof(TEXT), "%d", robotRegister_received[0]);
    }
    else
    {
      snprintf(TEXT, sizeof(TEXT), "none");
    }
    gtk_entry_set_text (GTK_ENTRY (robotlocation), TEXT);

    snprintf(TEXT, sizeof(TEXT), "%d", robotRegister_received[2]);
    gtk_entry_set_text (GTK_ENTRY (robotstatus), TEXT);


    if(robotRegister_received[0] == 0)
    {
      robotRegister_sent[2] = robot_status;
      robotRegister_sent[3] = robot_sensor;
      DEBUG_PRINT("Robot was reseted\n");
    }
    else
    {
      robot_status = robotRegister_received[0];
      if(robotRegister_received[2] != 3)
      {
        robot_sensor = robotRegister_received[2];
      }
      // robotRegister_sent[2] = 0;

      //////////////////////// Reset status /////////////////////////////
      if(robotRegister_received[0] == 1)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation1), 
                               (GtkCallback) recallback, "ST1");
      }
      else if(robotRegister_received[0] == 2)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation2), 
                               (GtkCallback) recallback, "ST2");
      }
      else if(robotRegister_received[0] == 3)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation3), 
                               (GtkCallback) recallback, "ST3");
      }
      else if(robotRegister_received[0] == 4)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation4), 
                               (GtkCallback) recallback, "ST4");
      }
      else if(robotRegister_received[0] == 5)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation5), 
                               (GtkCallback) recallback, "ST5");
      }
      else if(robotRegister_received[0] == 6)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation6), 
                               (GtkCallback) recallback, "ST6");
      }
      else if(robotRegister_received[0] == 7)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation7), 
                               (GtkCallback) recallback, "ST7");
      }
      else if(robotRegister_received[0] == 8)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation8), 
                               (GtkCallback) recallback, "ST8");
      }
      else if(robotRegister_received[0] == 9)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation9), 
                               (GtkCallback) recallback, "ST9");
      }
      else if(robotRegister_received[0] == 10)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation10), 
                               (GtkCallback) recallback, "ST10");
      }
      else if(robotRegister_received[0] == 11)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation11), 
                               (GtkCallback) recallback, "ST11");
      }
      else if(robotRegister_received[0] == 12)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation12), 
                               (GtkCallback) recallback, "ST12");
      }
      else if(robotRegister_received[0] == 13)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation13), 
                               (GtkCallback) recallback, "ST13");
      }
      else if(robotRegister_received[0] == 14)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation14), 
                               (GtkCallback) recallback, "ST14");
      }
      else if(robotRegister_received[0] == 15)
      {
        gtk_container_foreach (GTK_CONTAINER (actstation15), 
                               (GtkCallback) recallback, "ST15");
      }
    }
    usleep(LOOP_uSLEEP_TIME);
  }
  return NULL;
}

////////////// Init Common ////////////
///////////////////////////////////////
///////////////////////////////////////
void initThread()
{
  memset(robotRegister_sent, 0, sizeof(robotRegister_sent));
  memset(robotRegister_received, 0, sizeof(robotRegister_received));
  memset(UserCallingBuffer, 0, sizeof(UserCallingBuffer));
  History.windex = 0;
  memset(History.data, -1, 100);
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
  modbus_rtu_station_ctx = modbus_new_rtu(STATION_DEVICE, 9600, 'N', 8, 1);
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
   memset(stationignore, 1, sizeof(stationignore));
   memset(stationstatus, 0, sizeof(stationstatus));
   //stationignore[2]=0;
   //stationignore[11]=0;
   int i;
   for(i=0;i<STATION_MAX;i++)
   {
    memset(stationRead_reg[i], 0, 5);
    memset(stationWrite_reg[i], 0, 5);
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
  // GdkColor red = {0x0000, 47575, 65535, 64858};
  // gtk_widget_modify_bg(GTK_CONTAINER(window), GTK_STATE_NORMAL, &red);

  table = gtk_table_new(11, 9, FALSE);
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
  gtk_table_attach(GTK_TABLE(table), halign, 1, 4, 9, 10, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Status ");
  gtk_widget_modify_font(title, df);
  gtk_table_attach(GTK_TABLE(table), title, 4, 6, 2, 3, 
      GTK_FILL, GTK_FILL, 0, 0);

  // Robot Status labels
  ////////////////////////////////////
  title = gtk_label_new("Robot Battery: ");
  //gtk_widget_modify_font(title, df);
  gtk_widget_set_size_request(title, 150, 50);
  gtk_table_attach(GTK_TABLE(table), title, 4, 5, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotbattery = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotbattery), 50);
  gtk_entry_set_text (GTK_ENTRY (robotbattery), "none");
  gtk_table_attach(GTK_TABLE(table), robotbattery, 5, 6, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Location: ");
  gtk_table_attach(GTK_TABLE(table), title, 4, 5, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotlocation = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotlocation), 50);
  gtk_entry_set_text (GTK_ENTRY (robotlocation), "none");
  gtk_table_attach(GTK_TABLE(table), robotlocation, 5, 6, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Status: ");
  gtk_table_attach(GTK_TABLE(table), title, 4, 5, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotstatus = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotstatus), 50);
  gtk_entry_set_text (GTK_ENTRY (robotstatus), "none");
  gtk_table_attach(GTK_TABLE(table), robotstatus, 5, 6, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Connection: ");
  gtk_table_attach(GTK_TABLE(table), title, 4, 5, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotconnection = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotconnection), 50);
  gtk_entry_set_text (GTK_ENTRY (robotconnection), "none");
  gtk_table_attach(GTK_TABLE(table), robotconnection, 5, 6, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Robot Speed Max: ");
  gtk_table_attach(GTK_TABLE(table), title, 4, 5, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  robotspeedmax = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (robotspeedmax), 50);
  gtk_entry_set_text (GTK_ENTRY (robotspeedmax), "none");
  gtk_table_attach(GTK_TABLE(table), robotspeedmax, 5, 6, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station Status ");
  gtk_widget_modify_font(title, df);
  //halign = gtk_alignment_new(0, 0, 0, 0);
  //gtk_container_add(GTK_CONTAINER(halign), title);
  gtk_table_attach(GTK_TABLE(table), title, 6, 10, 2, 3, 
      GTK_FILL, GTK_FILL, 0, 0);

  // station Status labels
  ////////////////////////////////////
  title = gtk_label_new("   Station 1: ");
  gtk_table_attach(GTK_TABLE(table), title, 6, 7, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  station1status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station1status), 50);
  gtk_entry_set_text (GTK_ENTRY (station1status), "none");
  gtk_table_attach(GTK_TABLE(table), station1status, 7, 8, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 3: ");
  gtk_table_attach(GTK_TABLE(table), title, 6, 7, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  station3status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station3status), 50);
  gtk_entry_set_text (GTK_ENTRY (station3status), "none");
  gtk_table_attach(GTK_TABLE(table), station3status, 7, 8, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 5: ");
  gtk_table_attach(GTK_TABLE(table), title, 6, 7, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  station5status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station5status), 50);
  gtk_entry_set_text (GTK_ENTRY (station5status), "none");
  gtk_table_attach(GTK_TABLE(table), station5status, 7, 8, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 7: ");
  gtk_table_attach(GTK_TABLE(table), title, 6, 7, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  station7status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station7status), 50);
  gtk_entry_set_text (GTK_ENTRY (station7status), "none");
  gtk_table_attach(GTK_TABLE(table), station7status, 7, 8, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 9: ");
  gtk_table_attach(GTK_TABLE(table), title, 6, 7, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  station9status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station9status), 50);
  gtk_entry_set_text (GTK_ENTRY (station9status), "none");
  gtk_table_attach(GTK_TABLE(table), station9status, 7, 8, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 2: ");
  gtk_table_attach(GTK_TABLE(table), title, 8, 9, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);
  station2status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station2status), 50);
  gtk_entry_set_text (GTK_ENTRY (station2status), "none");
  gtk_table_attach(GTK_TABLE(table), station2status, 9, 10, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 4: ");
  gtk_table_attach(GTK_TABLE(table), title, 8, 9, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  station4status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station4status), 50);
  gtk_entry_set_text (GTK_ENTRY (station4status), "none");
  gtk_table_attach(GTK_TABLE(table), station4status, 9, 10, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 6: ");
  gtk_table_attach(GTK_TABLE(table), title, 8, 9, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  station6status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station6status), 50);
  gtk_entry_set_text (GTK_ENTRY (station6status), "none");
  gtk_table_attach(GTK_TABLE(table), station6status, 9, 10, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 8: ");
  gtk_table_attach(GTK_TABLE(table), title, 8, 9, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  station8status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station8status), 50);
  gtk_entry_set_text (GTK_ENTRY (station8status), "none");
  gtk_table_attach(GTK_TABLE(table), station8status, 9, 10, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 10: ");
  gtk_table_attach(GTK_TABLE(table), title, 8, 9, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  station10status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station10status), 50);
  gtk_entry_set_text (GTK_ENTRY (station10status), "none");
  gtk_table_attach(GTK_TABLE(table), station10status, 9, 10, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  ////////////////////////////////////
  title = gtk_label_new("Station 11: ");
  gtk_table_attach(GTK_TABLE(table), title, 10, 11, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  station11status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station11status), 50);
  gtk_entry_set_text (GTK_ENTRY (station11status), "none");
  gtk_table_attach(GTK_TABLE(table), station11status, 11, 12, 3, 4, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 12: ");
  gtk_table_attach(GTK_TABLE(table), title, 10, 11, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  station12status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station12status), 50);
  gtk_entry_set_text (GTK_ENTRY (station12status), "none");
  gtk_table_attach(GTK_TABLE(table), station12status, 11, 12, 4, 5, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 13: ");
  gtk_table_attach(GTK_TABLE(table), title, 10, 11, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  station13status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station13status), 50);
  gtk_entry_set_text (GTK_ENTRY (station13status), "none");
  gtk_table_attach(GTK_TABLE(table), station13status, 11, 12, 5, 6, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 14: ");
  gtk_table_attach(GTK_TABLE(table), title, 10, 11, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  station14status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station14status), 50);
  gtk_entry_set_text (GTK_ENTRY (station14status), "none");
  gtk_table_attach(GTK_TABLE(table), station14status, 11, 12, 6, 7, 
      GTK_FILL, GTK_FILL, 0, 0);

  title = gtk_label_new("Station 15: ");
  gtk_table_attach(GTK_TABLE(table), title, 10, 11, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  station15status = gtk_entry_new ();
  gtk_entry_set_max_length (GTK_ENTRY (station15status), 50);
  gtk_entry_set_text (GTK_ENTRY (station15status), "none");
  gtk_table_attach(GTK_TABLE(table), station15status, 11, 12, 7, 8, 
      GTK_FILL, GTK_FILL, 0, 0);

  ////////////////////////////////////

  gtk_text_view_set_editable(GTK_TEXT_VIEW(wins), FALSE);
  gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(wins), FALSE);
  gtk_widget_set_size_request(wins, 400, 400);
  gtk_table_attach(GTK_TABLE(table), wins,        1, 4, 10, 11, 
      GTK_FILL | GTK_SHRINK, GTK_FILL | GTK_SHRINK, 1, 1);

  title = gtk_label_new("Control Panel");
  gtk_widget_modify_font(title, df);

  halign = gtk_alignment_new(0, 0, 0, 0);
  gtk_container_add(GTK_CONTAINER(halign), title);
  gtk_table_attach(GTK_TABLE(table), halign, 1, 4, 1, 2, 
      GTK_FILL, GTK_FILL, 0, 0);

  btnstation1 = gtk_button_new_with_label("Station 1");
  gtk_widget_set_size_request(btnstation1, 300, 100);
  gtk_widget_modify_font(btnstation1, df);

  gtk_table_attach(GTK_TABLE(table), btnstation1, 1, 2, 2, 3, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(btnstation1), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST1");

  actstation4 = gtk_button_new_with_label("Station 4");
  gtk_widget_set_size_request(actstation4, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation4, 1, 2, 3, 4, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation4), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST4");

  actstation7 = gtk_button_new_with_label("Station 7");
  gtk_widget_set_size_request(actstation7, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation7, 1, 2, 4, 5, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation7), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST7");

  actstation10 = gtk_button_new_with_label("Station 10");
  gtk_widget_set_size_request(actstation10, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation10, 1, 2, 5, 6, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation10), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST10");

  actstation13 = gtk_button_new_with_label("Station 13");
  gtk_widget_set_size_request(actstation13, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation13, 1, 2, 6, 7, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation13), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST13");

  actstation2 = gtk_button_new_with_label("Station 2");
  gtk_widget_set_size_request(actstation2, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation2, 2, 3, 2, 3, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation2), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST2");

  actstation5 = gtk_button_new_with_label("Station 5");
  gtk_widget_set_size_request(actstation5, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation5, 2, 3, 3, 4, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation5), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST5");

  actstation8 = gtk_button_new_with_label("Station 8");
  gtk_widget_set_size_request(actstation8, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation8, 2, 3, 4, 5, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation8), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST8");

  actstation11 = gtk_button_new_with_label("Station 11");
  gtk_widget_set_size_request(actstation11, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation11, 2, 3, 5, 6, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation11), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST11");

  actstation14 = gtk_button_new_with_label("Station 14");
  gtk_widget_set_size_request(actstation14, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation14, 2, 3, 6, 7, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation14), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST14");

  //////////////////////////////////
  actstation3 = gtk_button_new_with_label("Station 3");
  gtk_widget_set_size_request(actstation3, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation3, 3, 4, 2, 3, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation3), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST3");

  actstation6 = gtk_button_new_with_label("Station 6");
  gtk_widget_set_size_request(actstation6, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation6, 3, 4, 3, 4, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation6), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST6");

  actstation9 = gtk_button_new_with_label("Station 9");
  gtk_widget_set_size_request(actstation9, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation9, 3, 4, 4, 5, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation9), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST9");

  actstation12 = gtk_button_new_with_label("Station 12");
  gtk_widget_set_size_request(actstation12, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation12, 3, 4, 5, 6, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation12), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST12");

  actstation15 = gtk_button_new_with_label("Station 15");
  gtk_widget_set_size_request(actstation15, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation15, 3, 4, 6, 7, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation15), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST15");

  //////////////////////////////////

  actstation1 = gtk_button_new_with_label("Recall Robot");
  gtk_widget_set_size_request(actstation1, 300, 100);
  gtk_table_attach(GTK_TABLE(table), actstation1, 1, 4, 7, 8, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(actstation1), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "ST1");

  btnallowcalling = gtk_button_new_with_label("Get yarn dyed");
  gtk_widget_set_size_request(btnallowcalling, 300, 100);
  gtk_table_attach(GTK_TABLE(table), btnallowcalling, 1, 4, 8, 9, 
          GTK_FILL, GTK_FILL, 0, 0);
  g_signal_connect (GTK_OBJECT(btnallowcalling), "clicked",
          G_CALLBACK (button_was_clicked), (gpointer) "STCALL");
  IS_GUI_INITED = 1;

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

  gtk_container_add(GTK_CONTAINER(window), table);

  g_signal_connect(G_OBJECT(window), "destroy",
        G_CALLBACK(gtk_main_quit), G_OBJECT(window));

  gtk_widget_show_all(window);
  gtk_main();
}

/* Our usual callback function */
static void callback( GtkWidget *widget,
                      gpointer   data )
{
	const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
    if(strcmp(data, "ST1") == 0)
	    {
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 1");
	    	  deleteHistory(1);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(1);
	    	}
	  	}
	  	else if(strcmp(data, "ST2") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 2");
	    	  deleteHistory(2);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(2);
	    	}
	  	}
	  	else if(strcmp(data, "ST3") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 3");
	    	  deleteHistory(3);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(3);
	    	}
	  	}
	  	else if(strcmp(data, "ST4") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 4");
	    	  deleteHistory(4);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(4);
	    	}
	  	}
	  	else if(strcmp(data, "ST5") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  	gtk_label_set (GTK_LABEL(widget), "Station 5");
	    	  	deleteHistory(5);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(5);

	    	}
	  	}
	  	else if(strcmp(data, "ST6") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 6");
	    	  deleteHistory(6);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(6);
	    	}
	  	}
	  	else if(strcmp(data, "ST7") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 7");
	    	  deleteHistory(7);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(7);
	    	}
	  	}
	  	else if(strcmp(data, "ST8") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 8");
	    	  deleteHistory(8);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(8);
	    	}
	  	}
	  	else if(strcmp(data, "ST9") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 9");
	    	  deleteHistory(9);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(9);

	    	}
	  	}
	  	else if(strcmp(data, "ST10") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 10");
	    	  deleteHistory(10);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(10);

	    	}
	  	}
	  	else if(strcmp(data, "ST11") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 11");
	    	  deleteHistory(11);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(11);

	    	}
	  	}
	  	else if(strcmp(data, "ST12") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 12");
	    	  deleteHistory(12);
	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(12);

	    	}
	  	}
	  	else if(strcmp(data, "ST13") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 13");
	    	  deleteHistory(13);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(13);

	    	}
	  	}
	  	else if(strcmp(data, "ST14") == 0)
	    	{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 14");
	    	  deleteHistory(14);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(14);

	    	}
	  	}
	  	else if(strcmp(data, "ST15") == 0)
		{
	    	if(strcmp(button_label, "Calling") == 0)
	    	{
	    	  gtk_label_set (GTK_LABEL(widget), "Station 15");
	    	  deleteHistory(15);

	    	}
	    	else
	    	{
	    		gtk_label_set (GTK_LABEL(widget), "Calling");
	    		attachcalling(15);

	    	}
	  	}
}

/* Our usual callback function */
static void recallback( GtkWidget *widget,
                      gpointer   data )
{
  	const char* button_label = gtk_label_get_label(GTK_LABEL(widget));
  	    if(strcmp(data, "ST1") == 0)
  	    {
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 1");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST2") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 2");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST3") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 3");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST4") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 4");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST5") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  	gtk_label_set (GTK_LABEL(widget), "Station 5");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST6") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 6");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST7") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 7");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST8") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 8");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST9") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 9");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST10") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 10");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST11") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 11");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST12") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 12");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST13") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 13");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST14") == 0)
  	    	{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 14");
  	    	}
  	  	}
  	  	else if(strcmp(data, "ST15") == 0)
  		{
  	    	if(strcmp(button_label, "Calling") == 0)
  	    	{
  	    	  gtk_label_set (GTK_LABEL(widget), "Station 15");
  	    	}
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

void printtoconsole(char* text)
{
  GtkTextIter start;
  GtkTextIter end;
  gtk_text_buffer_insert(consoletxt, &iter, text, -1);
  gtk_text_buffer_get_end_iter(consoletxt, &end);
  gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(wins), &end, 0.0, FALSE, 0.0,0.0);
}

uint16_t stationresponding(uint16_t id, int32_t usleeptime)
{
  printf("[AGV INFO] %d HAS Requeting\n", id);
  usleep(usleeptime);
  if(id > 1)
  {
  	// Confirm with Station
  	stationWrite_reg[id][2] = 2;
  	stationwriting(id, stationWrite_reg[id], 500000);
  }

  // Request to Robots
  robotRegister_sent[0] = id;

  // Get Robot location
  int16_t prt_robotlocation;
  prt_robotlocation = robotRegister_received[0];

  //prt_robotlocation = 12;
  if(prt_robotlocation == id)
  {
  	if(id > 1)
  	{
  		stationWrite_reg[id][0] = id;
  		stationwriting(id, stationWrite_reg[id], 500000);
  	}
    return 1;
  }
  return 0;
}

uint16_t stationderesponding(uint16_t id)
{
  if(stationstatus[id] == 0)
  {
    printf("[AGV INFO] %d HASN'T Requeting\n", id);
  }
  else
  {
    printf("[AGV INFO] %d HAS canceled Requeting\n", id);

    // Confirm with Station
    stationWrite_reg[id][2] = 0;
    stationwriting(id, stationWrite_reg[id], 500000);

    // Clear station calling logs
    stationstatus[id] = 0;

    // Request to Robots
    robotRegister_sent[0] = 15;
  }
}

uint16_t stationcontroller(uint16_t id)
{
	if(id<=1)
	{
		return 0;
	}
  int16_t increasing, decreasing, canceled;
  canceled = 0;
  while(1)
  {
  	usleep(LOOP_uSLEEP_TIME);
    printf("[AGV INFO] Station Controller Hanlder %d!!!\n", id);

    stationreading(id, stationRead_reg[id], 200000);
    canceled = stationRead_reg[id][0];
    increasing = stationRead_reg[id][2];
    decreasing = stationRead_reg[id][3];
    if(
        (increasing == 1) &&
        (decreasing == 0)
      )
    {
      robotRegister_sent[1]=1;
      printf("\r[AGV INFO] Increased the Carier");
    }
    else if(
            (increasing == 0) &&
            (decreasing == 1)
      )
    {
      robotRegister_sent[1]=0;
      DEBUG_PRINT("\r[AGV INFO] Decreased the Carier");
    }

    if(canceled == 0)
    {
      DEBUG_PRINT("\nFinished at station %d\n", id);

      // Clear all station regs
      memset(stationWrite_reg[id], 0, 5);
      stationwriting(id, stationWrite_reg[id], 500000);

      // Clear station calling logs
      stationstatus[id] = 0;
      
      // Default sending robot to station 15
      robotRegister_sent[0] = 15;

      return 0;
    }

    if(robotlocationGet() != id)
    {
      return 0;
    }
    DEBUG_PRINT("Robot is working at %d\n", id);
  }
  return 1;
}

int16_t stationwriting(int16_t id, int16_t* regs, int32_t usleeptime)
{
  usleep(200000);
  int16_t ret;

  // Setting for writing to station
  modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
  modbus_set_slave(modbus_rtu_station_ctx, id);
  modbus_set_debug(modbus_rtu_station_ctx, FALSE);
  
  // Begin to write to station
  ret = modbus_write_registers(modbus_rtu_station_ctx, 0, 5, regs);

  // Check the returned values
  if(ret == -1)
  {
  	printf("[AGV ERROR] %d Writing TIMEOUT\n", id);
  }

  // Return writing status
  return ret;
}

int16_t stationreading(int16_t id, int16_t *regs, int32_t usleeptime)
{
  // Sleep for each reading
  if(usleeptime > 0)
  {
    usleep(usleeptime);
  }

  // Return for reading status
  int16_t rc;

  // Setting for writing to station
  modbus_set_response_timeout(modbus_rtu_station_ctx, STATION_TIMEOUT_S, STATION_TIMEOUT_uS);
  modbus_set_slave(modbus_rtu_station_ctx, id);
  modbus_set_debug(modbus_rtu_station_ctx, FALSE);

  // Begin to read
  rc = modbus_read_registers(modbus_rtu_station_ctx, 0, 5, regs);

  // Check reading status
  if(rc == -1)
  {
    printf("[AGV ERROR] %s Station %d Reading TIMEOUT\n", __FUNCTION__, id);
    return 1;
  }
  else
  {
    DEBUG_PRINT("%s Station %d Reading OK\n", __FUNCTION__, id);
    return 0;
  }
}

void button_was_clicked (GtkWidget *widget, gpointer gdata)
{
	DEBUG_PRINT("%s was clicked\n ", gdata);
  	gtk_container_foreach (GTK_CONTAINER (widget), (GtkCallback) callback, gdata);
}

void attachcalling(int16_t data)
{
	DEBUG_PRINT("Attaching %d to History at %d\n", data, History.windex);
	int idx, isinsert=0;
	if(History.windex == 100)
	{
		printf("[AGV ERROR] Full calling history\n");
	}
	else
	{
		for(idx=0;idx<History.windex;idx++)
		{
			if(History.data[idx] == data)
			{
				DEBUG_PRINT("Station exists!!!\n");
				isinsert = 1;
				break;
			}
		}
		if(isinsert == 0)
		{
			History.data[History.windex] = data;
			History.windex++;
		}
	}
	#ifdef DEBUG
		printf("History Before: ");
		for(idx=0;idx<History.windex;idx++)
		{
			printf("%3d", History.data[idx]);
		}
		printf("\n");
	#endif

	// Arragne the buffer
	quickSort();

	/////// Arranged Buffer //////////////////
	#ifdef DEBUG
		printf("[AGV ERROR] History After: ");
		for(idx=0;idx<History.windex;idx++)
		{
			printf("%3d", History.data[idx]);
		}
		printf("\n");
	#endif
}
int16_t detachcalling(void)
{
	int16_t ret = -1;
	int16_t idx;
	if(History.windex == 1)
	{
		History.windex--;
		return History.data[0];
	}
	else
	{
		ret = History.data[0];
		for(idx=0;idx<History.windex-1;idx++)
		{
			History.data[idx] = History.data[idx+1];
		}
		History.windex--;

		/////// Arranged Buffer //////////////////
		printf("Got: %d, History: ", ret);
		for(idx=0;idx<History.windex;idx++)
		{
			printf("%3d", History.data[idx]);
		}
		printf("\n");		
	}
	return ret;
}
void quickSort(void)
{
   int j, i;
   for(i=0;i<History.windex-1;i++)
   {
   	for(j=i;j<History.windex;j++)
   	{
   		if(History.data[i] > History.data[j])
   		{
   			int16_t tmp = History.data[i];
   			History.data[i] = History.data[j];
   			History.data[j] = tmp;
   		}
   	}
   }
}

int16_t isemtpyHistory(void)
{
	if(History.windex <= 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void deleteHistory(int16_t data)
{
	int16_t idx;
	int16_t del;
	for(idx=0;idx<History.windex;idx++)
	{
		if(History.data[idx] == data)
		{
			for(del=idx;del<History.windex-1;del++)
			{
				History.data[del] = History.data[del+1];
			}
			History.windex--;
			DEBUG_PRINT("Removed %d", data);
			break;
		}
	}
}
int16_t checkingHistory()
{
	if(History.windex >= 1)
	{
		return History.data[0];
	}
	else
	{
		return -1;
	}
}

int16_t robotlocationGet(void)
{
  return robotRegister_received[0];
}