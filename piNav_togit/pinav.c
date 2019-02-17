//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// "Pisces" - A Raspberry Pi Nav/GPS Boat Computer                                                     //
//  Written by: CFLanger                                                                               //
//  Contributions by Andy Langer                                                                       //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// 

#define REVISION 					"rev 3.50 24jan2019"

#define GPS_PI      				3.1415926535897932384626433832795029
#define D2R 						(GPS_PI / 180.0)

#include <gtk/gtk.h>      
#include <stdio.h>
#include <stdlib.h>  
#include <string.h>
#include <dirent.h>
#include <errno.h> 
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/types.h> 
#include <sys/msg.h> 
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/sem.h>
#include <ctype.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/statvfs.h>
#include <syslog.h>
#include <semaphore.h>
#include <gps.h>
#include <wiringPi.h>
#include <glib.h>
#include "bmp180.h"

#define MAX_WAYPOINTS				20
#define MAX_HAZPOINTS				20

#define ALL_STOP					0
#define RAISE_LEVEL					1
#define AERATE						2
#define RAISE_LEVEL_IN_DURATION 	3
#define LOWER_LEVEL					4
#define RAISE_LEVEL_B4_ALL_STOP		5

#define LED_D1_ON					digitalWrite( 24, HIGH )
#define LED_D1_OFF					digitalWrite( 24, LOW )

#define FLOAT_SWITCH 				25
#define DRAIN_PUMP 					28
#define FILL_SPRAY_PUMP 			29

#define BEEP_DURATION				40000

GtkBuilder	*builder;
GtkWidget   *window_main, *label_speed, *label_heading, *label_longitude, *label_latitude;
GtkWidget   *label_time;
GtkWidget   *image_topo, *window_about;
GtkWidget   *window_options;
GtkWidget   *levelbar2; 
GtkWidget   *entry_mapwidth, *entry_mapheight, *entry_lonleft, *entry_lonright, *entry_latbottom;
GtkWidget   *window_waypoints, *checkbutton_way, *checkbutton_haz;
GtkWidget   *label_bmptemp, *label7, *labelCPUtemp, *label2;

GdkPixbuf   *pixbTopo, *cpixbTopo, *pixbBoatSymbol, *cpixbway_num, *cpixbhaz_num, *pixbwptrack;

GdkPixbuf   *pixbway1, *pixbway2, *pixbway3, *pixbway4, *pixbway5, *pixbway6, *pixbway7;
GdkPixbuf   *pixbway8, *pixbway9, *pixbway10;  
GdkPixbuf   *pixbway11, *pixbway12, *pixbway13, *pixbway14, *pixbway15, *pixbway16, *pixbway17;
GdkPixbuf   *pixbway18, *pixbway19, *pixbway20; 

GdkPixbuf   *pixbhaz1, *pixbhaz2, *pixbhaz3, *pixbhaz4, *pixbhaz5, *pixbhaz6, *pixbhaz7;
GdkPixbuf   *pixbhaz8, *pixbhaz9, *pixbhaz10;  
GdkPixbuf   *pixbhaz11, *pixbhaz12, *pixbhaz13, *pixbhaz14, *pixbhaz15, *pixbhaz16, *pixbhaz17;
GdkPixbuf   *pixbhaz18, *pixbhaz19, *pixbhaz20;  

GtkWidget   *entrywpid[MAX_WAYPOINTS], *entrywpx[MAX_WAYPOINTS], *entrywpy[MAX_WAYPOINTS];
GtkWidget   *entryhpid[MAX_HAZPOINTS], *entryhpx[MAX_HAZPOINTS], *entryhpy[MAX_HAZPOINTS];
GtkWidget	*comboboxPumpInterval, *comboboxPumpDuration;
GtkWidget	*comboboxtext1;
GtkWidget	*spinnerPumpStatus;
GtkWidget 	*window_wptrack, *image_wptrack, *window_wpselection; 
GtkWidget 	*cbwp1, *cbwp2, *cbwp3, *cbwp4, *cbwp5, *cbwp6, *cbwp7, *cbwp8, *cbwp9, *cbwp10;
GtkWidget 	*cbwp11, *cbwp12, *cbwp13, *cbwp14, *cbwp15, *cbwp16, *cbwp17, *cbwp18, *cbwp19, *cbwp20;

GtkWidget   *windowToolbar, *label25, *label28;

GtkToolButton *tblogtog;
GtkToolButton *tbpumptog;
GtkToolButton *tbdraintog;

GtkWidget   *window_weather;
GtkWidget   *label36, *label37, *label38, *label39, *label40, *label41;

GtkWidget   *windowRate;
GtkWidget   *cbrate1, *cbrate2, *cbrate3, *cbrate4, *cbrate5; 
GtkWidget   *buttonRateSubmit;

int IntervalSeconds[6] = { 60, 120, 180, 300, 600, 900 };
int DurationSeconds[6] = { 60, 120, 180, 300, 600, 900 };

int AerateTogFlag = 0;
int AerateStatus = 0;
int LiveWellState = ALL_STOP;
int Draining = 0;

time_t tAerate;
time_t tLower;
time_t tOnPoint;
time_t tPrevWeatherFile, tWeatherFile;

int Interval, Duration, PumpStatus, DrainStatus, Rating;
int run, mode, status, x, y, GPSDataAvail, LogToggle, LogFlag;
double heading;
double longitude;  
double latitude; 
double speed, HeadingToWP;
unsigned char ImAliveTicker;
int sg0, sg1, current_mapix, SelTrackWP = 999;
double Altitude, Hdop;
long sPressure;
float cPressure, Temperature, dsTemperature;

FILE *PiscesFILE_Handle = NULL;
FILE *PiscesLogFILE_Handle = NULL;
char CurrentMapDatFileName[128];
char CurrentWaypointsFileName[128];
char CurrentHazpointsFileName[128];   
char CurrentLogFileName[128];
char jpgFileName[128];

int MapWidth, MapHeight;
double LongitudeLeft, LongitudeRight, LatitudeBottom;	
double DistanceToWaypointm, DistanceToWaypointf;

long lng0;
float AvailableSpace;
int GetSpaceTic = 0;

FILE *temperatureFile;
float T;
int reftc = 0;
sem_t GpsReadSem;

unsigned int t[2];

float AmbTemp;
float Humidity;
float WindSpeed;
float WindDirection;
float CloudsPercent;
float WeatherCondition;
char  WeatherCondString[64];
char  WindDirectionString[5];

char *i2c_device = "/dev/i2c-1";
int address = 0x77;
void *bmp;
int bmptc = 0;

struct {
	int ID_Number;
	int x;
	int y;
	double lat;
	double lon;
} WayPoints[ MAX_WAYPOINTS ];

struct {
	int ID_Number;
	int x;
	int y;
	double lat;
	double lon;
} HazPoints[ MAX_HAZPOINTS ];

// GPS receiver mode
const char * mode_string[] = {
	"ZERO",					// mode update not seen yet 
	"NO_FIX",				// none
	"2D", 					// good for latitude/longitude 
	"3D"					// good for altitude/climb too 
};

// GPS fix status
const char * status_string[] = {
	"NO_FIX",				// no
	"FIX",					// yes, without DGPS 
	"DGPS_FIX"				// yes, with DGPS 
};


const unsigned int CompassToPolar[360] = { 
	90, 89, 88, 87, 86, 85, 84, 83, 82, 81,
	80, 79, 78, 77, 76, 75, 74, 73, 72, 71,
	70, 69, 68, 67, 66, 65, 64, 63, 62, 61,
	60, 59, 58, 57, 56, 55, 54, 53, 52, 51,
	50, 49, 48, 47, 46, 45, 44, 43, 42, 41,
	40, 39, 38, 37, 36, 35, 34, 33, 32, 31,
	30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
	20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
	10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 359,
	358, 357, 356, 355, 354, 353, 352, 351,
	350, 349, 348, 347, 346, 345, 344, 343,
	342, 341, 340, 339, 338, 337, 336, 335,
	334, 333, 332, 331, 330, 329, 328, 327,
	326, 325, 324, 323, 322, 321, 320, 319,
	318, 317, 316, 315, 314, 313, 312, 311,
	310, 309, 308, 307, 306, 305, 304, 303,
	302, 301, 300, 299, 298, 297, 296, 295,
	294, 293, 292, 291, 290, 289, 288, 287,
	286, 285, 284, 283, 282, 281, 280, 279,
	278, 277, 276, 275, 274, 273, 272, 271,
	270, 271, 270, 269, 268, 267, 266, 265,
	264, 263, 262, 261, 260, 259, 258, 257,
	256, 255, 254, 253, 252, 251, 250, 249,
	248, 247, 246, 245, 244, 243, 242, 241,
	240, 239, 238, 237, 236, 235, 234, 233,
	232, 231, 230, 229, 228, 227, 226, 225,
	224, 223, 222, 221, 220, 219, 218, 217,
	216, 215, 214, 213, 212, 211, 210, 209,
	208, 207, 206, 205, 204, 203, 202, 201,
	200, 199, 198, 197, 196, 195, 194, 193,
	192, 191, 190, 189, 188, 187, 186, 185,
	184, 183, 182, 179, 178, 177, 176, 175,
	174, 173, 172, 171, 170, 169, 168, 167,
	166, 165, 164, 163, 162, 161, 160, 159,
	158, 157, 156, 155, 154, 153, 152, 151,
	150, 149, 148, 147, 146, 145, 144, 143,
	142, 141, 140, 139, 138, 137, 136, 135,
	134, 133, 132, 131, 130, 129, 128, 127,
	126, 125, 124, 123, 122, 121, 120, 119,
	118, 117, 116, 115, 114, 113, 112, 111,
	110, 109, 108, 107, 106, 105, 104, 103,
	102, 101, 100, 99, 98, 97, 96, 95, 94,
	93, 92, 91
};

pthread_t t_GetGpsData_Thread = ( pthread_t )NULL;
pthread_attr_t attr1;
int GetGpsData_Thread( void * tdata );

pthread_t t_UpdateGUI_Thread = ( pthread_t )NULL;
pthread_attr_t attr2;
int UpdateGUI_Thread( void * tdata );

pthread_t t_Blinker_Thread = ( pthread_t )NULL;
pthread_attr_t attr3;
int Blinker_Thread( void * tdata );

pthread_t t_WaterTemp_Thread = ( pthread_t )NULL;
pthread_attr_t attr4;
int WaterTemp_Thread( void * tdata );

gboolean UpdateDataLabels( gpointer data );
void convertGeoToPixel( double lat, double lon );
double haversine_mi( double lat1, double long1, double lat2, double long2 );
void QuitProgram( void );
gboolean ShowOptions( gpointer data );
gboolean ShowAbout( gpointer data );
gboolean ShowWaypoints( gpointer data );
void readLakeData( void );
void writeLakeData( void );
gboolean UpdateMapSetupLabels( gpointer data );
void SaveWaypoints( void );
void SetWaypoint( void );
void LoadWaypointsIntoEntryBoxes( void );
void DeleteWaypoint( GtkButton * button, gpointer data );
void SaveHazpoints( void );
void SetHazpoint( void );
void LoadHazpointsIntoEntryBoxes( void );
void DeleteHazpoint( GtkButton * button, gpointer data );
void ManagePump( void );
gboolean TogglePumpEnable( gpointer data );

void LiveWellAllStop( void );
void LiveWellRaiseLevel( void );
void LiveWellLowerLevel( void );
void LiveWellAerate( void );

gboolean DrainToggle( gpointer data );
void MakePhasePlt( double degdata, double disdata );
gboolean ShowWPTrack( gpointer data );
gboolean ShowToolbar( gpointer data );
gboolean UpdateWPTrack( gpointer data );
gboolean ShowWPSelection( gpointer data );
double FindHeadingToWaypoint( double clat1, double clon1, double wplat2, double wplon2 );
long GetAvailableSpace( const char* path );
void InsertLogfileEntry( void );
gboolean Select_Logfile( gpointer data );
unsigned int Microseconds(void);
void ParseOWM_XML(char *fileName);
gboolean ShowWeather( gpointer data );
void FindWeatherString( int wn, char *p );
void DegreeToCardinal( float deg, char *p );
gboolean Submit_Rating( gpointer data );

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Main                                                                                                //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main( int argc, char *argv[] ) {
	
	int t;
	char name[25], nbr[5];
				
	if( wiringPiSetup() == -1 ) {		
		return(0);	
	}
		
	run = TRUE;

    gtk_init( &argc, &argv );		
    builder = gtk_builder_new( );	
    gtk_builder_add_from_file( builder, "pinav.glade", NULL );
 
    window_main          = GTK_WIDGET( gtk_builder_get_object( builder, "window_main" ) );
	window_about         = GTK_WIDGET( gtk_builder_get_object( builder, "window_about" ) );
	window_options       = GTK_WIDGET( gtk_builder_get_object( builder, "window_options" ) );
	window_waypoints     = GTK_WIDGET( gtk_builder_get_object( builder, "window_waypoints" ) );
    label_speed          = GTK_WIDGET( gtk_builder_get_object( builder, "label_speed" ) );
    label_heading        = GTK_WIDGET( gtk_builder_get_object( builder, "label_heading" ) );
    label_latitude       = GTK_WIDGET( gtk_builder_get_object( builder, "label_latitude" ) );
    label_longitude      = GTK_WIDGET( gtk_builder_get_object( builder, "label_longitude"  ));    
    label_time           = GTK_WIDGET( gtk_builder_get_object( builder, "label_time" ) );
    image_topo           = GTK_WIDGET( gtk_builder_get_object( builder, "image_topo" ) );
	levelbar2            = GTK_WIDGET( gtk_builder_get_object( builder, "levelbar2" ) );
	entry_mapwidth       = GTK_WIDGET( gtk_builder_get_object( builder, "entry_mapwidth" ) );
	entry_mapheight      = GTK_WIDGET( gtk_builder_get_object( builder, "entry_mapheight" ) );
	entry_lonleft        = GTK_WIDGET( gtk_builder_get_object( builder, "entry_lonleft" ) );
	entry_lonright       = GTK_WIDGET( gtk_builder_get_object( builder, "entry_lonright" ) );
	entry_latbottom      = GTK_WIDGET( gtk_builder_get_object( builder, "entry_latbottom" ) );
	checkbutton_way      = GTK_WIDGET( gtk_builder_get_object( builder, "checkbutton_way" ) );
	checkbutton_haz      = GTK_WIDGET( gtk_builder_get_object( builder, "checkbutton_haz" ) );	
	comboboxPumpInterval = GTK_WIDGET( gtk_builder_get_object( builder, "comboboxPumpInterval") );
	comboboxPumpDuration = GTK_WIDGET( gtk_builder_get_object( builder, "comboboxPumpDuration") );	
	spinnerPumpStatus    = GTK_WIDGET( gtk_builder_get_object( builder, "spinner1") );	
	window_wptrack		 = GTK_WIDGET( gtk_builder_get_object( builder, "window_wptrack" ) );	
	image_wptrack        = GTK_WIDGET( gtk_builder_get_object( builder, "image_wptrack" ) );	
	label7               = GTK_WIDGET( gtk_builder_get_object( builder, "label7" ) );
	label2               = GTK_WIDGET( gtk_builder_get_object( builder, "label2" ) );
	labelCPUtemp         = GTK_WIDGET( gtk_builder_get_object( builder, "labelCPUtemp" ) );
	window_wpselection	 = GTK_WIDGET( gtk_builder_get_object( builder, "window_wpselection" ) );	
	cbwp1                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp1" ) );
	cbwp2                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp2" ) );
	cbwp3                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp3" ) );
	cbwp4                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp4" ) );
	cbwp5                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp5" ) );
	cbwp6                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp6" ) );
	cbwp7                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp7" ) );
	cbwp8                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp8" ) );
	cbwp9                = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp9" ) );
	cbwp10               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp10" ) );
	cbwp11               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp11" ) );
	cbwp12               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp12" ) );
	cbwp13               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp13" ) );
	cbwp14               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp14" ) );
	cbwp15               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp15" ) );
	cbwp16               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp16" ) );
	cbwp17               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp17" ) );
	cbwp18               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp18" ) );
	cbwp19               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp19" ) );
	cbwp20               = GTK_WIDGET( gtk_builder_get_object( builder, "cbwp20" ) );
	
	windowToolbar        = GTK_WIDGET( gtk_builder_get_object( builder, "windowToolbar" ) );
	tblogtog             = (GtkToolButton *)( gtk_builder_get_object( builder, "tblogtog" ) );
	tbpumptog            = (GtkToolButton *)( gtk_builder_get_object( builder, "tbpumptog" ) );
	tbdraintog           = (GtkToolButton *)( gtk_builder_get_object( builder, "tbdraintog" ) );
	label25              = GTK_WIDGET( gtk_builder_get_object( builder, "label25" ) );
	label28              = GTK_WIDGET( gtk_builder_get_object( builder, "label28" ) );
	
	window_weather       = GTK_WIDGET( gtk_builder_get_object( builder, "window_weather" ) );
	label36              = GTK_WIDGET( gtk_builder_get_object( builder, "label36" ) );
	label37              = GTK_WIDGET( gtk_builder_get_object( builder, "label37" ) );
	label38              = GTK_WIDGET( gtk_builder_get_object( builder, "label38" ) );
	label39              = GTK_WIDGET( gtk_builder_get_object( builder, "label39" ) );
	label40              = GTK_WIDGET( gtk_builder_get_object( builder, "label40" ) );
	label41              = GTK_WIDGET( gtk_builder_get_object( builder, "label41" ) );
	
	windowRate           = GTK_WIDGET( gtk_builder_get_object( builder, "windowRate" ) );
	cbrate1         	 = GTK_WIDGET( gtk_builder_get_object( builder, "cbrate1" ) );
	cbrate2              = GTK_WIDGET( gtk_builder_get_object( builder, "cbrate2" ) );
	cbrate3              = GTK_WIDGET( gtk_builder_get_object( builder, "cbrate3" ) );
	cbrate4              = GTK_WIDGET( gtk_builder_get_object( builder, "cbrate4" ) );
	cbrate5              = GTK_WIDGET( gtk_builder_get_object( builder, "cbrate5" ) );
	buttonRateSubmit	 = GTK_WIDGET( gtk_builder_get_object( builder, "buttonRateSubmit" ) );
	
	gtk_tool_button_set_label( (GtkToolButton *)tbpumptog, "LW ON");
	pinMode( 28, OUTPUT );  																									
	digitalWrite( 28, LOW );	
	Interval = 0;
	Duration = 0;
	
	// Float switch input
	pinMode( 25, INPUT );  

	// Pump relays
	pinMode( 28, OUTPUT );  
	pinMode( 29, OUTPUT );  
	LiveWellAllStop(); 
	
	// Scope debug points
	pinMode( 22, OUTPUT );  
	pinMode( 24, OUTPUT );  
	digitalWrite( 22, LOW );
	digitalWrite( 24, LOW );
	
	// Buzzer OFF
	pinMode( 26, OUTPUT );  																									
	digitalWrite( 26, HIGH );
	usleep(250000);	
	digitalWrite( 26, LOW );
	
	// LED
	pinMode( 24, OUTPUT );  																									
	LED_D1_OFF;		
			
	gtk_tool_button_set_label( (GtkToolButton *)tbdraintog, "LWD ON");
	
	DrainStatus = 1;
			
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpInterval, 0, "1-Minute");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpInterval, 1, "2-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpInterval, 2, "3-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpInterval, 3, "5-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpInterval, 4, "10-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpInterval, 5, "15-Minutes");	

	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpDuration, 0, "1-Minute");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpDuration, 1, "2-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpDuration, 2, "3-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpDuration, 3, "5-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpDuration, 4, "10-Minutes");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxPumpDuration, 5, "15-Minutes");	
    
	gtk_combo_box_set_active( (GtkComboBox *)comboboxPumpInterval, Interval);
	gtk_combo_box_set_active( (GtkComboBox *)comboboxPumpDuration, Duration);

	comboboxtext1  = GTK_WIDGET( gtk_builder_get_object( builder, "comboboxtext1") );
	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 0, "Worster Lake, North Liberty IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 1, "My Home Town, Highland IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 2, "Salamonie (Beach Cove), Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 3, "Salamonie (Central) Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 4, "Salamonie (Corps Ramp) Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 5, "Salamonie (Dora Ramp) Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 6, "Salamonie (Hope Ramp) Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 7, "Salamonie (North Half) Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 8, "Cagles Mill (South) Cloverdale IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 9, "Cagles Mill (North) Cloverdale IN");	
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 10, "sally_dora_new_holland_area, Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 11, "L&J Technologies Area, Hillside IL");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 12, "Salamonie (Entire Lake) Andrews IN");
	gtk_combo_box_text_insert_text( (GtkComboBoxText *)comboboxtext1, 13, "Salamonie (Monument) Andrews IN");
	
	gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 0);	

	LogToggle = 0;
	LogFlag = 0;	
	gtk_tool_button_set_label( (GtkToolButton *)tblogtog, "Log ON");
	
	for(t = 0; t < MAX_WAYPOINTS; t++ ) {
		strcpy( name, "entrywpid" );		
		sprintf( nbr, "%d", t );
		strcat(name, nbr);
		entrywpid[t] = GTK_WIDGET( gtk_builder_get_object( builder, name ) );				
	}	
	
	for(t = 0; t < MAX_WAYPOINTS; t++ ) {
		strcpy( name, "entrywpx" );		
		sprintf( nbr, "%d", t );
		strcat(name, nbr);
		entrywpx[t] = GTK_WIDGET( gtk_builder_get_object( builder, name ) );				
	}

	for(t = 0; t < MAX_WAYPOINTS; t++ ) {
		strcpy( name, "entrywpy" );		
		sprintf( nbr, "%d", t );
		strcat(name, nbr);
		entrywpy[t] = GTK_WIDGET( gtk_builder_get_object( builder, name ) );				
	}	
	
	for(t = 0; t < MAX_HAZPOINTS; t++ ) {
		strcpy( name, "entryhpid" );		
		sprintf( nbr, "%d", t );
		strcat(name, nbr);
		entryhpid[t] = GTK_WIDGET( gtk_builder_get_object( builder, name ) );				
	}	
	
	for(t = 0; t < MAX_HAZPOINTS; t++ ) {
		strcpy( name, "entryhpx" );		
		sprintf( nbr, "%d", t );
		strcat(name, nbr);
		entryhpx[t] = GTK_WIDGET( gtk_builder_get_object( builder, name ) );				
	}

	for(t = 0; t < MAX_HAZPOINTS; t++ ) { 
		strcpy( name, "entryhpy" );		
		sprintf( nbr, "%d", t );
		strcat(name, nbr);
		entryhpy[t] = GTK_WIDGET( gtk_builder_get_object( builder, name ) );				
	}	
	
	readLakeData();
		
	GPSDataAvail = FALSE;
	
    gtk_builder_connect_signals( builder, NULL );	
	g_object_unref(builder);
	
	gtk_window_set_resizable( (GtkWindow *)window_main, FALSE);
    gtk_widget_show( window_main );
	gtk_widget_hide( label7 );
   	gtk_window_set_keep_above( (GtkWindow *)window_wptrack, TRUE);
	gtk_window_set_position( (GtkWindow*)window_about, GTK_WIN_POS_CENTER);
   	gtk_widget_show( window_about );
	
	gtk_window_set_resizable( (GtkWindow *)windowToolbar, FALSE);
	gtk_window_set_keep_above( (GtkWindow *)windowToolbar, TRUE);
	
	pixbTopo = gdk_pixbuf_new_from_file_at_size( jpgFileName, MapWidth, MapHeight, NULL );  
	pixbBoatSymbol = gdk_pixbuf_new_from_file_at_size( "BoatMark.jpg", 12, 12, NULL );		
		
	// Read in the way point pins
	pixbway1   = gdk_pixbuf_new_from_file_at_size( "map_pin1.jpg",   11, 14, NULL );	
	pixbway2   = gdk_pixbuf_new_from_file_at_size( "map_pin2.jpg",   11, 14, NULL );	
	pixbway3   = gdk_pixbuf_new_from_file_at_size( "map_pin3.jpg",   11, 14, NULL );	
	pixbway4   = gdk_pixbuf_new_from_file_at_size( "map_pin4.jpg",   11, 14, NULL );	
	pixbway5   = gdk_pixbuf_new_from_file_at_size( "map_pin5.jpg",   11, 14, NULL );	
	pixbway6   = gdk_pixbuf_new_from_file_at_size( "map_pin6.jpg",   11, 14, NULL );	
	pixbway7   = gdk_pixbuf_new_from_file_at_size( "map_pin7.jpg",   11, 14, NULL );	
	pixbway8   = gdk_pixbuf_new_from_file_at_size( "map_pin8.jpg",   11, 14, NULL );	
	pixbway9   = gdk_pixbuf_new_from_file_at_size( "map_pin9.jpg",   11, 14, NULL );	
	pixbway10  = gdk_pixbuf_new_from_file_at_size( "map_pin10.jpg",  11, 14, NULL );		
	pixbway11  = gdk_pixbuf_new_from_file_at_size( "map_pin11.jpg",  11, 14, NULL );	
	pixbway12  = gdk_pixbuf_new_from_file_at_size( "map_pin12.jpg",  11, 14, NULL );	
	pixbway13  = gdk_pixbuf_new_from_file_at_size( "map_pin13.jpg",  11, 14, NULL );	
	pixbway14  = gdk_pixbuf_new_from_file_at_size( "map_pin14.jpg",  11, 14, NULL );	
	pixbway15  = gdk_pixbuf_new_from_file_at_size( "map_pin15.jpg",  11, 14, NULL );	
	pixbway16  = gdk_pixbuf_new_from_file_at_size( "map_pin16.jpg",  11, 14, NULL );	
	pixbway17  = gdk_pixbuf_new_from_file_at_size( "map_pin17.jpg",  11, 14, NULL );	
	pixbway18  = gdk_pixbuf_new_from_file_at_size( "map_pin18.jpg",  11, 14, NULL );	
	pixbway19  = gdk_pixbuf_new_from_file_at_size( "map_pin19.jpg",  11, 14, NULL );	
	pixbway20  = gdk_pixbuf_new_from_file_at_size( "map_pin20.jpg",  11, 14, NULL );	
	
	// Read in the hazard pins
	pixbhaz1   = gdk_pixbuf_new_from_file_at_size( "hazard1.jpg",    11, 14, NULL );	
	pixbhaz2   = gdk_pixbuf_new_from_file_at_size( "hazard2.jpg",    11, 14, NULL );	
	pixbhaz3   = gdk_pixbuf_new_from_file_at_size( "hazard3.jpg",    11, 14, NULL );	
	pixbhaz4   = gdk_pixbuf_new_from_file_at_size( "hazard4.jpg",    11, 14, NULL );	
	pixbhaz5   = gdk_pixbuf_new_from_file_at_size( "hazard5.jpg",    11, 14, NULL );	
	pixbhaz6   = gdk_pixbuf_new_from_file_at_size( "hazard6.jpg",    11, 14, NULL );	
	pixbhaz7   = gdk_pixbuf_new_from_file_at_size( "hazard7.jpg",    11, 14, NULL );	
	pixbhaz8   = gdk_pixbuf_new_from_file_at_size( "hazard8.jpg",    11, 14, NULL );	
	pixbhaz9   = gdk_pixbuf_new_from_file_at_size( "hazard9.jpg",    11, 14, NULL );	
	pixbhaz10  = gdk_pixbuf_new_from_file_at_size( "hazard10.jpg",   11, 14, NULL );		
	pixbhaz11  = gdk_pixbuf_new_from_file_at_size( "hazard11.jpg",   11, 14, NULL );	
	pixbhaz12  = gdk_pixbuf_new_from_file_at_size( "hazard12.jpg",   11, 14, NULL );	
	pixbhaz13  = gdk_pixbuf_new_from_file_at_size( "hazard13.jpg",   11, 14, NULL );	
	pixbhaz14  = gdk_pixbuf_new_from_file_at_size( "hazard14.jpg",   11, 14, NULL );	
	pixbhaz15  = gdk_pixbuf_new_from_file_at_size( "hazard15.jpg",   11, 14, NULL );	
	pixbhaz16  = gdk_pixbuf_new_from_file_at_size( "hazard16.jpg",   11, 14, NULL );	
	pixbhaz17  = gdk_pixbuf_new_from_file_at_size( "hazard17.jpg",   11, 14, NULL );	
	pixbhaz18  = gdk_pixbuf_new_from_file_at_size( "hazard18.jpg",   11, 14, NULL );	
	pixbhaz19  = gdk_pixbuf_new_from_file_at_size( "hazard19.jpg",   11, 14, NULL );	
	pixbhaz20  = gdk_pixbuf_new_from_file_at_size( "hazard20.jpg",   11, 14, NULL );	
	
	bmp = bmp180_init( address, i2c_device );
	
	bmp180_eprom_t eprom;
	bmp180_dump_eprom( bmp, &eprom );		
	bmp180_set_oss( bmp, 3 );
	
	sem_init( &GpsReadSem, 0, 1 );	
	
   	t = 0; 
    pthread_attr_init( &attr1 );
 	pthread_attr_setdetachstate( &attr1, PTHREAD_CREATE_JOINABLE );  	    
    pthread_create( &t_GetGpsData_Thread, &attr1, ( void * ( * )( void* ) )&GetGpsData_Thread, ( void * )t );     	
   	
	pthread_attr_init( &attr2 );
 	pthread_attr_setdetachstate( &attr2, PTHREAD_CREATE_JOINABLE ); 
    pthread_create( &t_UpdateGUI_Thread, &attr2, ( void * ( * )( void* ) )&UpdateGUI_Thread, ( void * )t ); 
	
	pthread_attr_init( &attr3 );
 	pthread_attr_setdetachstate( &attr3, PTHREAD_CREATE_JOINABLE ); 
    pthread_create( &t_Blinker_Thread, &attr3, ( void * ( * )( void* ) )&Blinker_Thread, ( void * )t ); 
	
	pthread_attr_init( &attr4 );
 	pthread_attr_setdetachstate( &attr4, PTHREAD_CREATE_JOINABLE ); 
    pthread_create( &t_WaterTemp_Thread, &attr4, ( void * ( * )( void* ) )&WaterTemp_Thread, ( void * )t ); 
	
	gtk_main( );
	
	return EXIT_SUCCESS;
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Receive and Parse the GPS Data Stream                                                               //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int GetGpsData_Thread( void * tdata ) {
	
	int rc;   
	struct gps_data_t gps_data;
	
	long lng0;
	float flat, flon;
	
	if( ( rc = gps_open( "localhost", "2947", &gps_data ) ) == -1 ) {
		run = FALSE;		
		return EXIT_FAILURE;
	}	
	gps_stream( &gps_data, WATCH_ENABLE | WATCH_JSON, NULL ); 
		
	while( run ) {  
		
		ImAliveTicker++;
		if( ImAliveTicker > 50 ) ImAliveTicker = 0;  
		
		// Wait for up to 2 seconds to receive data 
		if( gps_waiting ( &gps_data, 20000000 ) ) {
								
			// Read data
			if( ( rc = gps_read( &gps_data ) ) == -1 ) {
				
				gps_stream( &gps_data, WATCH_DISABLE, NULL );
				gps_close( &gps_data );
			
				if( ( rc = gps_open( "localhost", "2947", &gps_data ) ) == -1 ) {
					run = FALSE;
					continue;
				}
				gps_stream( &gps_data, WATCH_ENABLE | WATCH_JSON, NULL );  
			
			} else {
				
				// Display data from the GPS receiver
				if( ( gps_data.status == STATUS_FIX ) && ( gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D ) &&
					!isnan( gps_data.fix.latitude ) && !isnan( gps_data.fix.longitude ) ) {
						
					// Convert speed from GPS receiver from meters/sec to miles per hour 
					if( !isnan( gps_data.fix.speed ) ) {
						speed = gps_data.fix.speed;											
						speed *= 2.23693629;
					}
					
					if( !isnan( gps_data.fix.track ) ) heading = gps_data.fix.track;
						
					latitude = gps_data.fix.latitude;
					longitude = gps_data.fix.longitude;
					
					Altitude = gps_data.fix.altitude;
						
				//	lng0 = (long)(latitude * 1000);
				//	flat = latitude - ( (float)lng0 / 1000 );
				//	lng0 = (long)(longitude * 1000);
				//	flon = longitude - ( (float)lng0 / 1000 );
				//	latitude = 41.548327 + flat;
				//	longitude = -86.357399 + flon; 
										
				//	latitude = 40.784134; longitude = -85.654885; 	
					
					GPSDataAvail = TRUE; 							
					
				} else {										
					GPSDataAvail = FALSE;					
				}
				mode   = gps_data.fix.mode;
				status = gps_data.status; 	

				sem_post( &GpsReadSem );		
			}						
		} else {
			
			gps_stream( &gps_data, WATCH_DISABLE, NULL );
			gps_close( &gps_data );
		
			if( ( rc = gps_open( "localhost", "2947", &gps_data ) ) == -1 ) {
				run = FALSE;
				continue;
			}
			gps_stream( &gps_data, WATCH_ENABLE | WATCH_JSON, NULL ); 
		}	
	}	
	gps_stream( &gps_data, WATCH_DISABLE, NULL );
	gps_close( &gps_data );
	return EXIT_SUCCESS;		
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Update all the widgets                                                                              //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int UpdateGUI_Thread( void * tdata ) { 
	
	int *data;
	double flt0;
	int h;
	time_t oldMTime;
	struct stat file_stat;
	
	static double plat, plon, pwplat, pwplon;
	
	data = 0;
	
	while( run ) {

		sem_wait( &GpsReadSem );  
		
		usleep(1); 								// Yield to OS
		t[0] = Microseconds();
		
		gdk_threads_add_idle( UpdateDataLabels, data );		   
		
		if( SelTrackWP != 999 ) {	  		
									
			if( plat != latitude || plon != longitude || pwplat != WayPoints[SelTrackWP].lat || pwplon != WayPoints[SelTrackWP].lon ) {						 
									
				HeadingToWP = FindHeadingToWaypoint( latitude, longitude, WayPoints[SelTrackWP].lat, WayPoints[SelTrackWP].lon );
				DistanceToWaypointm = haversine_mi( latitude, longitude, WayPoints[SelTrackWP].lat, WayPoints[SelTrackWP].lon );	
				
				plat = latitude;
				plon = longitude;
				pwplat = WayPoints[SelTrackWP].lat;
				pwplon = WayPoints[SelTrackWP].lon;
			}
		} else 	
			DistanceToWaypointm = 0.00378788;
				
		// convert miles to feet
		DistanceToWaypointf = DistanceToWaypointm * 5280;
	
		if( DistanceToWaypointf == 0 ) DistanceToWaypointf = 0.5;
				
		if(HeadingToWP > 359) HeadingToWP = 359; 
		if(HeadingToWP < 0 ) HeadingToWP = 0;
			
		flt0 = fabs(HeadingToWP);	
		h = (int)flt0;				
		flt0 = (double)CompassToPolar[h];
				
		if( DistanceToWaypointf <= 20 )
			MakePhasePlt( flt0, DistanceToWaypointf );
		else 								
			MakePhasePlt( flt0, 20 );
		
		
		stat( "/home/pi/piNav/phasedata.jpg", &file_stat);
		oldMTime = file_stat.st_mtime;
		system("gnuplot phasedata.plt");
		
		int trycnt = 5; 
		while( trycnt ) {				
			stat( "/home/pi/piNav/phasedata.jpg", &file_stat);	 
			if( file_stat.st_mtime > oldMTime ) break;
			else {
				trycnt--;	
			}	
			usleep(10000);
		}		
		gdk_threads_add_idle( UpdateWPTrack, data );	

	//	t[1] = Microseconds();
	//	printf("UpdateGUI_Thread took %d us\n", t[1]-t[0]);
		
	}		
	return EXIT_SUCCESS;	
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// LED Blinker                                                                                         //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int Blinker_Thread( void * tdata ) {
	
	static int tog0 = 1;   
	static int cdtmr0 = 0;	
	static int cdtmr1 = 0;	
	
	while( run ) { 
		
		usleep( 1000 );
		
		if( cdtmr0 ) cdtmr0--;   
		else {
			tog0 ^= 1;
			if( SelTrackWP != 999 ){ 											
				if( DistanceToWaypointf <= 5 ) cdtmr0 = 80;
				else if( DistanceToWaypointf <= 10 ) cdtmr0 = 300;
				else if( DistanceToWaypointf <= 15 ) cdtmr0 = 500;
				else if( DistanceToWaypointf <= 20 ) cdtmr0 = 1000;
				else tog0 = 0;	
			}
			else tog0 = 0;		
			
			if( tog0 ) { 
				LED_D1_ON;
			} else {
				LED_D1_OFF;
			}			
		}
		
		if( cdtmr1 ) cdtmr1--;   
		else {
			InsertLogfileEntry();
			cdtmr1 = 10000;
		}
	}
	return EXIT_SUCCESS;
}
	
	
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Water Temp Thread (DS18B20)                                                                         //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int WaterTemp_Thread( void * tdata ) {

	FILE * owmd;	
	int g, numDev, i;	
	DIR *dir;
	struct dirent *dirent;
	char devPath[128]; 						// Path to device
	char buf[256];     						// Data from device
	char tmpData[6];   						// Temp C * 1000 reported by device 
	char path[] = "/sys/bus/w1/devices"; 
	ssize_t numRead;
 	float tempC;
   	float tempF[5];
	struct stat file_stat;
	 
 	// Dallas DS18B20 1-Wire temperature sensors
	struct ID {
		char dev[16];      					// Dev ID
	} devID[5];
 
	dir = opendir(path);
	
	if(dir != NULL) {
		numDev = g = 0;
 		while((dirent = readdir(dir))) {
 		
			// 1-wire devices are links beginning with 28-
			if(dirent->d_type == DT_LNK && strstr(dirent->d_name, "28-") != NULL) { 
				strcpy(&devID[g].dev[0], dirent->d_name);
				printf("\nDevice: %s\n", &devID[g++].dev[0]);	
			}			
		}
		numDev = g;
		printf("\nNumber of family 28 devices found: %d\n\n", numDev);
		(void) closedir(dir);
    } else {
  		perror ("Couldn't open the w1 devices directory");
		run = 0;    	
	}
	
	usleep(100);
	
	// Read temp continuously 
	//   Opening the device's file triggers new reading
	while(run) {
		
		owmd = fopen( "/home/pi/piNav/CurGeoLoc.dat", "w");			
		fprintf( owmd, "%0.2f,%0.2f\n", latitude, longitude );
		fclose(owmd);	
		
		stat( "/home/pi/piNav/weather.xml", &file_stat);
		tWeatherFile = file_stat.st_mtime;
		
		// Beep to indicate OWM is running and retrieving XML weather file 
		if( difftime(tWeatherFile, tPrevWeatherFile) ) {	
			tPrevWeatherFile = tWeatherFile;		
			digitalWrite( 26, HIGH );
			usleep(100000);	
			digitalWrite( 26, LOW );
			ParseOWM_XML( "/home/pi/piNav/weather.xml" );
		}
		
		// Keep the FISH ALIVE!
		ManagePump();
		
		for(g = 0; g < numDev; g++ ){ 
	
			// Assemble path to OneWire device
			sprintf(devPath, "%s/%s/w1_slave", path, &devID[g].dev[0]);
			int fd = open(devPath, O_RDONLY);
			
			if(fd == -1) {
	   			perror ("Couldn't open the w1 device.");
	   			run = 0;    	
				continue; 
	  		}
	  		
	  		while((numRead = read(fd, buf, 256)) > 0) {
	  			
	   			strcpy(tmpData, strstr(buf, "t=") + 2); 
	   			
	   			for(i = 0; i < 8; i++ ) 
	   				if(tmpData[i] == 0x0A ) tmpData[i] = 0;
	   			   			   			
	   			tempC = strtof(tmpData, NULL);
	   			tempF[g] = (tempC / 1000) * 9 / 5 + 32;			
	  		}
	  		close(fd);
	  		
	  	}	
	  	dsTemperature = tempF[0];	  	
	  	  		 
	  	usleep(500000);
 	} 
	return EXIT_SUCCESS;
}	
	
	
	
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Prepare the Waypoint Tracking for Display by GTK process                                            //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean UpdateWPTrack( gpointer data ) { 
	
	char s[128];
	
	pixbwptrack = gdk_pixbuf_new_from_file_at_size( "phasedata.jpg", 250, 250, NULL );
		
	// Polar graph for way point tracking		
	gtk_image_set_from_pixbuf( (GtkImage *)image_wptrack, pixbwptrack); 

	g_object_unref( G_OBJECT (pixbwptrack) );	
	
	
	gtk_window_set_title( (GtkWindow *)window_wptrack, "WP TRACKING");
		
	if( WayPoints[0].x && WayPoints[0].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp1, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp1, FALSE);
	if( WayPoints[1].x && WayPoints[1].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp2, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp2, FALSE);
	if( WayPoints[2].x && WayPoints[2].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp3, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp3, FALSE);
	if( WayPoints[3].x && WayPoints[3].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp4, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp4, FALSE);
	if( WayPoints[4].x && WayPoints[4].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp5, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp5, FALSE);
	if( WayPoints[5].x && WayPoints[5].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp6, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp6, FALSE);
	if( WayPoints[6].x && WayPoints[6].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp7, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp7, FALSE);
	if( WayPoints[7].x && WayPoints[7].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp8, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp8, FALSE);
	if( WayPoints[8].x && WayPoints[8].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp9, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp9, FALSE);
	if( WayPoints[9].x && WayPoints[9].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp10, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp10, FALSE);
	if( WayPoints[10].x && WayPoints[10].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp11, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp11, FALSE);
	if( WayPoints[11].x && WayPoints[11].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp12, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp12, FALSE);
	if( WayPoints[12].x && WayPoints[12].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp13, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp13, FALSE);
	if( WayPoints[13].x && WayPoints[13].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp14, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp14, FALSE);
	if( WayPoints[14].x && WayPoints[14].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp15, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp15, FALSE);
	if( WayPoints[15].x && WayPoints[15].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp16, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp16, FALSE);
	if( WayPoints[16].x && WayPoints[16].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp17, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp17, FALSE);
	if( WayPoints[17].x && WayPoints[17].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp18, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp18, FALSE);
	if( WayPoints[18].x && WayPoints[18].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp19, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp19, FALSE);
	if( WayPoints[19].x && WayPoints[19].y ) gtk_widget_set_sensitive((GtkWidget *)cbwp20, TRUE);
	else gtk_widget_set_sensitive((GtkWidget *)cbwp20, FALSE);	
	
	usleep(100);
		
	if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp1 ) ) 	     SelTrackWP = 0;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp2 ) )  SelTrackWP = 1;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp3 ) )  SelTrackWP = 2;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp4 ) )  SelTrackWP = 3;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp5 ) )  SelTrackWP = 4;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp6 ) )  SelTrackWP = 5;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp7 ) )  SelTrackWP = 6;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp8 ) )  SelTrackWP = 7;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp9 ) )  SelTrackWP = 8;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp10 ) ) SelTrackWP = 9;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp11 ) ) SelTrackWP = 10;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp12 ) ) SelTrackWP = 11;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp13 ) ) SelTrackWP = 12;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp14 ) ) SelTrackWP = 13;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp15 ) ) SelTrackWP = 14;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp16 ) ) SelTrackWP = 15;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp17 ) ) SelTrackWP = 16;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp18 ) ) SelTrackWP = 17;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp19 ) ) SelTrackWP = 18;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbwp20 ) ) SelTrackWP = 19;
	else
		SelTrackWP = 999;		
	
	if( SelTrackWP != 999 ) {
		if( DistanceToWaypointf <= 1320 )    // .25 mile
		//	sprintf( s, "WP%02d H:%5.1f  D:%6.1f FT", SelTrackWP+1, HeadingToWP, DistanceToWaypointf ); 
			sprintf( s, "WP%02d D:%.1f FT", SelTrackWP+1, DistanceToWaypointf ); 
		else 
		//	sprintf( s, "WP%02d H:%5.1f  D:%.1f MI",SelTrackWP+1, HeadingToWP, DistanceToWaypointf / 5280 ); 
			sprintf( s, "WP%02d D:%.1f MI", SelTrackWP+1, DistanceToWaypointf / 5280 ); 
		
		gtk_label_set_text( ( GtkLabel * )label7, s );	
		gtk_widget_show( label7 );
	} else gtk_widget_hide( label7 );
	
	return G_SOURCE_REMOVE; 
}


//
///////////////////////////////////////////////////////////////////////////////////////////////////////// 
//                                                                                                     //
// Prepare the Data for Display by GTK process                                                         //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean UpdateDataLabels( gpointer data ) {
		
	char s[128];	
	int g;	

	time_t tt = time(NULL);
	struct tm tm = *localtime(&tt);
		
	sprintf( s,"%4.1f", speed ); 
	gtk_label_set_text( ( GtkLabel * )label_speed, s ); 	
	
	sprintf( s,"%5.1f", heading ); 
	gtk_label_set_text( ( GtkLabel * )label_heading, s );

	sprintf( s,"Lon %.7f   %s%s   ALT %.0f FT %c", longitude, status_string[ status ], mode_string[ mode ], Altitude * 3.28084, PiscesLogFILE_Handle!=NULL?'!':' ' ); 
	gtk_label_set_text( ( GtkLabel * )label_latitude, s );
	
	sprintf( s," Lat %.7f", latitude ); 
	gtk_label_set_text( ( GtkLabel * )label_longitude, s );
			
	sprintf( s, "%02d:%02d", tm.tm_hour, tm.tm_min );
	gtk_label_set_text( ( GtkLabel * )label_time, s );
	
//	usleep(100);
//	if( Microseconds() - t[0] > 190000 ) return G_SOURCE_REMOVE; 		// was 800000
	
	sprintf( s, "%.2f", cPressure );
	gtk_label_set_text( ( GtkLabel * )label25, s );
	
	sprintf( s, "%.1f", dsTemperature );
	gtk_label_set_text( ( GtkLabel * )label28, s );
		
	sprintf( s,"%.1f", AmbTemp ); 
	gtk_label_set_text( ( GtkLabel * )label36, s ); 	
	
	sprintf( s,"%.0f", Humidity ); 
	gtk_label_set_text( ( GtkLabel * )label37, s ); 	
	
	sprintf( s ,"%.0f", WindSpeed ); 
	gtk_label_set_text( ( GtkLabel * )label38, s ); 	
	
	gtk_label_set_text( ( GtkLabel * )label39, WindDirectionString ); 	
	
	sprintf( s,"%.0f", CloudsPercent ); 
	gtk_label_set_text( ( GtkLabel * )label40, s ); 	
	
	int n = (int)WeatherCondition;
    FindWeatherString( n, WeatherCondString );
	
	sprintf( s,"%s", WeatherCondString ); 
	gtk_label_set_text( ( GtkLabel * )label41, s ); 	
	
//	usleep(100);
//	if( Microseconds() - t[0] > 190000 ) return G_SOURCE_REMOVE; 
	
	if( GPSDataAvail == FALSE ) {
		sprintf(s, "PISCES - GPS BOAT NAVIGATION, No Data Available...%03u", ImAliveTicker );	
		gtk_window_set_title( (GtkWindow *)window_main, s);
		
	} else {
		
		GetSpaceTic--;
		if( GetSpaceTic <= 0 ) { 
			lng0 = GetAvailableSpace( "/media/usb" );
			AvailableSpace = (float)lng0 * 32768.0;
			GetSpaceTic = 500;
			sprintf( s,"Thumb Drive Space %.0f : bmpTemp = %.1f", AvailableSpace, Temperature ); 
			gtk_label_set_text( ( GtkLabel * )label2, s );
		}
		
		if( reftc ) reftc--;
		if( reftc == 0 ) {
			reftc = 150;
			temperatureFile = fopen ("/sys/class/thermal/thermal_zone0/temp", "r"); 
			if (temperatureFile == NULL)
				T= -999.9;
			else {
				fscanf (temperatureFile, "%f", &T);
				fclose (temperatureFile);
				T /= 1000;
				T = ( (T * 9.0) / 5 ) + 32.0;	
			}
			sprintf( s,"CPU Temp: %.1f deg F", T ); 
			gtk_label_set_text( ( GtkLabel * )labelCPUtemp, s );
			
		}
		
//		usleep(100);
//		if( Microseconds() - t[0] > 190000 ) return G_SOURCE_REMOVE; 
		
		if( bmptc ) bmptc--;
		if( bmptc == 0 ) {
			bmptc = 150;
			Temperature = bmp180_temperature( bmp );	
			Temperature = Temperature * ( 9.0 / 5.0 ) + 32.0;
			
			// get STATION pressure in pascals
			sPressure = bmp180_pressure( bmp );	
			
			// Convert to millibars
			sPressure = ( sPressure * 0.01 );
			
			// adjust to PRESSURE AT SEA LEVEL using GPS Altitude
			cPressure = sPressure / ( pow( 1.0 - ( Altitude / 44330.0 ), 5.255 ) );
			
			// Convert units to inches of mercury (Inhg)
			cPressure = ( cPressure * 0.02952998751 );		
		}
				
		sprintf( s,"PISCES - GPS BOAT NAVIGATION + WAYPOINT TRACKING" );  
		gtk_window_set_title( (GtkWindow *)window_main, s);
	}
	
//	if( Microseconds() - t[0] > 190000 ) return G_SOURCE_REMOVE; 
	
	gtk_level_bar_set_value( (GtkLevelBar *)levelbar2, ImAliveTicker);
	
	convertGeoToPixel( latitude, longitude );
		
	// Reload the clean copy, raw topo pic from RAM, the boat mark is still resident
	cpixbTopo = gdk_pixbuf_copy(pixbTopo);	
	
	if( ( x >= 0 && x <= MapWidth ) && ( y >= 0 && y <= MapHeight ) ) {

		// Plot the current boat position symbol from GPS position
		gdk_pixbuf_composite( pixbBoatSymbol, cpixbTopo,		// src, dst
							  x-6, y-6,             			// destx, desty
							  12, 12,               			// destwdth, destheight 
							  x-6, y-6,             			// offset_x, offset_y 
							  1, 1,                 			// scale_x, scale_y
							  GDK_INTERP_NEAREST, 255 );		// interp_type, overall_alpha   
							  
		
	} 

	if( gtk_toggle_button_get_active( (GtkToggleButton *)checkbutton_way ) ) {
						
		// Plot the waypoints pins			 
		for( g = 0; g < MAX_WAYPOINTS; g++ ) {
									
			if( WayPoints[g].x && WayPoints[g].y ) {
				
				x = WayPoints[g].x;
				y = WayPoints[g].y;

				switch(g) {
				case  0: cpixbway_num = gdk_pixbuf_copy(pixbway1);   break;	
				case  1: cpixbway_num = gdk_pixbuf_copy(pixbway2);   break;	
				case  2: cpixbway_num = gdk_pixbuf_copy(pixbway3);   break;	
				case  3: cpixbway_num = gdk_pixbuf_copy(pixbway4);   break;	
				case  4: cpixbway_num = gdk_pixbuf_copy(pixbway5);   break;	
				case  5: cpixbway_num = gdk_pixbuf_copy(pixbway6);   break;	
				case  6: cpixbway_num = gdk_pixbuf_copy(pixbway7);   break;	
				case  7: cpixbway_num = gdk_pixbuf_copy(pixbway8);   break;	
				case  8: cpixbway_num = gdk_pixbuf_copy(pixbway9);   break;	
				case  9: cpixbway_num = gdk_pixbuf_copy(pixbway10);  break;
				case 10: cpixbway_num = gdk_pixbuf_copy(pixbway11);  break;	
				case 11: cpixbway_num = gdk_pixbuf_copy(pixbway12);  break;	
				case 12: cpixbway_num = gdk_pixbuf_copy(pixbway13);  break;	
				case 13: cpixbway_num = gdk_pixbuf_copy(pixbway14);  break;	
				case 14: cpixbway_num = gdk_pixbuf_copy(pixbway15);  break;	
				case 15: cpixbway_num = gdk_pixbuf_copy(pixbway16);  break;	
				case 16: cpixbway_num = gdk_pixbuf_copy(pixbway17);  break;	
				case 17: cpixbway_num = gdk_pixbuf_copy(pixbway18);  break;	
				case 18: cpixbway_num = gdk_pixbuf_copy(pixbway19);  break;	
				case 19: cpixbway_num = gdk_pixbuf_copy(pixbway20);  break;							
				}	

				// Overlay the way point pin on the topographic map
				gdk_pixbuf_composite( cpixbway_num, cpixbTopo,   	// src, dst
						  x-5, y-7,                  				// destx, desty 
						  11, 14,                   				// destwdth, destheight 
						  x-5, y-7,                  				// offset_x, offset_y 
						  1, 1,                     				// scale_x, scale_y
						  GDK_INTERP_NEAREST, 255 );				// interp_type, overall_alpha 					
				
				g_object_unref( G_OBJECT (cpixbway_num) );
			}
		}
		
	}

//	usleep(100);
	
	if( gtk_toggle_button_get_active( (GtkToggleButton *)checkbutton_haz ) ) {
		
		// Plot the hazard pins			 
		for( g = 0; g < MAX_HAZPOINTS; g++ ) {
									
			if( HazPoints[g].x && HazPoints[g].y ) {
				
				x = HazPoints[g].x;
				y = HazPoints[g].y; 

				switch(g) {
				case  0: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz1);   break;	
				case  1: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz2);   break;	
				case  2: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz3);   break;	
				case  3: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz4);   break;	
				case  4: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz5);   break;	
				case  5: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz6);   break;	
				case  6: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz7);   break;	
				case  7: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz8);   break;	
				case  8: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz9);   break;	
				case  9: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz10);  break;
				case 10: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz11);  break;	
				case 11: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz12);  break;	
				case 12: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz13);  break;	
				case 13: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz14);  break;	
				case 14: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz15);  break;	
				case 15: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz16);  break;	
				case 16: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz17);  break;	
				case 17: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz18);  break;	
				case 18: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz19);  break;	 
				case 19: cpixbhaz_num = gdk_pixbuf_copy(pixbhaz20);  break;							
				}	

				// Overlay the hazard point pin on the topographic map
				gdk_pixbuf_composite( cpixbhaz_num, cpixbTopo,   	// src, dst
						  x-5, y-7,                  				// destx, desty
						  11, 14,                   				// destwdth, destheight 
						  x-5, y-7,                  				// offset_x, offset_y 
						  1, 1,                     				// scale_x, scale_y
						  GDK_INTERP_NEAREST, 255 );				// interp_type, overall_alpha 					
									
				g_object_unref( G_OBJECT (cpixbhaz_num) );
			}
		}
		
	}	

	gtk_image_set_from_pixbuf( (GtkImage *)image_topo, cpixbTopo);  	
	g_object_unref( G_OBJECT (cpixbTopo) );	

	return G_SOURCE_REMOVE; 
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Determine the x and y coordinates from the geo data                                                 //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void convertGeoToPixel( double lat, double lon ) {

	double mapLonDelta = LongitudeRight - LongitudeLeft;	
	double mapLatBottomDegree = LatitudeBottom * D2R;
	
	x = (lon - LongitudeLeft) * (MapWidth / mapLonDelta); 
	lat = lat * D2R;
	double worldMapWidth = ((MapWidth / mapLonDelta) * 360) / (2 * GPS_PI);   
		
	double mapOffsetY = (worldMapWidth / 2 * log((1 + sin(mapLatBottomDegree)) / (1 - sin(mapLatBottomDegree))));      
	y = MapHeight - ((worldMapWidth / 2 * log((1 + sin(lat)) / (1 - sin(lat)))) - mapOffsetY);
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Determine the distance in miles from one pair to another                                            //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
double haversine_mi( double lat1, double long1, double lat2, double long2 ) { 
	
    double dlong = ( long2 - long1 ) * D2R;
    double dlat = ( lat2 - lat1 ) * D2R;
	double a = pow( sin( dlat / 2.0 ), 2 ) + cos( lat1 * D2R ) * cos( lat2 * D2R) * pow( sin( dlong /2.0 ), 2 );
    double c = 2 * atan2( sqrt( a ), sqrt( 1 - a ) );
    double d = 3956 * c; 

    return d;
}


		
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// Shutdown gracefully                                                                                 //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void QuitProgram( void ) {
		
// 	void *status;
	
	digitalWrite( 26, HIGH );
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	run = FALSE;
	
	if(PiscesLogFILE_Handle != NULL) 
		fclose(PiscesLogFILE_Handle);	
	
//	pthread_join( t_GetGpsData_Thread, &status );	
	pthread_cancel( t_GetGpsData_Thread );	
	pthread_attr_destroy( &attr1 );	
		
//	pthread_join( t_UpdateGUI_Thread, &status );	
	pthread_cancel( t_UpdateGUI_Thread );	
	pthread_attr_destroy( &attr2 );			
	
//	pthread_join( t_Blinker_Thread, &status );
	pthread_cancel( t_Blinker_Thread );		
	pthread_attr_destroy( &attr3 );		
	
//	pthread_join( t_WaterTemp_Thread, &status );			
	pthread_cancel( t_WaterTemp_Thread );	
	pthread_attr_destroy( &attr4 );		
	
	sem_destroy( &GpsReadSem ); 
	
	LiveWellAllStop();
	
	LED_D1_OFF;
	
    gtk_main_quit( );
	
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowOptions( gpointer data ) {
		
	digitalWrite( 26, HIGH );
	
	gtk_window_set_title( (GtkWindow *)window_options, "Pisces Options and Configurations");
	
	gtk_window_present( (GtkWindow*)window_options);	
	
	gtk_window_set_position( (GtkWindow*)window_options, GTK_WIN_POS_CENTER);
	
	UpdateMapSetupLabels( data );
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}	


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowAbout( gpointer data ) {
	
	digitalWrite( 26, HIGH );
		
	gtk_window_set_position( (GtkWindow*)window_about, GTK_WIN_POS_CENTER);
	gtk_window_present( (GtkWindow*)window_about);	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}	


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowWaypoints( gpointer data ) {
		
	digitalWrite( 26, HIGH );
	
	gtk_window_set_position( (GtkWindow*)window_waypoints, GTK_WIN_POS_CENTER);
	gtk_window_present( (GtkWindow*)window_waypoints);	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}	



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void writeLakeData( void ) {
	
	int t;
		
	digitalWrite( 26, HIGH );
	
	current_mapix = gtk_combo_box_get_active( (GtkComboBox *)comboboxtext1 );
	if( current_mapix == 0 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/worster.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 1 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/myhometown.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 2 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/salamonie_beachcove.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 3 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/salamonie_central.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 4 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/salamonie_corpsramp.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 5 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/salamonie_doraramp.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 6 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/salamonie_hoperamp.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 7 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/salamonie_northhalf.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 8 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/caglesmillsouth.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 9 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/caglesmillnorth.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 10 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/sally_dora_new_holland_area.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 11 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/lj_area.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 12 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/sally_entire.dat" );			
		fclose(PiscesFILE_Handle);
	} else if( current_mapix == 13 ) {
		PiscesFILE_Handle = fopen( "config.ini", "w");	
		fprintf( PiscesFILE_Handle, "%s\n", "/home/pi/piNav/sally_monument.dat" );			
		fclose(PiscesFILE_Handle);
	}
	
	MapWidth = atoi( (char *)gtk_entry_get_text( (GtkEntry *)entry_mapwidth ) );
	MapHeight = atoi( (char *)gtk_entry_get_text( (GtkEntry *)entry_mapheight ) );
	LongitudeLeft = atof( (char *)gtk_entry_get_text( (GtkEntry *)entry_lonleft ) );
	LongitudeRight = atof( (char *)gtk_entry_get_text( (GtkEntry *)entry_lonright ) );
	LatitudeBottom = atof( (char *)gtk_entry_get_text( (GtkEntry *)entry_latbottom ) );
	
	PiscesFILE_Handle = fopen( CurrentMapDatFileName, "w");	
	fprintf( PiscesFILE_Handle, "%d,%d,%lf,%lf,%lf\n", MapWidth, MapHeight, LongitudeLeft, LongitudeRight, LatitudeBottom);
	fprintf( PiscesFILE_Handle, "%s\n", jpgFileName );		
	fprintf( PiscesFILE_Handle, "%s\n", CurrentWaypointsFileName );
	fprintf( PiscesFILE_Handle, "%s\n", CurrentHazpointsFileName );
	
	sg0 = gtk_toggle_button_get_active( (GtkToggleButton*)checkbutton_way);
	sg1 = gtk_toggle_button_get_active( (GtkToggleButton*)checkbutton_haz);
	
	fprintf( PiscesFILE_Handle, "%d\n", sg0 );
	fprintf( PiscesFILE_Handle, "%d\n", sg1 );
	fclose(PiscesFILE_Handle);
	
	PiscesFILE_Handle = fopen( CurrentWaypointsFileName, "w");				
	for( t = 0; t < MAX_WAYPOINTS; t++ ) {		
		fprintf( PiscesFILE_Handle, "%d,%d,%d,%lf,%lf\n", WayPoints[t].ID_Number, WayPoints[t].x, WayPoints[t].y, WayPoints[t].lat, WayPoints[t].lon );					
	}		
	fclose(PiscesFILE_Handle);	
	
	PiscesFILE_Handle = fopen( CurrentHazpointsFileName, "w");				
	for( t = 0; t < MAX_HAZPOINTS; t++ ) {		
		fprintf( PiscesFILE_Handle, "%d,%d,%d,%lf,%lf\n", HazPoints[t].ID_Number, HazPoints[t].x, HazPoints[t].y, HazPoints[t].lat, HazPoints[t].lon );					
	}		
	fclose(PiscesFILE_Handle);	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void readLakeData( void ) {
	
	int t;
	char line[64]; 
	
	PiscesFILE_Handle = fopen( "config.ini", "r");	
	fscanf( PiscesFILE_Handle, "%s", CurrentMapDatFileName );			
	fclose(PiscesFILE_Handle);
		
	PiscesFILE_Handle = fopen( CurrentMapDatFileName, "r");			
	fscanf( PiscesFILE_Handle, "%d,%d,%lf,%lf,%lf\n", &MapWidth, &MapHeight, &LongitudeLeft, &LongitudeRight, &LatitudeBottom);
	fscanf( PiscesFILE_Handle, "%s\n", jpgFileName );		
	fscanf( PiscesFILE_Handle, "%s\n", CurrentWaypointsFileName );	
	fscanf( PiscesFILE_Handle, "%s\n", CurrentHazpointsFileName );	
	fscanf( PiscesFILE_Handle, "%d\n", &sg0 );	
	fscanf( PiscesFILE_Handle, "%d\n", &sg1 );	
	fclose( PiscesFILE_Handle);
	
	if( strstr( CurrentMapDatFileName, "worster" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 0 );		
	}  else if( strstr( CurrentMapDatFileName, "myhometown" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 1 );		
	} else if( strstr( CurrentMapDatFileName, "salamonie_beachcove" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 2);		
	} else if( strstr( CurrentMapDatFileName, "salamonie_central" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 3);		
	} else if( strstr( CurrentMapDatFileName, "salamonie_corpsramp" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 4);		
	} else if( strstr( CurrentMapDatFileName, "salamonie_doraramp" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 5);		
	} else if( strstr( CurrentMapDatFileName, "salamonie_hoperamp" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 6);		
	} else if( strstr( CurrentMapDatFileName, "salamonie_northhalf" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 7);		
	} else if( strstr( CurrentMapDatFileName, "caglesmillsouth" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 8);		
	} else if( strstr( CurrentMapDatFileName, "caglesmillnorth" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 9);		
	} else if( strstr( CurrentMapDatFileName, "sally_dora_new_holland_area" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 10);		
	} else if( strstr( CurrentMapDatFileName, "lj_area" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 11);		
	} else if( strstr( CurrentMapDatFileName, "sally_entire" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 12);		
	} else if( strstr( CurrentMapDatFileName, "sally_monument" ) ) {
		gtk_combo_box_set_active( (GtkComboBox *)comboboxtext1, 13);		
	}
	  
	if( sg0 ) gtk_toggle_button_set_active( (GtkToggleButton *)checkbutton_way, true );		
	else gtk_toggle_button_set_active( (GtkToggleButton *)checkbutton_way, false );			

	if( sg1 ) gtk_toggle_button_set_active( (GtkToggleButton *)checkbutton_haz, true );
	else gtk_toggle_button_set_active( (GtkToggleButton *)checkbutton_haz, false );

	// waypoint pins
	PiscesFILE_Handle = fopen( CurrentWaypointsFileName, "r");			
	t = 0;
	while(fgets( line, sizeof(line), PiscesFILE_Handle ) != NULL ) {						
		sscanf( line, "%d,%d,%d,%lf,%lf", &WayPoints[t].ID_Number, &WayPoints[t].x, &WayPoints[t].y, &WayPoints[t].lat, &WayPoints[t].lon );			
		t++;
	}		
	fclose(PiscesFILE_Handle);	
	
	for(t = 0; t < MAX_WAYPOINTS; t++) {
		
		sprintf(line, "%d", WayPoints[t].ID_Number);
		gtk_entry_set_text( (GtkEntry *)entrywpid[t], line );	
		
		sprintf(line, "%d", WayPoints[t].x);
		gtk_entry_set_text( (GtkEntry *)entrywpx[t], line );	
		
		sprintf(line, "%d", WayPoints[t].y);
		gtk_entry_set_text( (GtkEntry *)entrywpy[t], line );	
	}
	
	// hazard pins
	PiscesFILE_Handle = fopen( CurrentHazpointsFileName, "r");			
	t = 0;
	while(fgets( line, sizeof(line), PiscesFILE_Handle ) != NULL ) {						
		sscanf( line, "%d,%d,%d,%lf,%lf", &HazPoints[t].ID_Number, &HazPoints[t].x, &HazPoints[t].y, &HazPoints[t].lat, &HazPoints[t].lon );			
		t++;
	}		
	fclose(PiscesFILE_Handle);	
	
	for(t = 0; t < MAX_HAZPOINTS; t++) {
		
		sprintf(line, "%d", HazPoints[t].ID_Number);
		gtk_entry_set_text( (GtkEntry *)entryhpid[t], line );	
		
		sprintf(line, "%d", HazPoints[t].x);
		gtk_entry_set_text( (GtkEntry *)entryhpx[t], line );	
		
		sprintf(line, "%d", HazPoints[t].y);
		gtk_entry_set_text( (GtkEntry *)entryhpy[t], line );	
	}
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean UpdateMapSetupLabels( gpointer data ) {
	
	char s[128];
		
	sprintf( s, "%d", MapWidth ); 		
	gtk_entry_set_text( (GtkEntry *)entry_mapwidth, s);	
	
	sprintf( s, "%d", MapHeight ); 		
	gtk_entry_set_text( (GtkEntry *)entry_mapheight, s );	
	
	sprintf( s, "%lf", LongitudeLeft ); 		
	gtk_entry_set_text( (GtkEntry *)entry_lonleft, s );	
	
	sprintf( s, "%lf", LongitudeRight ); 		
	gtk_entry_set_text( (GtkEntry *)entry_lonright, s );	
	
	sprintf( s, "%lf", LatitudeBottom ); 		
	gtk_entry_set_text( (GtkEntry *)entry_latbottom, s );	
	
	return G_SOURCE_REMOVE; 
}

  
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void SaveWaypoints( void ) {
	
	int t;
	
	PiscesFILE_Handle = fopen( CurrentWaypointsFileName, "w");				
	for( t = 0; t < MAX_WAYPOINTS; t++ ) {		
		fprintf( PiscesFILE_Handle, "%d,%d,%d,%lf,%lf\n", WayPoints[t].ID_Number, WayPoints[t].x, WayPoints[t].y, WayPoints[t].lat, WayPoints[t].lon );					
	}		
	fclose(PiscesFILE_Handle);	
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void SetWaypoint( void ) {
	
	int g;
		
	digitalWrite( 26, HIGH );
	
	for( g = 0; g < MAX_WAYPOINTS; g++ ) {
												
		if( WayPoints[g].x == 0 && WayPoints[g].y == 0 ) {
							
			convertGeoToPixel( latitude, longitude );
			
			WayPoints[g].x = x; 
			WayPoints[g].y = y; 

			WayPoints[g].lat = latitude; 
			WayPoints[g].lon = longitude; 
			
			break;
		}
	}												
	SaveWaypoints();
	
	LoadWaypointsIntoEntryBoxes( );
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void LoadWaypointsIntoEntryBoxes( void ) {
	
	char str[10];
	int g;
	
	for( g = 0; g < MAX_WAYPOINTS; g++ ) {
		
		sprintf(str, "%d", WayPoints[g].x );
		gtk_entry_set_text( (GtkEntry *)entrywpx[g], &str[0] );	
		
		sprintf(str, "%d", WayPoints[g].y );
		gtk_entry_set_text( (GtkEntry *)entrywpy[g], &str[0] );	
	
	}
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void DeleteWaypoint( GtkButton * button, gpointer data ) {

	char s[20];
	int ix;
	
	strcpy( s, gtk_buildable_get_name((GtkBuildable *)button) );
	  
	ix = atoi( &s[12] ); 	
	 
	WayPoints[ix].x = 0;
	WayPoints[ix].y = 0;
	
	WayPoints[ix].lat = 0;
	WayPoints[ix].lon = 0;
	
	SaveWaypoints();
	LoadWaypointsIntoEntryBoxes();
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void SaveHazpoints( void ) {
	
	int t;
		
	PiscesFILE_Handle = fopen( CurrentHazpointsFileName, "w");				
	for( t = 0; t < MAX_HAZPOINTS; t++ ) {		
		fprintf( PiscesFILE_Handle, "%d,%d,%d,%lf,%lf\n", HazPoints[t].ID_Number, HazPoints[t].x, HazPoints[t].y, HazPoints[t].lat, HazPoints[t].lon );					
	}		
	fclose(PiscesFILE_Handle);	
		
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void SetHazpoint( void ) {
	
	int g;
	
	digitalWrite( 26, HIGH );
	
	for( g = 0; g < MAX_HAZPOINTS; g++ ) {
												
		if( HazPoints[g].x == 0 && HazPoints[g].y == 0 ) {
							
			convertGeoToPixel( latitude, longitude );

			HazPoints[g].x = x;
			HazPoints[g].y = y;
			
			HazPoints[g].lat = latitude;
			HazPoints[g].lon = longitude;

			break;
		}
	}												
	SaveHazpoints();
	
	LoadHazpointsIntoEntryBoxes( );
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void LoadHazpointsIntoEntryBoxes( void ) {
	
	char str[10];
	int g;
	
	for( g = 0; g < MAX_HAZPOINTS; g++ ) {
		
		sprintf(str, "%d", HazPoints[g].x );
		gtk_entry_set_text( (GtkEntry *)entryhpx[g], &str[0] );	
		
		sprintf(str, "%d", HazPoints[g].y );
		gtk_entry_set_text( (GtkEntry *)entryhpy[g], &str[0] );	
	
	}
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void DeleteHazpoint( GtkButton * button, gpointer data ) {

	char s[20];
	int ix;
	
	strcpy( s, gtk_buildable_get_name((GtkBuildable *)button) );
	  
	ix = atoi( &s[12] ); 	
	 
	HazPoints[ix].x = 0;
	HazPoints[ix].y = 0;
	
	HazPoints[ix].lat = 0;
	HazPoints[ix].lon = 0;
	
	SaveHazpoints();
	LoadHazpointsIntoEntryBoxes();
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define INDICATES_EMPTY		0
#define INDICATES_FULL		1

#define ON	LOW
#define OFF HIGH

void LiveWellAllStop( void ) {		
	digitalWrite( DRAIN_PUMP, OFF );		
	digitalWrite( FILL_SPRAY_PUMP, OFF );	
	gtk_widget_hide( spinnerPumpStatus );	
}

void LiveWellRaiseLevel( void ) {		
	digitalWrite( DRAIN_PUMP, OFF );		
	digitalWrite( FILL_SPRAY_PUMP, ON);		
	gtk_widget_show( spinnerPumpStatus );
}

void LiveWellLowerLevel( void ) {		
	digitalWrite( DRAIN_PUMP, ON);			
	digitalWrite( FILL_SPRAY_PUMP, ON);		
	gtk_widget_show( spinnerPumpStatus );
}

void LiveWellAerate( void ) {		
	digitalWrite( DRAIN_PUMP, ON);			
	digitalWrite( FILL_SPRAY_PUMP, ON );	
	gtk_widget_show( spinnerPumpStatus );	
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean TogglePumpEnable( gpointer data ) {
	
	digitalWrite( 26, HIGH );
	
	AerateTogFlag ^= 1;
	
	if( AerateTogFlag ) {
		
		gtk_tool_button_set_label( (GtkToolButton *)tbpumptog, "LW OFF");
		
		Interval = gtk_combo_box_get_active( (GtkComboBox *)comboboxPumpInterval );
		Duration = gtk_combo_box_get_active( (GtkComboBox *)comboboxPumpDuration );
		
		// Wait until FILLED
		if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
			usleep(5000);
			if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
				usleep(5000);
				if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
					LiveWellRaiseLevel();	
					LiveWellState = RAISE_LEVEL;
				}
			}
		} else {
			LiveWellAerate();			
			tLower = tAerate = time(NULL);				
			AerateStatus = 2;				
			LiveWellState = AERATE;
		}
			
	} else {		
		gtk_tool_button_set_label( (GtkToolButton *)tbpumptog, "LW ON");
		LiveWellState = ALL_STOP;
	}	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );	
	
	return G_SOURCE_REMOVE; 	
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void ManagePump( void ) {
	
	time_t tNow;

	if( Draining ) return;
	
	switch( LiveWellState ) {
	
	default:
	case ALL_STOP:		
		LiveWellAllStop();
		break;	
		
	case RAISE_LEVEL:
		
		// WAIT FOR filled up 
		if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
			usleep(5000);
			if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
				usleep(5000);
				if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
					break;								
				}
			}
		}
		
		tAerate = time(NULL);			
		AerateStatus = 1;	
		LiveWellAllStop();			
		LiveWellState = AERATE;		
		break;	

	case RAISE_LEVEL_IN_DURATION:
		
		// WAIT FOR filled up 
		if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
			usleep(5000);
			if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
				usleep(5000);
				if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
					break;								
				}
			}
		}				
		LiveWellAerate();
		LiveWellState = AERATE;		
		break;	
	
	case RAISE_LEVEL_B4_ALL_STOP:
		
		// WAIT FOR filled up 
		if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
			usleep(5000);
			if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
				usleep(5000);
				if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
					break;								
				}
			}
		}
		AerateStatus = 1;
		tAerate = time(NULL);
		LiveWellState = AERATE;		
		LiveWellAllStop();
		break;
			
	case LOWER_LEVEL:
		tNow = time(NULL);
		if( difftime(tNow, tLower) >= 4 ) {		
			LiveWellRaiseLevel();
			LiveWellState = RAISE_LEVEL_IN_DURATION;		
		}		
		break;
	
	case AERATE:
					
		tNow = time(NULL); 
		
		if( AerateStatus == 1 ) {			
			if( difftime(tNow, tAerate) >= IntervalSeconds[Interval] ) {
				AerateStatus = 2;
				tAerate = time(NULL);
				LiveWellAerate();				
			}
		} else {			
			if( difftime(tNow, tAerate) >= DurationSeconds[Duration] ) {			
				LiveWellRaiseLevel();
				LiveWellState = RAISE_LEVEL_B4_ALL_STOP;		
			} else {
				
				// Check for level FALLING, Pump out exceeds pump in 
				//  because fill tube is 1/2 inch ID and pump out tube
				//  is 3/4" ID.
		
				if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
					usleep(5000);
					if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {
						usleep(5000);
						if( digitalRead( FLOAT_SWITCH ) == INDICATES_EMPTY ) {						
							tLower = time(NULL);	
							LiveWellState = LOWER_LEVEL; 
							LiveWellLowerLevel();		
						}
					}
				}
			}
		}			
		break;
	}
}




//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean DrainToggle( gpointer data ) {
	
	digitalWrite( 26, HIGH );
	
	DrainStatus ^= 1;
	
	if( DrainStatus ) {						
		gtk_tool_button_set_label( (GtkToolButton *)tbdraintog, "LWD ON");
		Draining = 0;						
		LiveWellAllStop();
	} else {				
		gtk_tool_button_set_label( (GtkToolButton *)tbdraintog, "LWD OFF");
		Draining = 1;	
		digitalWrite( DRAIN_PUMP, ON);		
		digitalWrite( FILL_SPRAY_PUMP, OFF);			
	}
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void MakePhasePlt( double degdata, double disdata ) {
	
	double polardegs[] = { 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, };	
	char * compassdegs[] = { "E", "60", "30", "N", "330", "300", "W", "240", "210", "S", "150", "120" };
	int t;
	
	FILE *f;
	
	// Heading to way point, Prepare for graphing
	f = fopen("phasedata.dat", "w+");
	if(f == NULL) {	
		return;
	}
	if( degdata > 359 ) degdata = 359;
	if( degdata < 0 ) degdata = 0;
	
	fprintf(f,"%lf %lf\n", degdata, disdata );		
	fclose(f);
	
	
	
	f = fopen("phasedata2.dat", "w+");
	if(f == NULL) {	
		return;
	}
			
	fprintf(f,"%lf %lf\n", (double)CompassToPolar[(int)heading], 20.0 );	 
	fclose(f);
	

	f = fopen("phasedata.plt", "w+");	
	if(f == NULL) {
		return;
	}	
	
	fputs("set terminal jpeg size 250,250 font 'Arial,10'\n", f);			
	
//	fputs("set object 1 rectangle from screen 0,0 to screen 1,1 fillcolor rgb \"green\" behind\n", f);
	
	fputs("set output 'phasedata.jpg'\n", f);
	fputs("set autoscale\n", f);
	fputs("unset border\n", f);
	fputs("set polar\n", f);    
	fputs("set angles degrees\n", f);
		
	fputs("set xtics axis\n",f);
	fputs("set ytics axis\n",f);
	fputs("set xtics scale 0\n",f);
	fputs("set ytics scale 0\n",f);

	for( t = 0; t < 12; t++ ) {	
		if( disdata > 10 ) {				
			if( t == 0 || t == 3 || t == 6 || t == 9 ) 
				fprintf(f,"set label \"%s\" at %lf,%lf center font \"Arial,18\" textcolor \"blue\"\n", compassdegs[t], ( (disdata-1.5) * cos( polardegs[t] * GPS_PI/180 )), ( (disdata-1.5) * sin( polardegs[t] * GPS_PI/180 )) );	  				
			else
				fprintf(f,"set label \"%s\" at %lf,%lf center font \"Arial,11\" textcolor \"red\"\n", compassdegs[t], ( (disdata-1.5) * cos( polardegs[t] * GPS_PI/180 )), ( (disdata-1.5) * sin( polardegs[t] * GPS_PI/180 )) );	  				
		} else {
			if( t == 0 || t == 3 || t == 6 || t == 9 ) 
				fprintf(f,"set label \"%s\" at %lf,%lf center font \"Arial,18\" textcolor \"blue\"\n", compassdegs[t], ( (disdata+10) * cos( polardegs[t] * GPS_PI/180 )), ( (disdata+10) * sin( polardegs[t] * GPS_PI/180 )) );	  				
			else
				fprintf(f,"set label \"%s\" at %lf,%lf center font \"Arial,11\" textcolor \"red\"\n", compassdegs[t], ( (disdata+10) * cos( polardegs[t] * GPS_PI/180 )), ( (disdata+10) * sin( polardegs[t] * GPS_PI/180 )) );	  				
		}
	}

	fputs("set grid polar lw 1 lt -1\n",f);		// 10   3 is good
	
	fputs("plot \"phasedata.dat\" lw 9 lt rgb \"red\" notitle, \"phasedata2.dat\" lw 9 lt rgb \"black\" notitle\n", f);  

	fputs("exit 0", f);
	fclose(f);
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowWPTrack( gpointer data ) {
	
	digitalWrite( 26, HIGH );
	
	if( SelTrackWP == 999 ) { 	
		
		ShowWPSelection( NULL ) ;
		usleep(BEEP_DURATION);	
		digitalWrite( 26, LOW );
	
		gtk_widget_hide( label7 );
	
		return G_SOURCE_REMOVE; 	
	}
	gtk_window_set_resizable( (GtkWindow *)window_wptrack, FALSE);
	gtk_window_present( (GtkWindow*)window_wptrack);	
		
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}	


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowToolbar( gpointer data ) {
	
	digitalWrite( 26, HIGH );
	
	gtk_window_set_resizable( (GtkWindow *)window_wptrack, FALSE);
	gtk_window_present( (GtkWindow*)windowToolbar);	
		
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}	

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowWPSelection( gpointer data ) {
		
	digitalWrite( 26, HIGH );
	
	gtk_window_set_resizable( (GtkWindow *)window_wpselection, FALSE);
						  
	gtk_window_set_title( (GtkWindow *)window_wpselection, "WayPoint Navigation");
	gtk_window_present( (GtkWindow*)window_wpselection);	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}	



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
double FindHeadingToWaypoint( double clat1, double clon1, double wplat2, double wplon2 ) {

    double head, lon1, lon2, lat1, lat2;
    double londif;
    double finalans, finalans2;
		
    lat1=(clat1*GPS_PI)/180;
    lon1=(clon1*GPS_PI)/180;
    lat2=(wplat2*GPS_PI)/180;
    lon2=(wplon2*GPS_PI)/180;
	
    londif=lon2-lon1;

	head=atan2((sin(londif)*cos(lat2)),((cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(londif)))) ;	
	
    finalans=(head*180)/GPS_PI;
    finalans2=0;
	
    if(finalans<=0) {		
        finalans2=finalans+360;   
		return finalans2;
    } else {	
		return finalans;
	}
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
long GetAvailableSpace( const char* path ) {
	
	struct statvfs stat;
  
	if( statvfs( path, &stat ) != 0 ) {
		// error happens, just quits here
		return -1;
	}

	// the available size is f_bsize * f_bavail
	return stat.f_bavail;  //stat.f_bsize * stat.f_bavail;
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void InsertLogfileEntry( void ) {

	char *p;
	time_t tt = time(NULL);
	struct tm tm = *localtime(&tt);
	
	if(PiscesLogFILE_Handle != NULL) {
		
		p = g_path_get_basename( CurrentMapDatFileName );	
		
		fprintf( PiscesLogFILE_Handle, "T,%f,%f,%.0f,%.1f,%.2f,%02d:%02d:%02d,%02d_%02d_%02d,%s,%.1f,%.0f,%.0f,%s,%.0f,%s\n",
		             latitude, longitude, Altitude * 3.28084, dsTemperature, cPressure,
					 tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_mon + 1, tm.tm_mday, tm.tm_year + 1900, (char *)p,
					 AmbTemp, Humidity, WindSpeed, WindDirectionString, CloudsPercent, WeatherCondString );		
		
		if( LogFlag == 1 ) {
			fprintf( PiscesLogFILE_Handle, "Rating: %d\n", Rating );		
			fclose(PiscesLogFILE_Handle);	
			PiscesLogFILE_Handle = NULL;
			LogToggle = 0;
			gtk_widget_hide( windowRate );
		}		
	}
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean Select_Logfile( gpointer data ) {
	
	char s[128];
	
	time_t tt = time(NULL);
	struct tm tm = *localtime(&tt);
	
	digitalWrite( 26, HIGH );
	
	if( LogToggle == 0 ) {
					
		gtk_tool_button_set_label( (GtkToolButton *)tblogtog, "Log OFF");
	
		sprintf(s, "/media/usb/%d_%02d_%02d_%02d_%02d_%02d.txt", tm.tm_mon + 1, tm.tm_mday, tm.tm_year + 1900,
			tm.tm_hour, tm.tm_min, tm.tm_sec );
	
		strcpy( CurrentLogFileName, s );
		
		PiscesLogFILE_Handle = fopen( CurrentLogFileName, "w");	
		
		fprintf( PiscesLogFILE_Handle, "type,latitude,longitude,altitude_ft,temperature_F,pressure_inHg,time,date,map,airTemp_F,Humidity_per,WindSpeed_mph,WindDirection_deg,CloudsPercent,WeatherCondition\n" );		
		
		LogToggle = 1;
		LogFlag = 0;
		
	} else {		
	
		gtk_tool_button_set_label( (GtkToolButton *)tblogtog, "Log ON");
		
		gtk_toggle_button_set_active( (GtkToggleButton *)cbrate1, FALSE);
		gtk_toggle_button_set_active( (GtkToggleButton *)cbrate2, FALSE);
		gtk_toggle_button_set_active( (GtkToggleButton *)cbrate3, FALSE);
		gtk_toggle_button_set_active( (GtkToggleButton *)cbrate4, FALSE);
		gtk_toggle_button_set_active( (GtkToggleButton *)cbrate5, FALSE);
		
		gtk_window_set_resizable( (GtkWindow *)windowRate, FALSE);
		gtk_window_set_keep_above( (GtkWindow *)windowRate, TRUE);
		gtk_widget_show( windowRate );
			
	}	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
    return G_SOURCE_REMOVE; 
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
unsigned int Microseconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec*1000000 + ts.tv_nsec/1000;
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void ParseOWM_XML( char *fileName ) {

	FILE *fp;
	int i, bufferIndex;
    int j;
	char c, buffer[256];
    float * globals[6];
    const char * tokens[6];

    globals[0] = &AmbTemp;
    globals[1] = &Humidity;
    globals[2] = &WindSpeed;
    globals[3] = &WindDirection;
    globals[4] = &CloudsPercent;
	globals[5] = &WeatherCondition;
	
    const char *tempVal = "<temperature value";
    const char *humidityVal = "<humidity value";
    const char *speedVal = "<speed value";
    const char *directionVal = "<direction value";
    const char *cloudsVal = "<clouds value";
    const char *weatherCond = "<weather number";

    tokens[0] = tempVal;
    tokens[1] = humidityVal;
    tokens[2] = speedVal;
    tokens[3] = directionVal;
    tokens[4] = cloudsVal;
    tokens[5] = weatherCond;

    int found = false;
    int readingToken = false;
    
    fp = fopen( fileName, "r" );

    for( j = 0; j < 6; j++ ) {

        bufferIndex = 0;
        for( i = 0; i < 256; i++ ) buffer[i] = '\0';
        found = false;

        // Look for token
        while( ( c = fgetc(fp) ) != EOF && !found ) {
            if( c == '<' ) {
                if( !readingToken )
                    readingToken = true;
            } else if( c == '>' ) {
                if( readingToken ) {
                    readingToken = false;
                    bufferIndex = 0;
                    for( i = 0; i < 256; i++ ) buffer[i] = '\0';               
                }
            } else if( c == '=' ) {
                if( readingToken ) {
                    readingToken = false;
                    bufferIndex = 0;
                    if( strcmp( buffer, tokens[j] ) == 0 )
                        found = true;
                    else                   
                        for( i = 0; i < 256; i++ ) buffer[i] = '\0';                                   
                }
            }
            if( readingToken ) buffer[bufferIndex++] = c;
        }

        bufferIndex = 0;
        for( i = 0; i < 256; i++ ) buffer[i] = '\0';
    
        while( ( c = fgetc(fp) ) != '"' )
            buffer[bufferIndex++] = c;
                
        *(globals[j]) = atof(buffer);        
    }
	
	DegreeToCardinal( WindDirection, WindDirectionString );    
	
#if 0
    printf("Ambient Temperature = %0.2f\n", AmbTemp );
    printf("Humidity = %0.2f\n", Humidity );
    printf("Wind Speed = %0.2f\n", WindSpeed );
    printf("Wind Direction = %0.2f\n", WindDirection );
	printf("Wind Direction = %s\n", WindDirectionString );
    printf("Cloud Percentage = %0.2f\n", CloudsPercent ); 
    printf("Weather Cond = %s\n\n", WeatherCondString );
#endif    

	fclose(fp);
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//

void FindWeatherString( int wn, char *p ) {

	switch( wn ) {
		
		case 500: strcpy( p, "light rain" ); 						break;	
		case 501: strcpy( p, "moderate rain" ); 					break;
		case 502: strcpy( p, "heavy intensity rain" ); 				break;
		case 503: strcpy( p, "very heavy rain" ); 					break;
		case 504: strcpy( p, "extreme rain" ); 						break;
		case 511: strcpy( p, "freezing rain" ); 					break;
		case 520: strcpy( p, "light intensity shower rain" ); 		break;
		case 521: strcpy( p, "shower rain" ); 						break;
		case 522: strcpy( p, "heavy intensity shower rain" ); 		break;
		case 531: strcpy( p, "ragged shower rain" ); 				break;
						
		case 800: strcpy( p, "clear sky" ); 						break;	
		case 801: strcpy( p, "few clouds" ); 						break;	
		case 802: strcpy( p, "scattered clouds" ); 					break;	
		case 803: strcpy( p, "broken clouds" ); 					break;	
		case 804: strcpy( p, "overcast clouds" ); 					break;	
		
		case 300: strcpy( p, "light intensity drizzle" ); 			break;	
		case 301: strcpy( p, "drizzle" ); 							break;	
		case 302: strcpy( p, "heavy intensity drizzle" ); 			break;	
		case 310: strcpy( p, "light intensity drizzle rain" ); 		break;	
		case 311: strcpy( p, "drizzle rain" ); 						break;	
		case 312: strcpy( p, "heavy intensity drizzle rain" ); 		break;	
		case 313: strcpy( p, "shower rain and drizzle" ); 			break;	
		case 314: strcpy( p, "heavy shower rain and drizzle" ); 	break;	
		case 321: strcpy( p, "shower drizzle" ); 					break;	
		
		case 200: strcpy( p, "thunderstorm with light rain" ); 		break;	
		case 201: strcpy( p, "thunderstorm with rain" ); 			break;	
		case 202: strcpy( p, "thunderstorm with heavy rain" ); 		break;	
		case 210: strcpy( p, "light thunderstorm" ); 				break;	
		case 211: strcpy( p, "thunderstorm" ); 						break;	
		case 212: strcpy( p, "heavy thunderstorm" ); 				break;	
		case 221: strcpy( p, "ragged thunderstorm" );			 	break;	
		case 230: strcpy( p, "thunderstorm with light drizzle" ); 	break;	
		case 231: strcpy( p, "thunderstorm with drizzle" ); 		break;	
		case 232: strcpy( p, "thunderstorm with heavy drizzle" ); 	break;	
		
		case 600: strcpy( p, "light snow" ); 						break;	
		case 601: strcpy( p, "snow" ); 								break;	
		case 602: strcpy( p, "heavy snow" ); 						break;	
		case 611: strcpy( p, "sleet" ); 							break;	
		case 612: strcpy( p, "shower sleet" ); 						break;	
		case 615: strcpy( p, "light rain and snow" ); 				break;	
		case 616: strcpy( p, "rain and snow" ); 					break;	
		case 620: strcpy( p, "light shower snow" ); 				break;	
		case 621: strcpy( p, "shower snow" ); 						break;	
		case 622: strcpy( p, "heavy shower snow" ); 				break;	
		
		case 701: strcpy( p, "mist" ); 								break;	
		case 711: strcpy( p, "smoke" ); 							break;	
		case 721: strcpy( p, "haze" ); 								break;	
		case 731: strcpy( p, "sand, dust whirls" ); 				break;	
		case 741: strcpy( p, "fog" ); 								break;	
		case 751: strcpy( p, "sand" ); 								break;	
		case 761: strcpy( p, "dust" ); 								break;	
		case 762: strcpy( p, "volcanic ash" ); 						break;	
		case 771: strcpy( p, "squalls" ); 							break;	
		case 781: strcpy( p, "tornado" ); 							break;	
			
		default:  strcpy( p, "???" ); 								break;	
	}
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void DegreeToCardinal( float deg, char *p ) {

	if(     348.75 <=  deg || deg < 11.25  ) strcpy( p, "N" );
	else if( 11.25 <=  deg && deg < 33.75  ) strcpy( p, "NNE" );
	else if( 33.75 <=  deg && deg < 56.25  ) strcpy( p, "NE" );
	else if( 56.25 <=  deg && deg < 78.75  ) strcpy( p, "ENE" );
	else if( 78.75 <=  deg && deg < 101.25 ) strcpy( p, "E" );
	else if( 101.25 <= deg && deg < 123.75 ) strcpy( p, "ESE" );
	else if( 123.75 <= deg && deg < 146.25 ) strcpy( p, "SE" );
	else if( 146.25 <= deg && deg < 168.75 ) strcpy( p, "SSE" );
	else if( 168.75 <= deg && deg < 191.25 ) strcpy( p, "S" );
	else if( 191.25 <= deg && deg < 213.75 ) strcpy( p, "SSW" );
	else if( 213.75 <= deg && deg < 236.25 ) strcpy( p, "SW" );
	else if( 236.25 <= deg && deg < 258.75 ) strcpy( p, "WSW" );
	else if( 258.75 <= deg && deg < 281.25 ) strcpy( p, "W" );
	else if( 281.25 <= deg && deg < 303.75 ) strcpy( p, "WNW" );
	else if( 303.75 <= deg && deg < 326.25 ) strcpy( p, "NW" );
	else if( 326.25 <= deg && deg < 348.75 ) strcpy( p, "NNW" );
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean ShowWeather( gpointer data ) {
	
	digitalWrite( 26, HIGH );
	
	gtk_window_set_resizable( (GtkWindow *)window_weather, FALSE);
						  
	gtk_window_present( (GtkWindow*)window_weather);	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 	
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     // 
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
gboolean Submit_Rating( gpointer data ) {
	
	digitalWrite( 26, HIGH );
	
	if( gtk_toggle_button_get_active( (GtkToggleButton *)cbrate1 ) ) 	 Rating = 1;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbrate2 ) )  Rating = 2;		
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbrate3 ) )  Rating = 3;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbrate4 ) )  Rating = 4;
	else if( gtk_toggle_button_get_active( (GtkToggleButton *)cbrate5 ) )  Rating = 5;	
	else Rating = 0;
	
	LogFlag = 1;	
	
	usleep(BEEP_DURATION);	
	digitalWrite( 26, LOW );
	
	return G_SOURCE_REMOVE; 
}




