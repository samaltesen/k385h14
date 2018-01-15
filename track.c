/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"


struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];
double lineSensorCal[8];
double Irdata[5];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);
void lineCalibration(int lineSensor[]);
double minLineSensor(double calLinesSensor[]);
float followLineCenterGB();
float followLineCenterGW();
void irCalibration(int irSensor[]);


componentservertype lmssrv,camsrv;

 symTableElement *
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement *
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.252	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902


typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		float xOld, yOld, orientationOld, turnOrientation;
		} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);




/********************************************
* Motion control
*/

typedef struct{//input
                int cmd;
		int curcmd;

		double speedcmd;
		double dist;
		double angle;
		double radius;
		int direction;
		double left_pos,right_pos;
		// parameters
		double w;
		//output
		double motorspeed_l,motorspeed_r;
		int finished;
		// internal variables
		double startpos, startpos2;
		int condition[5];
		double linechoice;
		double lineSensorOld;
		int LineSensorIndex;
		int lineType;
	       }motiontype;

enum{dir_LEFT, dir_RIGHT};
enum{mot_stop=1,mot_move,mot_turn, mot_turnR, mot_followLine, mot_followWall};
enum {con_crossingBlackLine = 1, con_FindBlackLine, con_FindWhiteLine, con_driveDist, con_laserScan, con_lasesL4, con_lasesL6, con_followWall, con_irScan, con_irFollowWallLeft, con_laserScanLeft};
enum {bl, bm, br, wm};


double acceleration = 0.2;

void update_motcon(motiontype *p);

void driveFwd(motiontype *p);
void driveTurn(motiontype *p);
void driveTurnR(motiontype *p);
void driveLine(motiontype *p);
void driveWall(motiontype *p);

int checkFlags(motiontype *p);
int followWall(double speed, int time, double dist, int condition[]);

int fwd(double dist, double speed,int time, int condition[]);
int turn(double angle, double speed,int time, int condition[], int direction);
int turnR(double angle, double radius, double speed,int time, int condition[], int direction);
int followLine(int lineType, double speed,int time, double dist,int condition[]);



typedef struct{
                int state,oldstate;
		int time;
		int newState;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

int lineSensorSize = 8;
float datalog[10000][9];
char datalog2[10000];
float laserlog[10000][10];
float LineFollowing[10000][4];

int newState = 0;
int count = 0;
FILE * missionState;

enum {ms_init,ms_fwd,ms_turn,ms_distTobox,ms_FirstPart,ms_followWall, ms_WhiteLine, ms_GoHome,ms_end};
enum {FW_INIT, FW_FINDGATE, FW_GOTOWALL, FW_MOVETOGATE, FW_FINISH, FW_TURN};
enum {PB_INIT, PB_DISTTOBOX ,PB_GOTOBOX, PB_GOTOBOX2, PB_GOTOBOX3, PB_GOTOBOX4, PB_GOTOBOX5, PB_GOTOBOX6, PB_GOTOBOX7, PB_GOTOBOX8, PB_GOTOBOX9, PB_PUSHBOX, PB_GOTONEXTCHAL,
  PB_GOTONEXTCHAL1, PB_GOTONEXTCHAL2, PB_GOTONEXTCHAL3, PB_GOTONEXTCHAL4, PB_GOTONEXTCHAL5, PB_GOTONEXTCHAL6, PB_GOTONEXTCHAL7, PB_GOTONEXTCHAL8, PB_FINISH};
enum{PW_INIT, PW_GOTOWHITELINE, PW_GOTOWHITELINE1, PW_GOTOWHITELINE2, PW_GOTOWHITELINE3, PW_GOTOWHITELINE4, PW_GOTOWHITELINE5, PW_GOTOWHITELINE6, PW_GOTOWHITELINE7, PW_FINISH};
enum{GH_INIT,GH_HOME, GH_HOME1, GH_HOME2, GH_HOME3, GH_HOME4, GH_HOME5, GH_HOME6, GH_HOME7, GH_HOME8, GH_HOME9, GH_FINISH};



int main()
{
  int FWState = FW_FINDGATE;
  int PBState = PB_INIT;
  int PWState = PW_INIT;
  int GHState = GH_INIT;
  int running,arg,time=0;
  double dist=0,angle=0, highSpeed=0.3, speed=0.2, lowSpeed=0.15;

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE);
      }

      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE);
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE);
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);

      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0;
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}




// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len;
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     //len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     len=sprintf(buf,"scanpush cmd='zoneobst'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}


  /* Read sensors and zero our position.
   */
  rhdSync();

  odo.w=0.256;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
running=1;
mission.state=ms_init;
mission.oldstate=-1;


missionState = fopen("file.csv","w+");
while (running){
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }

      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }


  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  update_odo(&odo);

/************************************
/ mission statemachine
*/
   sm_update(&mission);
   int condition[5];
   lineCalibration(linesensor->data);
   switch (mission.state) {
     case ms_init:
       //mission.state=mot_followLine;
	//printf("%s","vi er i mission.state= ms_init");
       //n=4; dist=2;angle=-90.0/180*M_PI;
       angle=90.0/180*M_PI;
       dist = 0.1;
       //mission.state= ms_distTobox;
       mission.state= ms_GoHome;
     break;

     case ms_fwd:
       
      
       condition[0] = con_driveDist;

       if (fwd(dist,0.3,mission.time, condition))  mission.state=ms_turn;
       //if (fwd(dist,0.4,mission.time))  mission.state=ms_end;
     break;

     case ms_turn:
       if (turn(angle,0.3,mission.time, condition, dir_LEFT)){

    	   mission.state=ms_end;
       }
       
     break;
     
     case ms_distTobox:
	dist = 0.3;
	condition[0] = con_laserScan;
	if (followLine(br,speed,mission.time, dist, condition)) {
	  printf("%s%f%s","distance to box: ",(-1*odo.yOld)+0.255+laserpar[4],"\n");
	  mission.newState = 1;
	  mission.state=ms_FirstPart;
	}
     break;
     
     case ms_FirstPart:
	//PB_INIT, PB_GOTOBOX, PB_PUSHBOX
       switch (PBState) {
	 case PB_INIT:
	   mission.newState = 1;
	   PBState = PB_GOTOBOX;
	 break;
	 
	 
	 
	 case PB_GOTOBOX:
	   angle = 120.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_LEFT)){
	     //printf("%s","i am in PB_GOTOBOX");
	     mission.newState = 1;
	     PBState=PB_GOTOBOX2;
	   }
	 break;
	 
	 case PB_GOTOBOX2:
	    dist = 0.3;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,speed,mission.time, condition)) {
	      //printf("%s","i am in PB_GOTOBOX2");
	      mission.newState = 1;
	      PBState=PB_GOTOBOX3;
	    }
	 break;
	 
	 case PB_GOTOBOX3:
	   dist = 2.0;
	   condition[0] = con_FindBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     //printf("%s","i am in PB_GOTOBOX3");
	     mission.newState = 1;
	     PBState=PB_GOTOBOX4;
	   }
	 break;
	 
	 case PB_GOTOBOX4:
	   angle = 25.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_RIGHT)){
	     //printf("%s","i am in PB_GOTOBOX4");
	     mission.newState = 1;
	     PBState=PB_GOTOBOX5;
	   }
	 break;
	 
	 case PB_GOTOBOX5:
	   //printf("%s","i am in PB_GOTOBOX4");
	   condition[0] = con_crossingBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     mission.newState = 1;
	     PBState=PB_GOTOBOX6;
	   }
	 break;
	 
	 case PB_GOTOBOX6:
	   angle = 60.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_RIGHT)){
	     printf("%s","i am in PB_GOTOBOX6");
	     mission.newState = 1;
	     PBState=PB_GOTOBOX7;
	   }
	 break;
	 
	 case PB_GOTOBOX7:
	   dist = 2.0;
	   condition[0] = con_FindBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     printf("%s","i am in PB_GOTOBOX7");
	     mission.newState = 1;
	     PBState=PB_GOTOBOX8;
	   }
	 break;
	 
	 case PB_GOTOBOX8:
	   //printf("%s","i am in PB_GOTOBOX4");
	   condition[0] = con_crossingBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     printf("%s","i am in PB_GOTOBOX8");
	     mission.newState = 1;
	     PBState=PB_PUSHBOX;
	   }
	 break;
	 
	 case PB_PUSHBOX:
	    dist = 0.15;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,speed,mission.time, condition)) {
	      //printf("%s","i am in PB_GOTOBOX9");
	      mission.newState = 1;
	      PBState=PB_GOTOBOX9;
	    }
	 break;
	 
	 case PB_GOTOBOX9:
	    dist = -1.0;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,lowSpeed,mission.time, condition)) {
	      //printf("%s","i am in PB_GOTOBOX9");
	      mission.newState = 1;
	      PBState=PB_GOTONEXTCHAL;
	    }
	 break;
	 case PB_GOTONEXTCHAL:
	   angle = 60.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_RIGHT)){
	     mission.newState = 1;
	     PBState=PB_GOTONEXTCHAL1;
	   }
	 break;
	 
	 case PB_GOTONEXTCHAL1:
	    dist = 0.2;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,lowSpeed,mission.time, condition)) {
	      mission.newState = 1;
	      PBState=PB_GOTONEXTCHAL2;
	    }
	 break;
	 
	 case PB_GOTONEXTCHAL2:
	   dist = 2.0;
	   condition[0] = con_FindBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     mission.newState = 1;
	     PBState=PB_GOTONEXTCHAL3;
	   }
	 break;
	 
	 case PB_GOTONEXTCHAL3:
	   //printf("%s","i am in PB_GOTOBOX4");
	   condition[0] = con_crossingBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     mission.newState = 1;
	     PBState=PB_GOTONEXTCHAL4;
	   }
	 break;
	 
	 case PB_GOTONEXTCHAL4:
	   angle = 70.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_LEFT)){
	     mission.newState = 1;
	     PBState=PB_GOTONEXTCHAL5;
	   }
	 break;
	 
	 case PB_GOTONEXTCHAL5:
	   dist = 2.0;
	   condition[0] = con_FindBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     mission.newState = 1;
	     PBState=PB_GOTONEXTCHAL6;
	   }
	 break;
	 
	 case PB_GOTONEXTCHAL6:
	   //printf("%s","i am in PB_GOTOBOX4");
	   condition[0] = con_crossingBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     mission.newState = 1;
	     PBState=PB_GOTONEXTCHAL7;
	   }
	 break;
	 
	 case PB_GOTONEXTCHAL7:
	    dist = 0.2;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,lowSpeed,mission.time, condition)) {
	      mission.newState = 1;
	      PBState=PB_GOTONEXTCHAL8;
	    }
	 break;
	 
	  case PB_GOTONEXTCHAL8:
	   //printf("%s","i am in PB_GOTOBOX4");
	   condition[0] = con_crossingBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     mission.newState = 1;
	     PBState=PB_FINISH;
	   }
	 break;
	 
	 
	 case PB_FINISH:
	   mission.state=ms_end;
	 break;
	 
	 
       }
      break;

     case ms_followWall:
	
       switch(FWState) {

	  
	  case FW_FINDGATE:
	    dist = 0.6;
	    speed = 0.3;
	    condition[0] = con_laserScanLeft;
	    if(followLine(bl, speed, mission.time, dist, condition)) 
	    {
	      mission.newState = 1;
	      FWState = FW_MOVETOGATE;
	    }
	  break;
	  
	  case FW_MOVETOGATE:
	    //printf("Mission time: %i, newState: %i\n", mission.time, newState);
	    dist = 0.6;
	    speed = 0.3;
	    condition[0] = con_driveDist;
	    if(fwd(dist, speed, mission.time, condition)) {
	      mission.newState = 1;
	      FWState = FW_TURN;
	    }
	    
	  break;
	  case FW_TURN:
	    angle = 90.0/180*M_PI;
	    speed = 0.3;
	    condition[0] = 0;
	    if (turn(angle, speed, mission.time, condition, dir_LEFT)){
	      mission.newState = 1;
	      FWState = FW_GOTOWALL;
	    }
	  break;
	  
	  case FW_GOTOWALL:
	    dist = 0.2;
	    speed = 0.3;
	    condition[0] = con_irScan;
	    if(fwd(dist, speed, mission.time, condition)) {
	      mission.newState = 1;
	      FWState = FW_FINISH;
	    }
	    
	    
	  case FW_FINISH:
	      mission.state = ms_end;
	      
	  break;
	}
       
     break;
     
      case ms_WhiteLine:
	switch (PWState) {
	 case PW_INIT:
	   mission.newState = 1;
	   PWState = PW_GOTOWHITELINE;
	 break;
	 
	 case PW_GOTOWHITELINE:
	   dist = 2.0;
	   condition[0] = con_FindWhiteLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     mission.newState = 1;
	     PWState=PW_GOTOWHITELINE1;
	   }
	 break;
	 
	 case PW_GOTOWHITELINE1:
	   angle = 50.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_LEFT)){
	     mission.newState = 1;
	     PWState=PW_GOTOWHITELINE2;
	   }
	 break;
	 
	 case PW_GOTOWHITELINE2:
	   dist = 2.0;
	   condition[0] = con_FindWhiteLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     mission.newState = 1;
	     PWState=PW_GOTOWHITELINE3;
	   }
	 break;
	 
	  case PW_GOTOWHITELINE3:
	   //printf("%s","i am in PB_GOTOBOX4");
	   condition[0] = con_crossingBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(wm,speed,mission.time, dist, condition)) {
	     printf("%s","i am in PB_GOTOBOX8");
	     mission.newState = 1;
	     PWState=PW_FINISH;
	   }
	 break;
	 
	  case PW_FINISH:
	    mission.state = ms_end;
	  break;
	}
	 
      break;
      
      case ms_GoHome:
	switch (GHState) {
	 case GH_INIT:
	   mission.newState = 1;
	   GHState = GH_HOME;
	 break;
	 
	 case GH_HOME:
	   dist = 0.2;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,lowSpeed,mission.time, condition)) {
	      mission.newState = 1;
	      GHState=GH_HOME1;
	    }
	 break;
	 
	 case GH_HOME1:
	   angle = 60.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_RIGHT)){
	     mission.newState = 1;
	     GHState=GH_HOME2;
	   }
	 break;
	 
	 case GH_HOME2:
	   dist = 2.0;
	   condition[0] = con_FindBlackLine; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (fwd(dist,speed,mission.time, condition)) {
	     mission.newState = 1;
	     GHState=GH_HOME3;
	   }
	 break;
	 
	  case GH_HOME3:
	   dist = 0.3;
	   condition[0] = con_lasesL6; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     mission.newState = 1;
	     GHState=GH_HOME4;
	   }
	 break;
	 
	 case GH_HOME4:
	   angle = 90.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_LEFT)){
	     mission.newState = 1;
	     GHState=GH_HOME5;
	   }
	 break;
	 
	 case GH_HOME5:
	   dist = 0.8;
	    condition[0] = con_driveDist; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	    if (fwd(dist,lowSpeed,mission.time, condition)) {
	      mission.newState = 1;
	      GHState=GH_HOME6;
	    }
	 break;
	 
	 case GH_HOME6:
	   angle = 270.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turnR(angle,0.26,speed,mission.time, condition, dir_RIGHT)){
	     mission.newState = 1;
	     GHState=GH_HOME7;
	   }
	 break;
	 
	 case GH_HOME7:
	   angle = 90.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turnR(angle,0.4,speed,mission.time, condition, dir_LEFT)){
	     mission.newState = 1;
	     GHState=GH_HOME8;
	   }
	 break;
	 
	 case GH_HOME8:
	   angle = 85.0/180*M_PI;
	   condition[0] = 0; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (turn(angle,speed,mission.time, condition, dir_LEFT)){
	     mission.newState = 1;
	     GHState=GH_HOME9;
	   }
	 break;
	 
	 case GH_HOME9:
	   dist = 0.3;
	   condition[0] = con_lasesL4; condition[1] = 0; condition[2] = 0; condition[3] = 0; condition[4] = 0;
	   if (followLine(bm,speed,mission.time, dist, condition)) {
	     mission.newState = 1;
	     GHState=GH_FINISH;
	   }
	 break;
	 
	  case GH_FINISH:
	    mission.state = ms_end;
	  break;
	}
	 
      break;
       
      case ms_end:
	mot.cmd=mot_stop;
	running=0;
      break;
   }
  //printf("Mission time: %i, newState: %i\n", mission.time, newState);
   //fprintf(missionState, "%i%s%f%s%f%s" ,count,";",mot.motorspeed_l,";",mot.motorspeed_r,"\n");
   datalog[count][0] = count;
   datalog[count][1] = odo.xOld;
   datalog[count][2] = odo.yOld;
   datalog[count][3] = odo.orientationOld;
   datalog2[count] = mission.state;
   int i;
   for(i = 0; i < 10; i++) {
     laserlog[count][i] = laserpar[i];
   }
   //laserlog[count] = laserpar;
   //fprintf(missionState, "%i%s%f%s%f%s%f%i%s" ,count,";",odo.xOld,";",odo.yOld,";",odo.orientationOld,mission.state,"\n");
   count = count +1;
/*  end of mission  */

  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0){
    //    printf(" laser %f \n",laserpar[3]);
  }


  time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;

}/* end of main control loop */

int i;
for(i = 0; i < count; i++) {
  //print state
  fprintf(missionState, "%i",datalog2[i]);
  //print count, x and y
  //fprintf(missionState, "%s%i%s%f%s%f",";",i,";",datalog[i][1],";",datalog[i][2]);
  //print orientation
  //fprintf(missionState, "%s%f",";",datalog[i][3]);
  //print right_pos and left_pos
  //fprintf(missionState, "%s%f%s%f",";",datalog[i][4],";",datalog[i][5]);
  //print delta U right and delta U left
  //fprintf(missionState, "%s%f%s%f",";",datalog[i][6],";",datalog[i][7]);
  //print direction
  //fprintf(missionState, "%s%f",";",datalog[i][8]);
  int j;
  fprintf(missionState, "%s%f",";",laserlog[i][0]);
  for(j = 1; j < 10; j++) {
    fprintf(missionState, "%s%f",";",laserlog[i][j]);
  }
  //
  fprintf(missionState,"%s%f%s%f%s%f",";",LineFollowing[i][0],";",LineFollowing[i][1],";",LineFollowing[i][2]);
  fprintf(missionState, "%s","\n");


  //fprintf(missionState, "%i%s%f%s%f%s%f%s%i%s%f%s%f%s%f%s%f%s%f%s" ,i,";",datalog[i][1],";",datalog[i][2],";",datalog[i][3],";",
	  //datalog2[i],";",datalog[i][4],";",datalog[i][5],";",datalog[i][6],";",datalog[i][7],";",datalog[i][8],"\n");
}
fclose(missionState);
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}


/*
 * Routines to convert encoder values to positions. int con[5];
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->xOld = -2;
  p->yOld = 0;
  p->orientationOld = 0;
  p->turnOrientation = 0;
}

void update_odo(odotype *p)
{
  int delta;

  float dUr, dUl, displacement, orientation;
  dUr = p->right_pos - datalog[count-1][4];
  dUl = p->left_pos - datalog[count-1][5];
  displacement = (dUr+dUl)/2;
  orientation = (dUr-dUl)/WHEEL_SEPARATION;
  p->xOld = p->xOld+displacement*cos(p->orientationOld);
  p->yOld = p->yOld+displacement*sin(p->orientationOld);
  p->orientationOld = p->orientationOld+orientation;
  p->turnOrientation = p->turnOrientation + orientation;
  datalog[count][4] = p->right_pos;
  datalog[count][5] = p->left_pos;
  datalog[count][6] = dUr;
  datalog[count][7] = dUl;
  datalog[count][8] = displacement;



  delta = p->right_enc - p->right_enc_old;
  if (p->right_enc > 0x8000) p->right_enc -= 0x10000;
  else if (p->right_enc < -0x8000) p->right_enc += 0x10000;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;



  delta = p->left_enc - p->left_enc_old;
  if (p->left_enc > 0x8000) p->left_enc -= 0x10000;
  else if (p->left_enc < -0x8000) p->left_enc += 0x10000;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;




}


void update_motcon(motiontype *p){

if (p->cmd !=0){

     p->finished=0;
     switch (p->cmd){
     case mot_stop:
       p->curcmd=mot_stop;
       break;
       case mot_move:
       p->startpos=(p->left_pos+p->right_pos)/2;
       p->curcmd=mot_move;
       break;

       case mot_turn:
	 //printf("we are here");
         if (p->angle > 0) {
	    p->startpos=p->right_pos;
	    p->startpos2 = p->left_pos;
	 }
	 else {
	    p->startpos=p->left_pos;
	    p->startpos2 = p->right_pos;
	 }
         p->curcmd=mot_turn;
       break;
       case mot_turnR:
	 //printf("we are here");
         if (p->angle > 0) {
	    p->startpos=p->right_pos;
	    p->startpos2 = p->left_pos;
	 }
	 else {
	    p->startpos=p->left_pos;
	    p->startpos2 = p->right_pos;
	 }
         p->curcmd=mot_turnR;
       break;

      case mot_followLine:
	 p->startpos=(p->left_pos+p->right_pos)/2;
         p->curcmd=mot_followLine;
      break;
      
      case mot_followWall:
	p->startpos=(p->left_pos+p->right_pos)/2;
	p->curcmd=mot_followWall;
      break;
     }

     p->cmd=0;
   }


    if(checkFlags(p)) 
    {
      
      p->motorspeed_r=0;
      p->motorspeed_l=0;
      p->finished=1;
      
    } 
    else 
    {
	switch (p->curcmd){
	    case mot_stop:
	      p->motorspeed_l=0;
	      p->motorspeed_r=0;
	    break;
	    
	    case mot_move:
	      
		  driveFwd(p);
	
	    break;

	    case mot_turn:

		driveTurn(p);
	    

	    break;
	    case mot_turnR:
		driveTurnR(p);
	    break;

	    case mot_followLine:

		driveLine(p);
	    
	    break;
	    
	    case mot_followWall:
	      
		driveWall(p);

	    break;
	}
    }
}

int checkFlags(motiontype *p) {
    
    int i;
    int j;
    for(j = 0; j < 5; j++) {
      
      switch (p->condition[j]){
	case 0:
	  
	break;
	case con_crossingBlackLine:
	  for(i = 0; i < 8; i++) {
	    if((1-lineSensorCal[i]) < 0.8) {
	      return 0;
	    }
	  }
	  return 1;
	break;
	  
	case con_FindBlackLine:
	  
	  for(i = 0; i < 8; i++) {
	    //printf("lineSensorCal is: %f",(1-lineSensorCal[i]));
	    if(((1-lineSensorCal[i])) > 0.8) {
	      return 1;
	    }
	  }
	  return 0;
	break;
	
	case con_FindWhiteLine:
	  //printf("lineSensorCal is: %f %f %f %f %f %f %f %f",(lineSensorCal[0]),(lineSensorCal[1]),(lineSensorCal[2]),(lineSensorCal[3]),(lineSensorCal[4]),(lineSensorCal[5]),
	  //  (lineSensorCal[6]),(lineSensorCal[7]));
	  for(i = 0; i < 8; i++) {
	    if(((lineSensorCal[i])) > 0.8) {
	      return 1;
	    }
	  }
	  return 0;
	break;
	
	case con_driveDist:
	  if(p->dist < 0) {
	    //printf("newPos %f and oldPos &f \n",((p->right_pos+p->left_pos)/2- p->startpos),(p->dist- 0.001));
	    if((p->right_pos+p->left_pos)/2- p->startpos < p->dist- 0.001) {
	    return 1;
	    }
	    return 0;
	  }else {
	    if((p->right_pos+p->left_pos)/2- p->startpos > p->dist- 0.001) {
	    return 1;
	    }
	    return 0;
	  }
	  
	break;
	
	case con_laserScan:
	  for(i = 0; i < 10; i++) {
	    //printf("i: %i laser: %f:, dist: %f\n", i, laserpar[i], p->dist);
	    if(laserpar[i] < p->dist && laserpar[i] > 0) {
	      return 1;
	    }
	  }
	  return 0;
	break;
	 
	case con_lasesL4:
	  if(laserpar[4] < p->dist && laserpar[4] > 0) {
	      return 1;
	  }
	  return 0;
	break;
	 
	case con_lasesL6:
	  if(laserpar[6] < p->dist && laserpar[6] > 0) {
	      return 1;
	  }
	  return 0;
	break;
	 
	case con_laserScanLeft:
	  //printf("laser: %f:, dist: %f\n", laserpar[i], p->dist);
	  if(laserpar[0] < p->dist && laserpar[0] > 0) {
	      return 1;
	  }
	  return 0;
	
	break;
	  
	case con_irScan:
	  irCalibration(irsensor->data);
	  for(i = 0; i < 3; i++) {
	    if(Irdata[i+1] < p->dist && Irdata[i+1] > 0) {
	      return 1;
	    }
	  }
	  return 0;
	break;
	case con_irFollowWallLeft:
	  irCalibration(irsensor->data);
	  if(Irdata[0] > 0.5 && Irdata[0] > 0) {
	    return 1;
	  }
	  return 0;
	  
	break;
      }
    }
    return 0;
}


int fwd(double dist, double speed,int time, int condition[]){
  //printf("time: %i, dist: %f, condition: %i", time, dist, condition[0]);
  if (time==0){
     mot.speedcmd=speed;
     mot.cmd=mot_move;
     mot.dist=dist;
     memcpy(mot.condition, condition, 5*sizeof(int));
     return 0;
   }
   else
     return mot.finished;
}

int followLine(int lineType, double speed,int time, double dist,int condition[]){
 // printf("time: %i\n", mission.time);
  if (time==0){
	//printf("time=0");
     mot.speedcmd=speed;
     mot.cmd=mot_followLine;
     memcpy(mot.condition, condition, 5*sizeof(int));
     mot.dist=dist;
     mot.lineType = lineType;
     switch (lineType){
      case bl:
	mot.linechoice = -0.8;
      break;
      case bm:
	mot.linechoice = 0.0;
      break;
      case br:
	mot.linechoice = 0.8;
      break;
      case wm:
	mot.linechoice = 0.0;
    }
     return 0;
   }
   else
     return mot.finished;
}

/* calculate the values for linesensor to be besween 0 and 1
*/


float followLineCenterGB() {
  int i; 
  double topSum = 0, bottomSum = 0.0001;
  
  for(i = 1; i < 9; i++) {
    topSum = topSum + i * (1-lineSensorCal[i-1]);
    bottomSum = bottomSum + (1-lineSensorCal[i-1]);
  }
  return topSum/bottomSum;

}

float followLineCenterGW() {
  int i;
  double topSum = 0.0, bottomSum = 0.0001;
  /*printf("lineSensorCal is: %f %f %f %f %f %f %f %f",(lineSensorCal[0]),(lineSensorCal[1]),(lineSensorCal[2]),(lineSensorCal[3]),(lineSensorCal[4]),(lineSensorCal[5]),
    (lineSensorCal[6]),(lineSensorCal[7]));*/
  for(i = 1; i < 9; i++) {
    topSum = topSum + i * (lineSensorCal[i-1]);
    bottomSum = bottomSum + (lineSensorCal[i-1]);
  }
  //printf("topSum %f and bottomSum %f \n",topSum, bottomSum);
  return topSum/bottomSum;

}

int turn(double angle, double speed,int time, int condition[], int direction){

  //printf("time %i, angle: %f \n", time, angle);
  if (time==0){
     odo.turnOrientation = 0;
     mot.direction = direction;
     mot.speedcmd=speed;
     mot.cmd=mot_turn;
     mot.angle=angle;
     memcpy(mot.condition, condition, 5*sizeof(int));
     //printf("Angle: %f", mot.angle);
     return 0;
   }
   else
     return mot.finished;
}

int turnR(double angle, double radius, double speed,int time, int condition[], int direction){


  if (time==0){
     odo.turnOrientation = 0;
     mot.direction = direction;
     mot.speedcmd=speed;
     mot.cmd=mot_turnR;
     mot.angle=angle;
     mot.radius = radius;
     memcpy(mot.condition, condition, 5*sizeof(int));
     return 0;
   }
   else
     return mot.finished;
}

int followWall(double speed, int time, double dist, int condition[]) 
{
  if(time == 0) 
  {
    mot.speedcmd = speed; 
    mot.cmd = mot_followWall;
    mot.dist = dist;
    memcpy(mot.condition, condition, 5*sizeof(int));
    return 0;
  }
  else 
  {
    return mot.finished;
  }
}

void sm_update(smtype *p){
  if (p->state!=p->oldstate || p->newState){
       p->time=0;
       p->newState = 0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
   
}

void driveFwd(motiontype *p) 
{	
    double speed;
    if(p->dist < 0) {
      speed = -p->speedcmd;
    }else {
      speed = p->speedcmd;
    }
    
    double maxSpeed = sqrt(2*acceleration*(p->dist - ((p->right_pos+p->left_pos)/2- p->startpos)));
    
    if((p->dist > 0 && p->motorspeed_l < p->speedcmd) || (p->dist < 0 && p->motorspeed_l < -p->speedcmd)) 
    {
	speed = p->motorspeed_l + acceleration/100;
    }
    if(speed > maxSpeed) 
    {
	speed = maxSpeed;
    }
    
    p->motorspeed_l = speed;
    p->motorspeed_r = speed;
}

void driveTurn(motiontype *p) 
{
  double speed;
  double static iVal;
  double e = fabs(p->angle) - fabs(odo.turnOrientation);
  double kp = 0.04;
  double ki = 0.01;
  
  iVal = iVal + e*ki;
  speed = kp*(e + iVal);
  
  //printf("Speed: %f, angle: %f, odoAngle: %f\n", speed, p->angle, odo.turnOrientation);
  if (p->direction == dir_LEFT){
    p->motorspeed_l = 0;
    
    if (fabs(odo.turnOrientation) < fabs(p->angle))
    {
	p->motorspeed_r = speed;
	p->motorspeed_l = speed * -1;
    }
    else 
    {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      odo.turnOrientation = 0;
    }
  }
  else if (p->direction == dir_RIGHT) 
  {
    p->motorspeed_r=0;
    if (fabs(odo.turnOrientation) < fabs(p->angle))
    {
	p->motorspeed_l = speed;
	p->motorspeed_r = speed*-1;
    }
    else 
    {
      p->motorspeed_r=0;
      p->motorspeed_l=0;
      p->finished=1;
      odo.turnOrientation = 0;
    }
  }
}

void driveTurnR(motiontype *p) 
{
    double centerDistance;
    double turnTime;
    double leftWheelRadius;
    double rightWheelRadius;
    double leftWheelDistance;
    double rightWheelDistance;
    /*
    double speed;
    double static iVal;
    double e = fabs(p->angle) - fabs(odo.turnOrientation);
    double kp = 0.04;
    double ki = 0.01;
    
    iVal = iVal + e*ki;
    speed = kp*(e + iVal);
    */
    
    centerDistance = p-> radius * p->angle;
    turnTime = centerDistance/p->speedcmd;
    
    if (fabs(odo.turnOrientation) < fabs(p->angle))
    {
	
      if(p->direction == dir_LEFT) 
      {
	  leftWheelRadius = p-> radius-(odo.w/2);
	  rightWheelRadius = p-> radius + (odo.w/2);
	  leftWheelDistance = leftWheelRadius * p->angle;
	  rightWheelDistance = rightWheelRadius * p->angle;
	  p->motorspeed_l = leftWheelDistance / turnTime;
	  p->motorspeed_r = rightWheelDistance / turnTime;
      } 
      else if(p->direction == dir_RIGHT)
      {
	  leftWheelRadius = p-> radius + (odo.w/2);
	  rightWheelRadius = p-> radius - (odo.w/2);
	  leftWheelDistance = leftWheelRadius * p->angle;
	  rightWheelDistance = rightWheelRadius * p->angle;
	  p->motorspeed_l = leftWheelDistance / turnTime;
	  p->motorspeed_r = rightWheelDistance / turnTime;
      }
    }
    else 
    {
      p->motorspeed_r=0;
      p->motorspeed_l=0;
      p->finished=1;
      odo.turnOrientation = 0;
    }
    
  
}

void driveLine(motiontype *p) 
{
    float centerOfGravity;
    if(p->lineType != wm) {
      centerOfGravity = followLineCenterGB();
    }else {
      centerOfGravity = followLineCenterGW();
    }
    
    double setPoint = 4.5 + p -> linechoice;
    
    double static iVal;
    //double lineSensor[8] = linesensor->data;
    //lineCalibration(linesensor->data);
    double e = fabs(centerOfGravity - setPoint);
    double kp = 0.03;
    double ki = 0.005;
    iVal = iVal + e*ki;
    double deltaSpeed = kp*(e + iVal);
    
    LineFollowing[count][0] = centerOfGravity;
    LineFollowing[count][1] = setPoint;
    LineFollowing[count][2] = deltaSpeed;
  
    //printf("CenterOfGravity: %f, deltaSpeed: %f, linechoice: %f, setPoint: %f\n", centerOfGravity, deltaSpeed, p->linechoice, setPoint);
    if(centerOfGravity < setPoint) 
    {
      p->motorspeed_l = p->speedcmd + deltaSpeed;
      p->motorspeed_r = p->speedcmd - deltaSpeed;
    }
    else if(centerOfGravity > setPoint) 
    {
      p->motorspeed_l = p->speedcmd - deltaSpeed;
      p->motorspeed_r = p->speedcmd + deltaSpeed;
    }
    else
    {
      p->motorspeed_l = p->speedcmd;
      p->motorspeed_r = p->speedcmd;
    }  
}


void driveWall(motiontype *p)
{
    double setPoint = 0.1;
    double static iVal;
    double e = fabs(Irdata[0] - setPoint);
    double kp = 0.06;
    double ki = 0.03;
    iVal = iVal + e*ki;
    double deltaSpeed = kp*(e + iVal);
  
    //printf("error: %f, deltaspeed: %f, irdata: %f, setPoint: %f", e, deltaSpeed, Irdata[0], setPoint);
    
    if(Irdata[0] > setPoint) 
    {
	//printf(" turning left\n");
	p->motorspeed_l = p->speedcmd - deltaSpeed;
	p->motorspeed_r = p->speedcmd + deltaSpeed;
    } 
    else if (Irdata[0] < setPoint)
    {
	//printf(" turning right\n");
	p->motorspeed_l = p->speedcmd + deltaSpeed;
	p->motorspeed_r = p->speedcmd - deltaSpeed;
    } 
    else
    {
      //printf(" going straight\n");
      p->motorspeed_l = p->speedcmd;
      p->motorspeed_r = p->speedcmd;
    }
}


void lineCalibration(int lineSensor[]){
  //data for the smr robot
  //double a[] = {52.0733, 33.5055, 109.6757, 36.5417, 38.1459,35.9391, 36.0485, 31.3501};
  //double b[] = {57.6863, 53.7157, 71.8627, 53.8333, 53.9118, 53.6667, 54.4804, 55.2941};
  //data for the simulater
  double a[] = {170, 170, 170, 170, 170, 170, 170, 170};
  double b[] = {85, 85, 85, 85, 85, 85, 85, 85};
  //data for the simulater2
  //double a[] = {255, 255, 255, 255, 255, 255, 255, 255};
  //double b[] = {86.70, 86.70, 86.70, 86.70, 86.70, 86.70, 86.70, 86.70};
  int i;
  for (i = 0; i < lineSensorSize; i++) {
    lineSensorCal[i] = (lineSensor[i]-b[i])/a[i];
  }
  /*printf("lineSensorCal: %f, %f, %f, %f, %f, %f, %f, %f, %f\n",(1-lineSensorCal[0]), (1-lineSensorCal[1]), (1-lineSensorCal[2]), (1-lineSensorCal[3]), (1-lineSensorCal[4]),
    (1-lineSensorCal[5]), (1-lineSensorCal[6]), (1-lineSensorCal[7]));
    */
}

/*find the smalles value of the line sensor
*/
double minLineSensor(double calLineSensor[]) {
  double minimum = calLineSensor[0];
  int index = 0;
  //find the mimimum in the array
  int i;
  for (i = 1 ; i < lineSensorSize ; i++ ) {
      if ( calLineSensor[i] < minimum ) {
         minimum = calLineSensor[i];
         index = i;
      }
  }
  return index;
}

void irCalibration(int irSensor[]){
  double k[5][2] = {{14.06, 72.91},{16.54, 89.73}, {16.68, 84.01}, {16.32, 71.89},{17.07, 53.32}};
  int i;
  for(i = 0; i < 5; i++) {
    Irdata[i] = k[i][0]/(irSensor[i]-k[i][1]);
  }
  //printf("%s%f%s%f%s%f%s","result:",Irdata[0],";",Irdata[1],";",Irdata[2],"\n");
}
