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

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);
void lineCalibration(int lineSensor[]);
double minLineSensor(double calLinesSensor[]);
float followLineCenterGB();
float followLineCenterGW();


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
		double left_pos,right_pos;
		// parameters
		double w;
		//output
		double motorspeed_l,motorspeed_r;
		int finished;
		// internal variables
		double startpos, startpos2;
		int condition;
		double linechoice;
	      double lineSensorOld;
	      int LineSensorIndex;
	       }motiontype;

enum {mot_stop=1,mot_move,mot_turn, mot_followLine};
enum {con_crossingBlackLine, con_FindBlackLine, con_driveDist, con_laserScan};
enum {bl, bm, br};

void update_motcon(motiontype *p);
double followLine(motiontype *p);
int checkFlags(motiontype *p);
void FollowLinemotorControl(motiontype *p);

int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int followLine2(int lineType, double dist,int con, double speed,int time);




typedef struct{
                int state,oldstate;
		int time;
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
float CenterOfGravity[10000];
double lineSensorCal[8];
int count = 0;
FILE * missionState;

enum {ms_init,ms_fwd,ms_turn,ms_end};

int main()
{
  int running,n=0,arg,time=0;
  double dist=0,angle=0;

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

   switch (mission.state) {
     case ms_init:
       mission.state=mot_followLine;
	//printf("%s","vi er i mission.state= ms_init");
       //n=4; dist=2;angle=-90.0/180*M_PI;
       //n=4; dist=2;angle=90.0/180*M_PI;
       //mission.state= ms_fwd;
     break;

     case ms_fwd:
       if (fwd(dist,0.3,mission.time))  mission.state=ms_turn;
       //if (fwd(dist,0.4,mission.time))  mission.state=ms_end;
     break;

     case ms_turn:
       if (turn(angle,0.3,mission.time)){
         n=n-1;
    	 if (n==0)
    	   mission.state=ms_end;
       else
    	    mission.state=ms_fwd;
  	   }
       break;

     case mot_followLine:
	//printf("%s","we are in the mot_followLine Case of mission \n");
       if (followLine2(bl,0.25,con_laserScan,0.3,mission.time)) {
	 mission.state=ms_end;
	 printf("%s%f%s","distance to box: ",(-1*odo.yOld)+0.255+laserpar[4],"\n");
       }
        break;

     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
   }

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
  //print the center of gravity that is calculated
  fprintf(missionState, "%s%f",";",CenterOfGravity[i]);
  //Print the laser data
  /*int j;
  fprintf(missionState, "%s%f",";",laserlog[i][0]);
  for(j = 1; j < 10; j++) {
    fprintf(missionState, "%s%f",";",laserlog[i][j]);
  }*/
  fprintf(missionState, "%s","\n");
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
 * Routines to convert encoder values to positions.
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

int dec = 0;
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

       case mot_followLine:
	 p->startpos=(p->left_pos+p->right_pos)/2;
         p->curcmd=mot_followLine;
       break;


     }

     p->cmd=0;
   }

   double maxSpeed = sqrt(2*0.1*(p->dist - ((p->right_pos+p->left_pos)/2- p->startpos)));
   double static currentSpeed = 0;


    if(p->curcmd == mot_move) {
      printf("Straight ");
      if(p->motorspeed_l < p->speedcmd) {
	  currentSpeed = p->motorspeed_l + 0.1/100;


      }
      if(currentSpeed > maxSpeed) {

	  currentSpeed = maxSpeed;

      }
    } else if(p->curcmd == mot_turn) {
      printf("turning ");
      double static iVal;
      double e = fabs(p->angle) - fabs(odo.turnOrientation);
      double kp = 0.04;
      double ki = 0.01;
      iVal = iVal + e*ki;
      currentSpeed = kp*(e + iVal);
    }


    //printf("Speed: %f, max speed %f\n", currentSpeed, maxSpeed);

   switch (p->curcmd){
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     break;
     case mot_move:

       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist- 0.001){
          p->finished=1;
	  dec = 0;
	  p->motorspeed_l=currentSpeed;
          p->motorspeed_r=currentSpeed;
       }
       else {
	  p->motorspeed_l=currentSpeed;
          p->motorspeed_r=currentSpeed;
       }
     break;

     case mot_turn:

       if (p->angle>0){
          p->motorspeed_l=0;
	  if (fabs(odo.turnOrientation) < fabs(p->angle)){
	      p->motorspeed_r=currentSpeed;
	      p->motorspeed_l=currentSpeed*-1;
	  }
	  else {
            p->motorspeed_r=0;
	    p->motorspeed_l=0;
            p->finished=1;
	    odo.turnOrientation = 0;
	  }
	}
	else {
          p->motorspeed_r=0;
	  if (fabs(odo.turnOrientation) < fabs(p->angle)){
	      p->motorspeed_l=currentSpeed;
	      p->motorspeed_r=currentSpeed*-1;
	  }
	  else {
	    p->motorspeed_r=0;
            p->motorspeed_l=0;
            p->finished=1;
	    odo.turnOrientation = 0;
	  }
	}

     break;

     case mot_followLine:
	//printf("%s","vi er i mot_followline casen where we set motorspeed");
      if(checkFlags(p) == 1) {
	p->finished=1;
	dec = 0;
	p->motorspeed_l=currentSpeed;
        p->motorspeed_r=currentSpeed;
      }else {
	FollowLinemotorControl(p);
      }
	break;

   }
}
//con_crossingBlackLine,con_FindBlackLine,con_driveDist
int checkFlags(motiontype *p) {
  int i;
    switch (p->condition){
      case con_crossingBlackLine:
	for(i = 0; i < 8; i++) {
	  if((1-lineSensorCal[i]) > 0.1) {
	    return 0;
	  }
	}
	return 1;
      break;
	
      case con_FindBlackLine:
	for(i = 0; i < 8; i++) {
	  if((1-lineSensorCal[i]) > 0.1) {
	    return 1;
	  }
	}
	return 0;
      break;
      
      case con_driveDist:
	if((p->right_pos+p->left_pos)/2- p->startpos > p->dist- 0.001) {
	  return 1;
	}
	return 0;
      break;
      
      case con_laserScan:
	for(i = 0; i < 10; i++) {
	  if(laserpar[i] < p->dist && laserpar[i] > 0) {
	    return 1;
	  }
	}
	return 0;
	break;
    }
    return 0;
}

void FollowLinemotorControl(motiontype *p) {
  double deltaSpeed = followLine(p)*15;
  float index = followLineCenterGB();
  CenterOfGravity[count] = index;
  if(index < 4.5+p->linechoice) {
    p->motorspeed_l = p->speedcmd - deltaSpeed;
    p->motorspeed_r = p->speedcmd + deltaSpeed;
  }else if(index > 4.5+p->linechoice) {
    p->motorspeed_l = p->speedcmd + deltaSpeed;
    p->motorspeed_r = p->speedcmd - deltaSpeed;
  }else {
    p->motorspeed_l = p->speedcmd;
    p->motorspeed_r = p->speedcmd;
  }  
}


int fwd(double dist, double speed,int time){

  if (time==0){
     mot.speedcmd=speed;
     mot.cmd=mot_move;
     mot.dist=dist;
     return 0;
   }
   else
     return mot.finished;
}

int followLine2(int lineType, double dist,int con, double speed,int time){
  //printf("time=0");
  if (time==0){
	//printf("time=0");
     mot.speedcmd=speed;
     mot.cmd=mot_followLine;
     mot.condition = con;
     mot.dist=dist;
     switch (lineType){
      case bl:
	mot.linechoice = -0.15;
      break;
      case bm:
	mot.linechoice = 0.0;
      break;
      case br:
	mot.linechoice = 0.15;
      break;
    }
     mot.linechoice = lineType;
     return 0;
   }
   else
     return mot.finished;
}

/* calculate the values for linesensor to be besween 0 and 1
*/
void lineCalibration(int lineSensor[]){
  //data for the smr robot
  //double a[] = {52.0733, 33.5055, 109.6757, 36.5417, 38.1459,35.9391, 36.0485, 31.3501};
  //double b[] = {57.6863, 53.7157, 71.8627, 53.8333, 53.9118, 53.6667, 54.4804, 55.2941};
  //data for the simulater
  double a[] = {170, 170, 170, 170, 170, 170, 170, 170};
  double b[] = {85, 85, 85, 85, 85, 85, 85, 85};
  int i;
  for (i = 0; i < lineSensorSize; i++) {
    lineSensorCal[i] = (lineSensor[i]-b[i])/a[i];
  }
}
/*find the smalles value of the line sensor
*/
double minLineSensor(double calLineSensor[]) {
  double minimum = calLineSensor[0];
  int index = 0;
  //find the mimimum in the array
  int i;
  for (i = 1 ; i < lineSensorSize ; i++ ) {
      if ( (1-calLineSensor[i]) < minimum ) {
         minimum = (1-calLineSensor[i]);
         index = i;
      }
  }
  return index;
}

double followLine(motiontype *p) {
  double static iVal;
  //double lineSensor[8] = linesensor->data;
  lineCalibration(linesensor->data);
  p->LineSensorIndex = minLineSensor(lineSensorCal);
  double e = fabs(p->lineSensorOld) - fabs(lineSensorCal[p->LineSensorIndex]);
  p->lineSensorOld = lineSensorCal[p->LineSensorIndex];
  double kp = 0.16;
  double ki = 0.04;
  iVal = iVal + e*ki;
  return kp*(e + iVal);

}

float followLineCenterGB() {
  int i; 
  double topSum = 0, bottomSum = 0;
  
  lineCalibration(linesensor->data);
  for(i = 1; i < 9; i++) {
    topSum = topSum + i * (1-lineSensorCal[i-1]);
    bottomSum = bottomSum + (1-lineSensorCal[i-1]);
  }
  return topSum/bottomSum;

}

float followLineCenterGW() {
  int i, topSum = 0, bottomSum = 0;
  
  lineCalibration(linesensor->data);
  for(i = 1; i < 9; i++) {
    topSum = topSum + i * (lineSensorCal[i-1]);
    bottomSum = bottomSum + (lineSensorCal[i-1]);
  }
  return topSum/bottomSum;

}

int turn(double angle, double speed,int time){


  if (time==0){
     mot.speedcmd=speed;
     mot.cmd=mot_turn;
     mot.angle=angle;
     return 0;
   }
   else
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}
