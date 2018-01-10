

//Add mot_followWall to motiontype enum
//Add ms_followWall to missionstate enum
//Add con_followWall to condition enum

void followWall(double speed, int time, int con) 
{
  if(time == 0) 
  {
    mot.speedcmd = speed; 
    mot.cmd = mot_followWall;
    mot.condition = con;
    return 0;
  }
  else 
  {
    return mot.finished;
  }
}

double irDistLeft;

void driveWall(double distToWall, motiontype *p)
{
  if(irDistLeft > distToWall) 
  {
      p->motorspeed_l = p->speedcmd - deltaSpeed;
      p->motorspeed_r = p->speedcmd + deltaSpeed;
  } 
  else if ( irDistLeft < distToWall)
  {
     p->motorspeed_l = p->speedcmd + deltaSpeed;
     p->motorspeed_r = p->speedcmd - deltaSpeed;
  } 
  else
  {
    p->motorspeed_l = p->speedcmd;
    p->motorspeed_r = p->speedcmd;
  }
}

void driveFwd(motiontype *p) 
{
   p->motorspeed_l=currentSpeed;
   p->motorspeed_r=currentSpeed;
}

void driveTurn(motiontype *p) 
{
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
}

enum{dir_LEFT, dir_RIGHT};

void driveTurn(motiontype *p, double radius, int direction) 
{
    double centerDistance;
    double turnTime;
    double leftWheelRadius;
    double rightWheelRadius;
    double leftWheelDistance;
    double rightWheelDistance;
    double leftSpeed;
    double rightSpeed;
    
    centerDistance = radius * p->angle;
    turnTime = centerDistance/p->speedcmd;
    
    if(direction == dir_LEFT) 
    {
	leftWheelRadius = radius-(odo.w/2);
	rightWheelRadius = radius + (odo.w/2);
	leftWheelDistance = leftWheelRadius * p->angle;
	rightWheelDistance = rightWheelRadius * p->angle;
	p->motorspeed_l = leftWheelDistance / turnTime;
	p->motorspeed_r = rightWheelDistance / turnTime;
    } 
    else 
    {
	leftWheelRadius = radius + (odo.w/2);
	rightWheelRadius = radius - (odo.w/2);
	leftWheelDistance = leftWheelRadius * p->angle;
	rightWheelDistance = rightWheelRadius * p->angle;
	p->motorspeed_r = leftWheelDistance / turnTime;
	p->motorspeed_l = rightWheelDistance / turnTime;
    }
    
  
}


void irCalibration()
{

}



if(checkFlags) 
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

	    driveTurn(*p, dir_LEFT);
	

	break;

	case mot_followLine:

	    driveTurn(*p);
	
	break;
	
	case mot_followWall;
	  
	    driveWall(0.2, *p);

	break;
    }
}
  









