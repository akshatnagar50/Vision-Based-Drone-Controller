/*Do not remove the include below*/

#include "PlutoPilot.h"
#include "Estimate.h" /*gives access to drone rates, angles, velocities and positions*/
#include "Utils.h" /*gives access to LED, Graphs and Print*/
#include "User.h"


/*The setup function is called once at Pluto's hardware startup*/
/*fstream fout;*/
void plutoInit()
{
/*Add your hardware initialization code here*/
}
/*The function is called once before plutoPilot when you activate Developer
Mode*/
void onLoopStart()
{/*do your one time tasks here*/
	LED.flightStatus(DEACTIVATE); /*Disable default Led behaviour*/
	}
/*The loop function is called in an endless loop*/
void plutoLoop()
{  /*Add your repeated code here*/
	/*fout.open("data.csv", ios::out | ios::app);*/

	Monitor.print("",Angle.get(AG_ROLL));
	Monitor.print(",",Angle.get(AG_PITCH));
	Monitor.print(",",Angle.get(AG_YAW));
	Monitor.print(",",Rate.get(X));
	Monitor.print(",",Rate.get(Y));
	Monitor.print(",",Rate.get(Z));
	Graph.red(Rate.get(Z));
	Monitor.print(",",Position.get(X));
	Monitor.print(",",Position.get(Y));
	Monitor.print(",",Position.get(Z));
	Graph.red(Position.get(Z));
	Monitor.print(",",Velocity.get(X));
	Monitor.print(",",Velocity.get(Y));
	Monitor.print(",",Velocity.get(Z));
	Monitor.println(" ");

	/*fout<<(Angle.get(AG_ROLL))<<","<<(Angle.get(AG_PITCH))<<","<<(Angle.get(AG_YAW))<<","<<(Rate.get(X))<<","<<(Rate.get(Y))<<","<<(Rate.get(Z))<<","<<(Position.get(X))<<","<<(Position.get(Y))<<","<<(Position.get(Z))<<", "<<(Velocity.get(X))<<", "<<(Velocity.get(Y))<<","<<(Velocity.get(Z))<<"\n";*/

	}
	/*The function is called once after plutoPilot when you deactivate Developer
	  Mode*/
void onLoopFinish()
{
/*do your cleanup tasks here*/
LED.flightStatus(ACTIVATE);
}
