/*
Robotics Final Project
by
Ian Camp
*/


#include <libplayerc++/playerc++.h>
#include <iostream>
#include <math.h>
#include <vector>
#include "args.h"

#define RAYS 32
using namespace std;

int sizeY=17;
int sizeX=17;
int goalx=-1;
int goaly=-1;
bool finished =false;
int validcount=1;


//--------------------Start Cost To Go Functions for path finding------------------

//GetInputs
vector<int> GetInputs(int y, int x, vector< vector <int> > map){
//down,up,left,right
vector<int> choices;
if((y-1)>0 && map[y-1][x]==1){ 
choices.push_back(0);
}
if((y+1)<sizeY && map[y+1][x]==1){ 
choices.push_back(1);
}
if((x-1)>0 && map[y][x-1]==1) {
choices.push_back(2);
}
if((x+1)<sizeX && map[y][x+1]==1){ 
choices.push_back(3);
}

return choices;
}


//NextState
vector<int> NextState(int y, int x, int u){
vector<int> state;
if(u==0){
state.push_back(y-1);
state.push_back(x);
}
if(u==1){
state.push_back(y+1);
state.push_back(x);
}
if(u==2){
state.push_back(y);
state.push_back(x-1);
}
if(u==3){
state.push_back(y);
state.push_back(x+1);
}

return state;
}




//Cost
int Cost(int r,int c,int u, vector< vector <int> > CostToGo, vector< vector <int> > map){

int cost=1000;
vector <int> choices = GetInputs(r,c,map);
bool okay=false;

for(int i=0;i<choices.size();i++){
if(choices[i]==u){okay=true;}
}

if(okay){
vector<int> state=NextState(r,c,u);

cost=CostToGo[state[0]][state[1]]+1;
}

return cost;
}


//Plan
vector<vector <int> > Plan(vector< vector <int> >  map,vector< vector <int> > CostToGo){


for (int i=0;i<sizeY;++i){
for (int j=0;j<sizeX;++j){
vector<int> costs;
if(map[i][j]==1){
vector<int> choices=GetInputs(i,j,map);
if(choices.size()>0){
for (int k=0;k<choices.size();++k){	
costs.push_back(Cost(i,j,choices[k],CostToGo,map));
}
}
}
costs.push_back(CostToGo[i][j]);
int mincost=1000;
for(int l=0;l<costs.size();++l){
if(mincost>costs[l])mincost=costs[l];
}
CostToGo[i][j]=mincost;
		}
	}
return CostToGo;
}



//---------------------------END Cost Go Functions-----------------------



int main(int argc, char **argv)
{
  parse_args(argc,argv);

//initialize map
vector<int> maprow;
vector< vector <int> > map;
for(int i = 0; i < sizeY; ++i){
maprow.push_back(0);
}
for(int j=0;j<sizeX;++j)
{
map.push_back(maprow);
}

//end map creation





  // we throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;

    PlayerClient robot(gHostname, gPort);
    Position2dProxy pp(&robot, gIndex);
    LaserProxy lp(&robot, gIndex);

    std::cout << robot << std::endl;

    pp.SetMotorEnable (true);

    // start loop
    while(!finished)
    {
      double newspeed = 0;
      double newturnrate = 0;


      robot.Read();

	double x= pp.GetXPos();
	double y= pp.GetYPos();
	double yaw=pp.GetYaw();
double range;


double goToX=0;
double goToY=0;



//map what we see
if(lp.GetCount()>0){
for(int lc=0;lc<3;lc++){
if(lc==0){
range= lp.GetRange(180);
yaw=pp.GetYaw();
}
if(lc==1){
range= lp.GetRange(360);
yaw=pp.GetYaw()+(3.14/2);
}
if(lc==2){
range= lp.GetRange(0);
yaw=pp.GetYaw()-(3.14/2);
}

int irange=(int)floor(range);


double vx,vy;
int wx,wy;
if(range<8){
vx=cos(yaw)*range;
vy=sin(yaw)*range;

wx=(int)floor(vx+x);
wy=(int)floor(vy+y);
if(wx>7)wx=7;
if(wy>7)wy=7;
if(wx<-8)wx=-8;
if(wy<-8)wy=-8;

map[wy+8][wx+8]=2;
}


for(double i=0;i<irange-1;i+=0.1){

vx=cos(yaw)*i;
vy=sin(yaw)*i;

wx=(int)floor(vx+x);
wy=(int)floor(vy+y);
if(wx>7)wx=7;
if(wy>7)wy=7;
if(wx<-8)wx=-8;
if(wy<-8)wy=-8;
if(map[wy+8][wx+8]==0)map[wy+8][wx+8]=1;
}
}


//get nearest unknown location
int dist;
dist=100;

if(map[goaly][goalx]>0 || goaly==-1 ||goalx==-1){
validcount=0;
for(int i = 0; i < sizeY; ++i){
for(int j = 0; j < sizeX; ++j){

if(map[i][j]==0){

//first check if we can reach it
bool valid=false;
if((i-1)>0 && map[i-1][j]==1){ 

valid=true;
}
if((i+1)<sizeY && map[i+1][j]==1){ 

valid=true;
}
if((j-1)>0 && map[i][j-1]==1) {

valid=true;
}
if((j+1)<sizeX && map[i][j+1]==1){ 

valid=true;
}

//then get distance and check if its a min (meaning closest)
if(valid){
validcount++;
int newdist=sqrt(pow(floor(x+8)-j,2)+pow(floor(y+8)-i,2));
if(newdist<dist)
{dist=newdist;goalx=j;goaly=i;}
}



}//if
}//for j
}//for i
}


//create costmap for planning
vector<int> costrow;
vector< vector <int> > cost;
for(int i = 0; i < sizeY; ++i){
costrow.push_back(1000);
}
for(int j=0;j<sizeX;++j)
{
cost.push_back(costrow);
}


//get a valid location next to that goal
vector<int> nextTo= GetInputs(goaly, goalx, map);

if(nextTo.size()>0){

nextTo=NextState(goaly,goalx,nextTo[0]);

//set cost of the that spot to 0(we want to go there)
cost[nextTo[0]][nextTo[1]]=0;
}



//start pathfinding
bool go=true;
vector< vector <int> > cost2;
while(go){
cost2=Plan(map,cost);
bool equal=true;
for(int i=0;i<sizeY;++i){
for(int j=0;j<sizeX;++j){
if(cost[i][j]!=cost2[i][j])equal=false;
}}

if(!equal){
cost=cost2;
}
else go=false;
}
//end path finding


//set go to next cheap spot from current location
int minCost=1000;
vector<int> options;
options.push_back(0);
options.push_back(1);
options.push_back(2);
options.push_back(3);

for(int i=0;i<4;i++){
vector<int> next=NextState(y+8,x+8,options[i]);
if(cost[next[0]][next[1]]<minCost){
goToY=(double)next[0]-7.5;
goToX=(double)next[1]-7.5;
minCost=cost[next[0]][next[1]];

}
}
if (minCost>=1000){
goaly=-1;
goalx=-1;

}
cout<<"goalX:"<<goalx<<" goalY:"<<goaly<<"\n";
cout<<"goToX:"<<goToX+8<<" goToY:"<<goToY+8<<"\n";

//print map

for(int i = sizeY-1; i>=0; --i){
for(int j=0;j<sizeX;++j)
{
std::cout<<cost[i][j]<<" ";
}
std::cout<<"\n";
}

}//end mapping




//start motion

//print map

for(int i = sizeY-1; i>=0; --i){
for(int j=0;j<sizeX;++j)
{
std::cout<<map[i][j]<<" ";
}
std::cout<<"\n";
}

std::cout<<"X:"<<x+8<<"  Y:"<<y+8<<" Yaw:"<<yaw<<"\n";

pp.GoTo(goToX,goToY,pp.GetYaw());


//cout<<validcount<<endl;
if(validcount==0)finished=true;
if(finished){cout<<"DONE!"<<endl;}
    }
  }
  catch (PlayerCc::PlayerError & e)
  {
    std::cerr << e << std::endl;
    return -1;
  }
}
