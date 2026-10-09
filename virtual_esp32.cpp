#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

static std::atomic<bool> running{true};
static std::mutex stateMutex;
static int motorL=0,motorR=0,battery=100;
static float temperature=29.0f;
static double x=60,y=60,heading=0;
static bool collision=false;
static int ultrasonicCm=250;
static std::vector<std::pair<double,double>> path{{60,60}};
struct Rect{double x,y,w,h;};
static const std::vector<Rect> obstacles={{180,40,40,160},{320,200,180,40},{100,280,160,40},{430,60,40,100}};
static constexpr double WORLD_W=600,WORLD_H=400,CELL=20,DT=0.5,CM_PER_TICK=0.75;

static double normAngle(double a){while(a>180)a-=360;while(a<=-180)a+=360;return a;}
static bool inside(double px,double py,const Rect& r,double pad=7){
    return px>=r.x-pad&&px<=r.x+r.w+pad&&py>=r.y-pad&&py<=r.y+r.h+pad;
}
static double rayDistance(double ox,double oy,double deg){
    const double rad=deg*M_PI/180.0,dx=std::cos(rad),dy=std::sin(rad);
    double best=std::min({ox, WORLD_W-ox, oy, WORLD_H-oy});
    if(std::abs(dx)>1e-9){double t=(dx>0?(WORLD_W-ox)/dx:(0-ox)/dx);if(t>0)best=std::min(best,t);}
    if(std::abs(dy)>1e-9){double t=(dy>0?(WORLD_H-oy)/dy:(0-oy)/dy);if(t>0)best=std::min(best,t);}
    for(const auto& r:obstacles){
        double tx=1e9,ty=1e9;
        if(std::abs(dx)>1e-9){double t1=(r.x-ox)/dx,t2=(r.x+r.w-ox)/dx;if(t1>0&&oy+t1*dy>=r.y&&oy+t1*dy<=r.y+r.h)tx=t1;if(t2>0&&oy+t2*dy>=r.y&&oy+t2*dy<=r.y+r.h)tx=std::min(tx,t2);}
        if(std::abs(dy)>1e-9){double t1=(r.y-oy)/dy,t2=(r.y+r.h-oy)/dy;if(t1>0&&ox+t1*dx>=r.x&&ox+t1*dx<=r.x+r.w)ty=t1;if(t2>0&&ox+t2*dx>=r.x&&ox+t2*dx<=r.x+r.w)ty=std::min(ty,t2);}
        best=std::min(best,std::min(tx,ty));
    }
    return std::max(0.0,std::min(250.0,best));
}
static void physicsTick(){
    std::lock_guard<std::mutex> lock(stateMutex);
    const double left=motorL/100.0,right=motorR/100.0;
    const double avg=(left+right)/2.0;
    const double turn=(right-left)*70.0*DT;
    heading=normAngle(heading+turn);
    const double rad=heading*M_PI/180.0;
    const double nx=x+std::cos(rad)*avg*CM_PER_TICK*20.0;
    const double ny=y+std::sin(rad)*avg*CM_PER_TICK*20.0;
    bool hit=nx<7||nx>WORLD_W-7||ny<7||ny>WORLD_H-7;
    for(const auto& r:obstacles) if(inside(nx,ny,r)) hit=true;
    if(hit){collision=true;motorL=0;motorR=0;}
    else{collision=false;x=nx;y=ny;if(std::abs(avg)>0.01&&path.size()<3000)path.push_back({x,y});}
    if((motorL||motorR)&&battery>0) battery=std::max(0,battery-1);
    temperature=29.0f+(std::abs(motorL)+std::abs(motorR))*0.015f;
}
static int frontDistance(){ return (int)std::round(rayDistance(x,y,heading)); }

std::string hardware(){
 return R"({"ok":true,"type":"wheeled","name":"Virtual ESP32 Rover","camera":false,"sensors":[{"id":"front_distance","type":"ultrasonic","unit":"cm"},{"id":"collision","type":"digital","unit":"bool"}],"actuators":[{"id":"motor_l","type":"motor","unit":"percent"},{"id":"motor_r","type":"motor","unit":"percent"}],"panels":["DRIVETRAIN","VIRTUAL_MAP","CODE","CIRCUIT"],"map":{"width":600,"height":400,"cell":20},"firmware":"virtual-esp32","version":"4.0"})";
}
std::string telemetry(){
 std::lock_guard<std::mutex> lock(stateMutex);
 std::ostringstream o;
 o<<"{\"type\":\"telemetry\",\"state\":{\"connected\":true,\"battery\":"<<battery<<",\"currentCommand\":\""<<(motorL==0&&motorR==0?"STOP":"DRIVE")<<"\",\"speed\":"<<(motorL+motorR)/2<<",\"sensors\":{\"front_distance\":"<<frontDistance()<<",\"collision\":"<<(collision?"true":"false")<<"},\"actuators\":{\"motor_l\":"<<motorL<<",\"motor_r\":"<<motorR<<"},\"pose\":{\"x\":"<<x<<",\"y\":"<<y<<",\"heading\":"<<heading<<"},\"camera\":null,\"link\":{\"latency\":0}}}";
 return o.str();
}
std::string world(){
 std::lock_guard<std::mutex> lock(stateMutex);
 std::ostringstream o;
 o<<"{\"width\":600,\"height\":400,\"cell\":20,\"robot\":{\"x\":"<<x<<",\"y\":"<<y<<",\"heading\":"<<heading<<"},\"frontDistance\":"<<frontDistance()<<",\"motorL\":"<<motorL<<",\"motorR\":"<<motorR<<",\"battery\":"<<battery<<",\"collision\":"<<(collision?"true":"false")<<",\"obstacles\":[";
 for(size_t i=0;i<obstacles.size();++i){if(i)o<<",";o<<"{\"x\":"<<obstacles[i].x<<",\"y\":"<<obstacles[i].y<<",\"w\":"<<obstacles[i].w<<",\"h\":"<<obstacles[i].h<<"}";}
 o<<"],\"path\":[";
 size_t start=path.size()>600?path.size()-600:0;
 for(size_t i=start;i<path.size();++i){if(i>start)o<<",";o<<"{\"x\":"<<path[i].first<<",\"y\":"<<path[i].second<<"}";}
 o<<"]}";
 return o.str();
}
std::string executeCommand(const std::string& cmd){
 std::lock_guard<std::mutex> lock(stateMutex);
 std::istringstream in(cmd);std::string op;int value=0;in>>op>>value;
 value=std::max(0,std::min(100,value));std::cout<<"[COMMAND] "<<cmd<<"\n";
 if(op=="DRIVE_FORWARD"||op=="WALK_FORWARD"||op=="FORWARD")motorL=std::abs(value),motorR=std::abs(value);
 else if(op=="DRIVE_BACKWARD"||op=="WALK_BACKWARD"||op=="BACKWARD")motorL=-std::abs(value),motorR=-std::abs(value);
 else if(op=="TURN_LEFT")motorL=-std::abs(value),motorR=std::abs(value);
 else if(op=="TURN_RIGHT")motorL=std::abs(value),motorR=-std::abs(value);
 else if(op=="SET_MOTOR_L")motorL=value;
 else if(op=="SET_MOTOR_R")motorR=value;
 else if(op=="STOP"||op=="E_STOP"||op=="SIT"||op=="LAND")motorL=0,motorR=0;
 else return "ERR UNKNOWN_COMMAND";
 return "ACK "+cmd;
}
void sendLine(int fd,const std::string&s){std::string out=s+"\n";send(fd,out.c_str(),out.size(),0);}
void handleClient(int fd,sockaddr_in addr){
 char ip[INET_ADDRSTRLEN]{};inet_ntop(AF_INET,&addr.sin_addr,ip,sizeof(ip));
 std::string buffer;char temp[4096];
 while(running){ssize_t n=recv(fd,temp,sizeof(temp)-1,0);if(n<=0)break;temp[n]=0;buffer+=temp;size_t pos;
  while((pos=buffer.find('\n'))!=std::string::npos){std::string line=buffer.substr(0,pos);buffer.erase(0,pos+1);if(line.empty())continue;
   if(line=="PING")sendLine(fd,"PONG");
   else if(line=="GET_HARDWARE")sendLine(fd,hardware());
   else if(line=="GET_TELEMETRY")sendLine(fd,telemetry());
   else if(line=="GET_WORLD")sendLine(fd,world());
   else if(line.rfind("COMMAND ",0)==0)sendLine(fd,executeCommand(line.substr(8)));
   else {try{if(line.find("\"type\":\"hello\"")!=std::string::npos)sendLine(fd,hardware());else if(line.find("\"type\":\"command\"")!=std::string::npos){auto p=line.find("\"cmd\"");auto q=line.find("\"",p+5);auto r=line.find("\"",q+1);std::string c=(q!=std::string::npos&&r!=std::string::npos)?line.substr(q+1,r-q-1):"STOP";auto a=line.find("\"arg\"");int v=0;if(a!=std::string::npos){auto colon=line.find(':',a);v=std::atoi(line.c_str()+colon+1);}sendLine(fd,executeCommand(c+" "+std::to_string(v)));}else sendLine(fd,"ERR UNKNOWN_REQUEST");}catch(...){sendLine(fd,"ERR BAD_JSON");}}
  }
 }
 close(fd);
}
void discoveryLoop(){
 int fd=socket(AF_INET,SOCK_DGRAM,0);if(fd<0)return;int yes=1;setsockopt(fd,SOL_SOCKET,SO_BROADCAST,&yes,sizeof(yes));setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
 sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=INADDR_ANY;a.sin_port=htons(4210);if(bind(fd,(sockaddr*)&a,sizeof(a))<0){close(fd);return;}
 char b[1024];while(running){sockaddr_in from{};socklen_t fl=sizeof(from);ssize_t n=recvfrom(fd,b,sizeof(b)-1,0,(sockaddr*)&from,&fl);if(n<=0)continue;b[n]=0;std::string q(b);if(q.find("ROBOT_DISCOVER")!=std::string::npos||q.find("\"type\":\"discover\"")!=std::string::npos){std::string j=R"({"type":"robot","protocol":1,"id":"VESP32-01","name":"Virtual ESP32 Rover","robot_type":"wheeled","tcp_port":5000,"firmware":"virtual-esp32","version":"4.0"})";sendto(fd,j.c_str(),j.size(),0,(sockaddr*)&from,fl);}}
 close(fd);
}
void telemetryLoop(){using namespace std::chrono_literals;while(running){physicsTick();ultrasonicCm=frontDistance();std::this_thread::sleep_for(500ms);}}
int main(){
 std::cout<<"VIRTUAL ESP32 ROVER 4.0\nTCP :5000 | UDP :4210\n";
 int server=socket(AF_INET,SOCK_STREAM,0);if(server<0)return 1;int yes=1;setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_addr.s_addr=INADDR_ANY;sa.sin_port=htons(5000);if(bind(server,(sockaddr*)&sa,sizeof(sa))<0)return 1;if(listen(server,8)<0)return 1;
 std::thread d(discoveryLoop),t(telemetryLoop);while(running){sockaddr_in c{};socklen_t l=sizeof(c);int fd=accept(server,(sockaddr*)&c,&l);if(fd>=0)std::thread(handleClient,fd,c).detach();}running=false;d.join();t.join();close(server);
}