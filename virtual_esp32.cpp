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

static std::atomic<bool> running{true};
static std::mutex stateMutex;
static int frontDistance=75, motorL=0, motorR=0, battery=100;
static float temperature=29.0f;

std::string hardware() {
    return R"({"ok":true,"type":"wheeled","name":"Virtual ESP32 Robot","camera":false,"sensors":[{"id":"front_distance","type":"ultrasonic","unit":"cm"}],"actuators":[{"id":"motor_l","type":"motor","unit":"percent"},{"id":"motor_r","type":"motor","unit":"percent"}],"panels":["DRIVETRAIN"]})";
}
std::string telemetry() {
    std::lock_guard<std::mutex> lock(stateMutex);
    std::ostringstream o;
    o<<"{\"ok\":true,\"front_distance\":"<<frontDistance<<",\"motor_l\":"<<motorL<<",\"motor_r\":"<<motorR<<",\"battery\":"<<battery<<",\"temperature\":"<<temperature<<",\"latency\":0,\"camera\":null}";
    return o.str();
}
std::string executeCommand(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(stateMutex);
    std::cout<<"[COMMAND] "<<cmd<<"\n";
    std::istringstream in(cmd); std::string op; int value=0; in>>op>>value;
    if(op=="DRIVE_FORWARD"||op=="WALK_FORWARD"||op=="FORWARD") motorL=value,motorR=value;
    else if(op=="DRIVE_BACKWARD"||op=="WALK_BACKWARD"||op=="BACKWARD") motorL=-value,motorR=-value;
    else if(op=="TURN_LEFT") motorL=-value,motorR=value;
    else if(op=="TURN_RIGHT") motorL=value,motorR=-value;
    else if(op=="SET_MOTOR_L") motorL=value;
    else if(op=="SET_MOTOR_R") motorR=value;
    else if(op=="STOP"||op=="E_STOP"||op=="SIT"||op=="LAND") motorL=0,motorR=0;
    else return "ERR UNKNOWN_COMMAND";
    return "ACK "+cmd;
}
void sendLine(int fd,const std::string& s){
    std::string out=s+"\n"; send(fd,out.c_str(),out.size(),0); std::cout<<"> "<<s<<"\n";
}
void handleClient(int fd,sockaddr_in addr){
    char ip[INET_ADDRSTRLEN]{};
    inet_ntop(AF_INET,&addr.sin_addr,ip,sizeof(ip));
    std::cout<<"\n[CLIENT] Connected: "<<ip<<":"<<ntohs(addr.sin_port)<<"\n";
    std::string buffer; char temp[2048];
    while(running){
        ssize_t n=recv(fd,temp,sizeof(temp)-1,0); if(n<=0) break;
        temp[n]=0; buffer+=temp;
        size_t pos;
        while((pos=buffer.find('\n'))!=std::string::npos){
            std::string line=buffer.substr(0,pos); buffer.erase(0,pos+1);
            while(!line.empty()&&(line.back()=='\r'||line.back()==' ')) line.pop_back();
            if(line.empty()) continue;
            std::cout<<"< "<<line<<"\n";
            if(line=="PING") sendLine(fd,"PONG");
            else if(line=="GET_HARDWARE") sendLine(fd,hardware());
            else if(line=="GET_TELEMETRY") sendLine(fd,telemetry());
            else if(line.rfind("COMMAND ",0)==0) sendLine(fd,executeCommand(line.substr(8)));
            else sendLine(fd,"ERR UNKNOWN_REQUEST");
        }
    }
    close(fd); std::cout<<"[CLIENT] Disconnected\n";
}
void discoveryLoop(){
    int fd=socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){ perror("discovery socket"); return; }

    int yes=1;
    setsockopt(fd,SOL_SOCKET,SO_BROADCAST,&yes,sizeof(yes));
    setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));

    sockaddr_in bindAddr{};
    bindAddr.sin_family=AF_INET;
    bindAddr.sin_addr.s_addr=INADDR_ANY;
    bindAddr.sin_port=htons(4210);

    if(bind(fd,(sockaddr*)&bindAddr,sizeof(bindAddr))<0){
        perror("discovery bind");
        close(fd);
        return;
    }

    std::cout<<"[DISCOVERY] UDP listening on :4210\n";

    char buffer[1024];
    while(running){
        sockaddr_in from{};
        socklen_t fromLen=sizeof(from);
        ssize_t n=recvfrom(fd,buffer,sizeof(buffer)-1,0,(sockaddr*)&from,&fromLen);
        if(n<=0) continue;
        buffer[n]=0;

        std::string request(buffer);
        while(!request.empty() && (request.back()=='\r'||request.back()=='\n'||request.back()==' ')) request.pop_back();

        if(request=="ROBOT_DISCOVER"){
            std::string json=R"({"ip":"127.0.0.1","port":5000,"name":"Virtual ESP32 Robot","type":"wheeled"})";
            sendto(fd,json.c_str(),json.size(),0,(sockaddr*)&from,fromLen);
            char ip[INET_ADDRSTRLEN]{};
            inet_ntop(AF_INET,&from.sin_addr,ip,sizeof(ip));
            std::cout<<"[DISCOVERY] Request from "<<ip<<":"<<ntohs(from.sin_port)<<" -> replied\n";
        }
    }
    close(fd);
}

void telemetryLoop(){
    using namespace std::chrono_literals;
    while(running){
        { std::lock_guard<std::mutex> lock(stateMutex);
          int forward=(motorL+motorR)/2;
          if(forward>0) frontDistance-=2; else if(forward<0) frontDistance+=2;
          if(frontDistance<10) frontDistance=10; if(frontDistance>120) frontDistance=120;
          if(battery>0&&(motorL!=0||motorR!=0)) battery--;
          temperature=29.0f+(std::abs(motorL)+std::abs(motorR))*0.015f;
        }
        std::this_thread::sleep_for(500ms);
    }
}
int main(){
    std::cout<<"========================================\n";
    std::cout<<"        VIRTUAL ESP32 ROBOT             \n";
    std::cout<<"========================================\n";
    std::cout<<"[BOOT] Virtual ESP32 started\n";
    std::cout<<"[NET]  TCP listening on 0.0.0.0:5000\n";
    std::cout<<"[INFO] Waiting for Robot Control Core...\n";
    int server=socket(AF_INET,SOCK_STREAM,0); if(server<0){perror("socket");return 1;}
    int yes=1; setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
    sockaddr_in sa{}; sa.sin_family=AF_INET; sa.sin_addr.s_addr=INADDR_ANY; sa.sin_port=htons(5000);
    if(bind(server,(sockaddr*)&sa,sizeof(sa))<0){perror("bind");close(server);return 1;}
    if(listen(server,8)<0){perror("listen");close(server);return 1;}
    std::thread discovery(discoveryLoop);
    std::thread t(telemetryLoop);
    while(running){ sockaddr_in client{}; socklen_t len=sizeof(client); int fd=accept(server,(sockaddr*)&client,&len); if(fd<0) continue; std::thread(handleClient,fd,client).detach(); }
    running=false; discovery.join(); t.join(); close(server); return 0;
}