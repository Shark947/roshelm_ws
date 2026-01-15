/************************************************************/
/*    NAME: Xinyu An                                        */
/*    ORGN: Zhejiang University                             */
/*    FILE: NavServer.h                                          */
/*    DATE: 2021-11-25                                      */
/************************************************************/

#ifndef NavServer_HEADER
#define NavServer_HEADER

#ifndef UNIX
#define UNIX
#endif

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "MOOS/libMOOS/Comms/XPCTcpSocket.h"
#include "AngleUtils.h"

#include <vector>
// #include <pthread.h>
// #include <sys/socket.h>
// #include <netinet/in.h> //structure sockaddr_in
// #include <arpa/inet.h>  //Func : htonl; htons; ntohl; ntohs
// #include <assert.h>     //Func :assert

#include<stdlib.h>
#include<pthread.h>
#include<sys/socket.h>
#include<sys/types.h>       //pthread_t , pthread_attr_t and so on.
#include<stdio.h>
#include<netinet/in.h>      //structure sockaddr_in
#include<arpa/inet.h>       //Func : htonl; htons; ntohl; ntohs
#include<assert.h>          //Func :assert
#include<string.h>          //Func :memset
#include<unistd.h>          //Func :close,write,read

#include <set>

// #define SOCK_PORT 5000
#define BUFFER_LENGTH 1024
#define MAX_CONN_LIMIT 512 //MAX connection limit
struct thr_Param{
  // void *sock_fd;
  // int sock_fd;
  XPCTcpSocket* sock_fd;
  // double dfOpticalNavVal[3];
  double * dfOpticalNavVal;
};

class NavServer : public AppCastingMOOSApp
{
public:
  NavServer();
  ~NavServer();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();
  // static void Data_handle(void *sock_fd); //Only can be seen in the file

protected:
  void RegisterVariables();
  bool ReceiveData(); //Only can be seen in the file
  bool tryConnect();
  int  isSocketConnected(XPCTcpSocket *_socket);
  void ensureConnect(XPCTcpSocket *_socket);
  int  tryWrite();
  int  tryRead();

private: // Configuration variables
  unsigned int m_iterations;
  double m_timewarp;
  double dfDockHeading = 0.0;
  double dfDockPitch = 0.0;
  double dfDockRoll = 0.0;
  double dfIMURollAmountBias = 0;
  double dfIMUPitchAmountBias = 0;
  double dfDepthBias = 0.1;
  double m_dfDistanceBias = 0.3;
  double dfAngleBias = 0.5;
  double dfFallDepth = 3.0;
  double dfCameraViewAngle = 25.0;
  std::set<double> setAlignDepth;
  std::vector<std::vector<double>> dfvAlignDepth;
  int    nMaxtryMinuteNum = 1;
  int    nConstantDepthMinuteNum = 5;
  int    nCtnuInvalidDataCount = 0;
  int    nConstantDepthCount = 0;
  double m_dfDepthBias = 0.0;
  double m_dfDepthGroundBias = 0.0;
  double m_dfDepthCameraBias = 0.528;
  double m_dfDockPanel = 0.28;
  double m_nLastPhaseFloatTime = 500; // in ms
  int    m_nTransimitDuration = 30;
  int    m_nDockingMaxTry = 3;

private: // State variables
  // char * IP = "192.168.1.213";
  std::string sIPaddr;
  int  SOCK_PORT = 5000;
  XPCTcpSocket *socketClient;
  XPCTcpSocket * socketOPTServer;
  std::vector<double> dfvPhaseDepth;
  int nPhaseNum = 1;

  int sockfd_server;
  int sockfd;
  int fd_temp;
  struct sockaddr_in s_addr_in;
  struct sockaddr_in s_addr_client;
  int client_length;
  std::string sMode;
  thr_Param thrParam;
  double dfOpticalNavLoc[3];
  double dfNavDepth, dfNavX, dfNavY, dfNavHeading;
  double dfDockDepth, dfDeltaL, dfDX, dfDY, dfInnerRadius, dfOuterRadius, dfCurrentDepth;
  double dradius,dinheading,distance,previousdistance = 0,radius1th,phrase;
  bool bDataFlag = false;
  double dfLastIMUX = 0.0, dfLastIMUY = 0.0, dfCurrentIMUX, dfCurrentIMUY;
  int nPhaseCount = 1;
  bool bRetryLastPhase = false;
  double dfCameraDepth = 0.0;
  double dfLightDepth = 0.0;
  bool bDockingPhase = false;
  int nCntuDockingCount = 0;
  int nDockingTryCount = 0;
  int nRetryLastPhase = 0;
  int nFloatIteration = 0;
};

#endif
