/************************************************************/
/*    NAME: Xinyu An                                        */
/*    ORGN: Zhejiang University                             */
/*    FILE: NavServer.cpp                                        */
/*    DATE: 2021-11-25                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ColorParse.h"
#include "NavServer.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include <iostream>
#include <cmath> 
#include <cstdlib>


using namespace std;


// static void Data_handle(void *sock_fd); //Only can be seen in the file
static void Data_handle(void *arg); //Only can be seen in the file

//---------------------------------------------------------
// Constructor

NavServer::NavServer()
{
  m_iterations = 0;
  m_timewarp = 1;

  dfDockDepth = 0.0;
  distance=0;
  nPhaseCount=0;
  // FILE *fp = NULL;
  // if ((fp = popen("canSend 0x7e9 72010000", "r")) == NULL)
  // {
  //   printf("Close power failed\n");
  // }
  // pclose(fp);
  
  // printf("Waiting for power on\n");
  // sleep(5);
  // if ((fp = popen("canSend 0x7e9 71010000", "r")) == NULL)
  // {
  //   printf("Open power failed\n");
  // }
  // pclose(fp);
  // printf("Waiting for system on\n");
  // sleep(10);
}

//---------------------------------------------------------
// Destructor

NavServer::~NavServer()
{
  // int ret = shutdown(sockfd_server,SHUT_WR); //shut down the all or part of a full-duplex connection.
  //   assert(ret != -1);

  //   printf("Server shuts down\n");
  socketClient->vCloseSocket();
  socketOPTServer->vCloseSocket();
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool NavServer::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool NavServer::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  m_timewarp = GetMOOSTimeWarp();

  double dNavServerPeriod = 1.0;
  string sAlignDepth = "";
  bool   bbar = false;
  if (!m_MissionReader.GetValue("NavServerPeriod", dNavServerPeriod))
  {
    MOOSTrace("NavServer communication period not set!!!");
    return false;
  }
  if (!m_MissionReader.GetValue("DepthBias", m_dfDepthBias))
  {
    MOOSTrace("DepthBias not set!!!");
    return false;
  }
  if (!m_MissionReader.GetValue("DepthGroundBias", m_dfDepthGroundBias))
  {
    MOOSTrace("DepthGroundBias not set!!!");
    return false;
  }
  if (!m_MissionReader.GetValue("DepthCameraBias", m_dfDepthCameraBias))
  {
    MOOSTrace("DepthCameraBias not set!!!");
    return false;
  }
  if (!m_MissionReader.GetValue("DockPanel", m_dfDockPanel))
  {
    MOOSTrace("DockPanel not set!!!");
    return false;
  }
  if (!m_MissionReader.GetValue("DockHeading", dfDockHeading))
  {
    MOOSTrace("DockHeading not set!!!");
    return false;
  }
  dfDockHeading = angle360(dfDockHeading + 90);
  if (!m_MissionReader.GetValue("DockPitch", dfDockPitch))
  {
    MOOSTrace("DockPitch not set!!!");
    return false;
  }
  // dfDockPitch = angle360(dfDockPitch + 90);
  if (!m_MissionReader.GetValue("DockRoll", dfDockRoll))
  {
    MOOSTrace("DockRoll not set!!!");
    return false;
  }
  // dfDockRoll = angle360(dfDockRoll + 90);
  // if (!m_MissionReader.GetValue("IMURollAmountBias", dfIMURollAmountBias))
  // {
  //   MOOSTrace("IMURollAmountBias not set!!!");
  //   return false;
  // }
  // if (!m_MissionReader.GetValue("IMUPitchAmountBias", dfIMUPitchAmountBias))
  // {
  //   MOOSTrace("IMUPitchAmountBias not set!!!");
  //   return false;
  // }
  
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string orig  = *p;
      string line  = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;

      bool handled = false;
      if(param == tolower("NavServerPeriod")) {
        dNavServerPeriod = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("IPAddress")) {
        sIPaddr = value;
        handled = true;
      }
      else if(param == tolower("SocketPORT")) {
        SOCK_PORT = atoi(value.c_str());
        handled = true;
      }
      else if(param == tolower("OpticalCameraDeltaL")) {
        dfDeltaL = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("CameraViewAngle")) {
        dfCameraViewAngle = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("DockDepth")) {
        dfDockDepth = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("FallDepth")) {
        dfFallDepth = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("AlignDepth")) {
        sAlignDepth = value;
        handled = true;
      }
      else if(param == tolower("PhaseDepth")) {
        string sPhaseDepth = value;
        double dfDepth = 0.0;
        cout << "Phase depth: ";
        while (!value.empty())
        {
          sPhaseDepth = MOOSChomp(value, ",");
          dfDepth = atof(sPhaseDepth.c_str());
          // dfvPhaseDepth.push_back(dfDepth);
          cout << dfDepth << ",";
        }
        cout << "(" << dfvPhaseDepth.size() << ")" << endl;
        nPhaseNum = dfvPhaseDepth.size()+1;
        handled = true;
      }
      else if(param == tolower("dradius")) {
        dradius = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("radius1th")) {
        radius1th = atof(value.c_str());
        handled = true;
      }
      // else if(param == tolower("DockHeading")) {
      //   dfDockHeading = angle360(atof(value.c_str()) + 90);
      //   handled = true;
      // }
      // else if(param == tolower("DockPitch")) {
      //   dfDockPitch = atof(value.c_str());
      //   handled = true;
      // }
      // else if(param == tolower("DocRoll")) {
      //   dfDockRoll = atof(value.c_str());
      //   handled = true;
      // }
      else if(param == tolower("DistanceBias")) {
        m_dfDistanceBias = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("AngleBias")) {
        dfAngleBias = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("dinheading")) {
        dinheading = atof(value.c_str());
        handled = true;
      }
      else if(param == tolower("MaxtryMinuteNum")) {
        nMaxtryMinuteNum = atoi(value.c_str());
        handled = true;
      }
      else if(param == tolower("ConstantDepthMinuteNum")) {
        nConstantDepthMinuteNum = atoi(value.c_str());
        handled = true;
      }

      else if(param == tolower("LastPhaseFloatTime")) {
        m_nLastPhaseFloatTime = atoi(value.c_str());
        handled = true;
      }

      else if(param == tolower("TransimitDuration")) {
        m_nTransimitDuration = atoi(value.c_str());
        handled = true;
      }

      else if(param == tolower("DockingMaxTry")) {
        m_nDockingMaxTry = atoi(value.c_str());
        handled = true;
      }

      if(!handled)
        reportUnhandledConfigWarning(orig);
    }
  }
  else
    reportConfigWarning("No config block found for " + GetAppName());

  dfLightDepth = dfDockDepth - m_dfDockPanel;
  string sTmpRaw = sAlignDepth;
  double dfTmpDepth = 0.0;
  cout << "Dockdepth: " << dfDockDepth <<  ", Aligning depth set (" << sAlignDepth << "): ";
  while (!sAlignDepth.empty())
  {
    sTmpRaw = MOOSChomp(sAlignDepth, ":");
    dfTmpDepth = dfLightDepth - atof(sTmpRaw.c_str());
    // nVolume = atoi(sTmpRaw.c_str());
    setAlignDepth.insert(dfTmpDepth);
    cout << dfTmpDepth << ", ";
  }
  
  nPhaseCount = 0;
  for (set<double>::iterator it_s = setAlignDepth.begin(); it_s != setAlignDepth.end(); it_s++)
  {
    nPhaseCount++;
    dfOuterRadius = (dfLightDepth - m_dfDepthBias - m_dfDepthGroundBias - *it_s) * tan(degToRadians(dfCameraViewAngle));
    if(nPhaseCount == setAlignDepth.size())
    dfInnerRadius = dradius;
    else
    {
      dfInnerRadius = (dfLightDepth - m_dfDepthBias - m_dfDepthGroundBias - *(++it_s)) * tan(degToRadians(dfCameraViewAngle));
      it_s--;
    }
    dfvAlignDepth.push_back({*it_s, dfInnerRadius, dfOuterRadius});
    cout << nPhaseCount << ":" << *it_s << "," << dfvAlignDepth[nPhaseCount-1][1] << "," << dfvAlignDepth[nPhaseCount-1][2] << "|";
  }
  nPhaseCount = 0;
  nPhaseNum = dfvAlignDepth.size()+1;
  cout << "(" << setAlignDepth.size() << "," << nPhaseNum << ")" << endl;

  // Add MOOS dynamic/run variables: AddMOOSVariable(sName, sSubscribeName, sPublishName, dfCommsTime)
  // AddMOOSVariable("", "", "", dNavServerPeriod);
  AddMOOSVariable("DESIRED_SPEED", "", "DESIRED_SPEED", dNavServerPeriod);
  AddMOOSVariable("DOCKHDG_UPDATES", "", "DOCKHDG_UPDATES", dNavServerPeriod);
  AddMOOSVariable("DOCKDEPTH_UPDATE", "", "DOCKDEPTH_UPDATE", dNavServerPeriod);
  AddMOOSVariable("STATIONING", "", "STATIONING", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAV", "MODE", "MODE", dNavServerPeriod);
  AddMOOSVariable("CONSTHEIGHT", "", "CONSTHEIGHT", dNavServerPeriod);
  AddMOOSVariable("OPTICALDOCKDEPTH", "DOCK_DEPTH", "DOCK_DEPTH", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVDEPTH", "DEPTHHEIGHT_DEPTHRAW", "", dNavServerPeriod);
  AddMOOSVariable("IMUX", "IMU_X", "", dNavServerPeriod);
  AddMOOSVariable("IMUY", "IMU_Y", "", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVX", "", "OPTICAL_X", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVY", "", "OPTICAL_Y", dNavServerPeriod);
  AddMOOSVariable("OPTICALSIMX", "", "SIM_X", dNavServerPeriod);
  AddMOOSVariable("OPTICALSIMY", "", "SIM_Y", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVHEADING", "NAV_HEADING", "OPT_HEADING", dNavServerPeriod);
  AddMOOSVariable("NAVPITCH", "NAV_PITCH", "", dNavServerPeriod);
  AddMOOSVariable("NAVROLL", "NAV_ROLL", "", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVWPT", "", "WPT_OPTLOC", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVDATA", "", "WPT_OPTNAVDATA", dNavServerPeriod);
  AddMOOSVariable("OPTICALRAW", "", "OPTICAL_RAW", dNavServerPeriod);
  AddMOOSVariable("OPTICALNAVSUMMARY", "", "OPTICAL_SUMMARY", dNavServerPeriod);
  AddMOOSVariable("OPTICALSENDINFO", "", "OPTICAL_SENDINFO", dNavServerPeriod);
  AddMOOSVariable("DOCKING_FALLING", "", "DOCKING_FALLING", dNavServerPeriod);
  AddMOOSVariable("MOOS_MANUAL_OVERIDE", "", "MOOS_MANUAL_OVERIDE", dNavServerPeriod);

  AddMOOSVariable("DOCKINGFAILED", "", "OPTICAL_DOCKINGFAILED", dNavServerPeriod);
  // AddMOOSVariable("OPTICALNAVX", "", "SIM_X", dNavServerPeriod);
  // AddMOOSVariable("OPTICALNAVY", "", "SIM_Y", dNavServerPeriod);
  // AddMOOSVariable("OPTICALNAVZ", "", "SIM_Z", dNavServerPeriod);

  // Register MOOS variables to MOOSDB
  RegisterMOOSVariables();

  SetMOOSVar("OPTICALDOCKDEPTH", dfDockDepth, MOOSTime());
  string sTmp = "heading=" + to_string(dfDockHeading); 
  SetMOOSVar("DOCKHDG_UPDATES", sTmp, MOOSTime());
  PublishFreshMOOSVariables();

  cout << "IP: " << sIPaddr.c_str() << ", PORT: " << SOCK_PORT << endl;
  socketClient = new XPCTcpSocket((long)SOCK_PORT);
  socketClient->iGetKeepAlive();
  bool bBind = false;
  bool bListen = false;
  int nRetryCount = 0;
  while (!bBind || !bListen)
  {
    try
    {
      socketClient->vBindSocket();
      bBind = true;
      socketClient->vListen(1);
      bListen = true;
    }
    catch(XPCException &socketExcept)
    {
      string sTmp = socketExcept.sGetException();
      cout << socketExcept.sGetException() << " [" << nRetryCount++ << "] ," << bBind << "," << bListen << "," << strcmp(sTmp.c_str(), "Error binding") << "," << strcmp(sTmp.c_str(), "Error Listening")<< endl;
      if (strcmp(sTmp.c_str(), "Error binding") > 0)
      bBind = false;
      else if(strcmp(sTmp.c_str(), "Error Listening") > 0)
      bListen = false;
      MOOSPause(2000);
    }
  }
  cout << "Waiting for new connection" << endl;
  
  
  tryConnect();

  // socketClient = socketClient->Accept();
  // cout << "Accept" << endl;
  // printf("A new connection occurs!\n");
  // socketClient = new XPCTcpSocket((long)SOCK_PORT);
  // socketClient->vConnect(sIPaddr.c_str());
  // thrParam.sock_fd = socketClient;
  // thrParam.dfOpticalNavVal = dfOpticalNavLoc;

  // RegisterVariables();
  // setsockopt(listenfd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
  // sleep(55);

  // sockfd_server = socket(AF_INET,SOCK_STREAM,0);  //ipv4,TCP
  // assert(sockfd_server != -1);

  // //before bind(), set the attr of structure sockaddr.
  //   memset(&s_addr_in,0,sizeof(s_addr_in));
  //   s_addr_in.sin_family = AF_INET;
  //   s_addr_in.sin_addr.s_addr = htonl(INADDR_ANY);  //trans addr from uint32_t host byte order to network byte order.
  //   // s_addr_in.sin_addr.s_addr = inet_addr(sIPaddr.c_str());  //trans addr from uint32_t host byte order to network byte order.
  //   // s_addr_in.sin_addr.s_addr = inet_addr(IP);  //trans addr from uint32_t host byte order to network byte order.
  //   s_addr_in.sin_port = htons(SOCK_PORT);          //trans port from uint16_t host byte order to network byte order.
  //   fd_temp = bind(sockfd_server,(__CONST_SOCKADDR_ARG)(&s_addr_in), (socklen_t)sizeof(s_addr_in));
  //   // fd_temp = bind(sockfd_server,(struct scokaddr *)(&s_addr_in), (socklen_t)sizeof(s_addr_in));
  //   int i = 0;
  //   while(fd_temp == -1)
  //   {
  //       fd_temp = bind(sockfd_server,(__CONST_SOCKADDR_ARG)(&s_addr_in), (socklen_t)sizeof(s_addr_in));
  //       fprintf(stderr,"bind error!\n");
  //       sleep(1);
  //       i++;
  //       if (i > 60)
  //       {fprintf(stderr,"bind error and exit!\n");exit(1);}
  //   }

  //   fd_temp = listen(sockfd_server,MAX_CONN_LIMIT);
  //   while(fd_temp == -1)
  //   {
  //       fd_temp = listen(sockfd_server,MAX_CONN_LIMIT);
  //       fprintf(stderr,"listen error!\n");
  //       // exit(1);
  //   }

  // client_length = sizeof(s_addr_client);

  // printf("waiting for new connection...\n");
  // //Block here. Until server accpets a new connection.
  // cout << "Begin accept a connection: " << endl;
  // sockfd = accept(sockfd_server,(struct sockaddr*)(&s_addr_client),(socklen_t *)(&client_length));
  // // sockfd = accept(sockfd_server,(struct sockaddr_*)(&s_addr_client),(socklen_t *)(&client_length));
  // cout << "sockfd: " << sockfd << endl;
  // while(sockfd == -1)
  // {
  //     sockfd = accept(sockfd_server,(struct sockaddr*)(&s_addr_client),(socklen_t *)(&client_length));
  //     fprintf(stderr,"Accept error!\n");
  //     // continue;                               //ignore current socket ,continue while loop.
  // }

  // thrParam.sock_fd = (void * )sockfd;
  // thrParam.sock_fd = sockfd;
  // thrParam.dfOpticalNavVal = dfOpticalNavLoc;

  return(true);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool NavServer::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  // MOOSMSG_LIST::iterator p;

  // for(p=NewMail.begin(); p!=NewMail.end(); p++) {
  //   CMOOSMsg &msg = *p;
  //   string key    = msg.GetKey();

  // #if 0 // Keep these around just for template
  //     string key   = msg.GetKey();
  //     string comm  = msg.GetCommunity();
  //     double dval  = msg.GetDouble();
  //     string sval  = msg.GetString();
  //     string msrc  = msg.GetSource();
  //     double mtime = msg.GetTime();
  //     bool   mdbl  = msg.IsDouble();
  //     bool   mstr  = msg.IsString();
  // #endif

  // if(key == "FOO")
  //   cout << "great!";

  // else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
  //   reportRunWarning("Unhandled Mail: " + key);
  //  }

  //Check MOOSDB for MOOS variables
  if( UpdateMOOSVariables(NewMail) )
  {
    //Get the value of MOOS variable: CMOOSVariable & MOOSVar = *GetMOOSVar(sName)
    // double dTemp = GetMOOSVar("")->GetDoubleVal();
    if (GetMOOSVar("OPTICALNAV")->IsFresh())
    {
      sMode = GetMOOSVar("OPTICALNAV")->GetStringVal();
      if (tolower(sMode) == tolower("DOCKING"))
      {
        nPhaseCount = 1;
        string sTmp = "depth=" + to_string(dfvAlignDepth[nPhaseCount-1][0]);
        SetMOOSVar("DOCKDEPTH_UPDATE", sTmp, MOOSTime());
        dfInnerRadius = dfvAlignDepth[nPhaseCount-1][1];
        dfOuterRadius = dfvAlignDepth[nPhaseCount-1][2]; // Otherwise return to acoustic postion
        cout << sTmp << endl;
      }
    }
    
    dfNavDepth = GetMOOSVar("OPTICALNAVDEPTH")->GetDoubleVal();
    dfCameraDepth = dfNavDepth + m_dfDepthBias + m_dfDepthCameraBias;
    dfNavHeading = GetMOOSVar("OPTICALNAVHEADING")->GetDoubleVal();
    if (GetMOOSVar("OPTICALDOCKDEPTH")->IsFresh())
    {dfDockDepth = GetMOOSVar("OPTICALDOCKDEPTH")->GetDoubleVal();dfLightDepth = dfDockDepth - m_dfDockPanel;}
    if (GetMOOSVar("IMUX")->IsFresh())
    dfCurrentIMUX = GetMOOSVar("IMUX")->GetDoubleVal();
    if (GetMOOSVar("IMUY")->IsFresh())
    dfCurrentIMUY = GetMOOSVar("IMUY")->GetDoubleVal();
    
    // dfNavX = GetMOOSVar("OPTICALNAVX")->GetDoubleVal();
    // dfNavY = GetMOOSVar("OPTICALNAVY")->GetDoubleVal();


  }

  NewMail.clear(); // Added on 2022-11-3
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool NavServer::Iterate()
{
  AppCastingMOOSApp::Iterate();

  m_iterations++;
  cout << "m_iterations: " << m_iterations << endl;
  //Set value/string to MOOS variables: SetMOOSVar(sName, dfVal/sVal, dfTime)
  // SetMOOSVar("", , MOOSTime());

  pthread_t thread_id;
  // if(pthread_create(&thread_id,NULL,(void* (*)(void*))(&Data_handle),(&sockfd)) == -1)
  // cout << "Accept" << endl;
  // printf("A new connection occurs!\n");
  // socketClient = new XPCTcpSocket((long)SOCK_PORT);
  // socketClient->vConnect(sIPaddr.c_str());
  thrParam.sock_fd = socketOPTServer;
  thrParam.dfOpticalNavVal = dfOpticalNavLoc;

  // if(pthread_create(&thread_id,NULL,(void* (*)(void*))(&Data_handle),(&thrParam)) == -1)
  // {
  //     fprintf(stderr,"pthread_create error!\n");
  //     // break;                                  //break while loop
  // }
  // ReceiveData(socketOPTServer, dfOpticalNavLoc);
  // if (!ReceiveData())
  // {
  //   cout << "No optical navigation info" << endl;
  //   #ifdef DEBUG
  //   AppCastingMOOSApp::PostReport();
  //   #endif
  //   return false;
  // }
  bDataFlag = false;
  ReceiveData();
  char chTmp[1000];
  string  sTmp;
  double dfNextX = 0.0, dfNextY = 0.0;
  // pthread_join(thread_id, NULL);
  printf("dHeading=%lf,thetaX=%lf,thetaY=%lf\n",dfOpticalNavLoc[0],dfOpticalNavLoc[1],dfOpticalNavLoc[2]);
  if (bDataFlag)
  {
    nCntuDockingCount++;
    // if (nCntuDockingCount >  m_dfFreq*m_nTransimitDuration && !bDockingPhase)
    if (nCntuDockingCount >  m_dfFreq*m_nTransimitDuration && !bDockingPhase && tolower(sMode) == tolower("CLOSETODOCKING"))
    {
      bDockingPhase = true;
      SetMOOSVar("OPTICALNAV", "DOCKING", MOOSTime());
      SetMOOSVar("CONSTHEIGHT", "false", MOOSTime());
    }
  }
  else 
  nCntuDockingCount = 0;

  if (tolower(sMode) == tolower("DOCKING") || tolower(sMode) == tolower("CLOSETODOCKING"))
  {
    dfNextX = 0.0;
    dfNextY = 0.0;
    if (bDataFlag)
    {
      dfDX = (dfLightDepth - dfCameraDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[1])) * sin(MOOSDeg2Rad(dfNavHeading)) - (dfLightDepth - dfCameraDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[2])) * cos(MOOSDeg2Rad(dfNavHeading)) - dfDeltaL * sin(MOOSDeg2Rad(dfNavHeading));// + dfDeltaL;
      dfDY = -(dfLightDepth - dfCameraDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[2])) * sin(MOOSDeg2Rad(dfNavHeading)) - (dfLightDepth - dfCameraDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[1])) * cos(MOOSDeg2Rad(dfNavHeading)) + dfDeltaL * cos(MOOSDeg2Rad(dfNavHeading));
      
      #ifdef DEBUG
      cout << "[z*tan(thetaY), z*tan(thetaX)]: " << (dfLightDepth - dfCameraDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[2])) << ", " << (dfLightDepth - dfCameraDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[1])) << endl;
      cout << "delta X: " << dfDX << ", delta Y: " << dfDY << endl;
      #endif
      
      dfNextX = dfDY;// + dfNavY;
      dfNextY = dfDX;// + dfNavX;
    }
    else
    {
      dfNextX = dfOpticalNavLoc[1];
      dfNextY = dfOpticalNavLoc[2];
    }
    // string sTmp = "OPT_LOC=\"points=" + to_string(dfNextX) + "," + to_string(dfNextY) + "\"";
    // string sTmp = "OPT_LOC=( 11111, " + to_string(MOOSDeg2Rad(dfOpticalNavLoc[1])) + "," + to_string(MOOSDeg2Rad(dfOpticalNavLoc[2])) + "," + to_string(MOOSDeg2Rad(dfNavHeading)) + "," + to_string(-((dfDockDepth - dfNavDepth)* tan(MOOSDeg2Rad(dfOpticalNavLoc[2])) * sin(MOOSDeg2Rad(dfNavHeading)))) + "," + to_string(- (dfDockDepth - dfNavDepth) * tan(MOOSDeg2Rad(dfOpticalNavLoc[1])) * cos(MOOSDeg2Rad(dfNavHeading))) + "," + to_string(dfDeltaL * cos(MOOSDeg2Rad(dfNavHeading))) + "," + to_string(dfNextX) + "," + to_string(dfNextY) + ")";
    // string sTmp = "OPT_LOC=(" + to_string(dfNextX) + "," + to_string(dfNextY) + ")";if

    // if(dfNavHeading>180)dfNavHeading-=360;
    // dfNavHeading = angle180(dfNavHeading);
  
    distance = distToPoint(0, 0, dfNextX ,dfNextY);
    sTmp = "OPT_LOC=(" + to_string(dfNextX) + "," + to_string(dfNextY) + ","+to_string(distance)+",["+to_string(dfInnerRadius)+","+to_string(dfOuterRadius)+"],"+to_string((dfNavHeading)); //  sqrt(dfNextX*dfNextX+dfNextY*dfNextY)

    sTmp=sTmp+","+to_string(bDataFlag)+")";
    cout << "Optical WPT: " << sTmp << endl;

    // cout << "test1" << endl;
    // if (bRetryLastPhase) {usleep(m_nLastPhaseFloatTime);bRetryLastPhase = false;nRetryLastPhase++;}
    // if (nRetryLastPhase > 5)
    // {
    //   SetMOOSVar("DOCKINGFAILED", "true", MOOSTime());
    // }

    if( (nPhaseCount > 0) && (nPhaseCount < nPhaseNum) )
    {
          dfCurrentDepth = dfvAlignDepth[nPhaseCount-1][0];
          string sDephtTmp = "depth=" + to_string(dfCurrentDepth);
          cout << sDephtTmp << endl;
          SetMOOSVar("DOCKDEPTH_UPDATE", sDephtTmp, MOOSTime());
      if (bDataFlag && distance <= dfOuterRadius)
      {
        nCtnuInvalidDataCount = 0;
        if(bDataFlag && distance < dfInnerRadius && nPhaseCount < nPhaseNum-1)
        { 
          nPhaseCount++;
          nCtnuInvalidDataCount = 0;
          dfCurrentDepth = dfvAlignDepth[nPhaseCount-1][0];
          string sTmp = "depth=" + to_string(dfCurrentDepth);
          cout << sTmp << endl;
          SetMOOSVar("DOCKDEPTH_UPDATE", sTmp, MOOSTime());
          SetMOOSVar("DOCKHDG_UPDATES","pwt=1",MOOSTime());
          SetMOOSVar("STATIONING","true",MOOSTime());
          dfInnerRadius = dfvAlignDepth[nPhaseCount-1][1];
          dfOuterRadius = dfvAlignDepth[nPhaseCount-1][2]; // Otherwise go to next phase
          if (nPhaseCount==nPhaseNum-1)
          {
            SetMOOSVar("DOCKHDG_UPDATES","pwt=50",MOOSTime());
          }
        }
        else if((bDataFlag == 1) &&  (nPhaseCount==nPhaseNum-1))
        {
          if(distance < dfInnerRadius * 0.8 && distance>0.3)
          {
            // SetMOOSVar("DOCKHDG_UPDATES","pwt=50",MOOSTime());
            SetMOOSVar("STATIONING","false",MOOSTime());
            string sTmp = "pwt=50,heading=" + to_string(dfDockHeading); 
            SetMOOSVar("DOCKHDG_UPDATES", sTmp, MOOSTime());
          }
          else if (distance > dfInnerRadius * 1.2)
          {
            SetMOOSVar("STATIONING","true",MOOSTime());
            SetMOOSVar("DOCKHDG_UPDATES","pwt=1",MOOSTime());  
          }          
          if(((GetMOOSVar("DESIRED_SPEED")->GetDoubleVal())*7+distance)<dradius)
          {
            if(distance < previousdistance)
            {
              if(abs(dfNavHeading - dfDockHeading)<dinheading)
              {
                nPhaseCount = nPhaseNum;
                dfCurrentDepth = dfDockDepth+dfFallDepth;
                string sTmp = "depth=" + to_string(dfCurrentDepth);
                SetMOOSVar("DOCKDEPTH_UPDATE",sTmp,MOOSTime());
                sTmp = "pwt=200,heading=" + to_string(dfDockHeading);
                SetMOOSVar("DOCKHDG_UPDATES",sTmp,MOOSTime());
                SetMOOSVar("STATIONING","false",MOOSTime());
                SetMOOSVar("DOCKING_FALLING","true",MOOSTime());
              }
            }
          }
          previousdistance=distance;    
        }

      }

      else if((!bDataFlag)||(distance>dfOuterRadius)){
        nCtnuInvalidDataCount++;
        if (nCtnuInvalidDataCount > m_dfFreq*60*nMaxtryMinuteNum)
        {
          if (nPhaseCount > 1)
          {
            nPhaseCount--;
            nCtnuInvalidDataCount = 0;
            dfCurrentDepth = dfvAlignDepth[nPhaseCount-1][0];
            string sTmp = "depth=" + to_string(dfCurrentDepth);
            cout << sTmp << endl;
            SetMOOSVar("DOCKDEPTH_UPDATE", sTmp, MOOSTime());
            SetMOOSVar("DOCKHDG_UPDATES","pwt=1",MOOSTime());
            SetMOOSVar("STATIONING","true",MOOSTime());
            SetMOOSVar("DOCKING_FALLING","false",MOOSTime());
            dfInnerRadius = dfvAlignDepth[nPhaseCount-1][1];
            dfOuterRadius = dfvAlignDepth[nPhaseCount-1][2]; // Otherwise return to previos phase
          }
          else if (nPhaseCount == 1)
          {
            if (nDockingTryCount < m_nDockingMaxTry)
            {
              SetMOOSVar("OPTICALNAV", "CLOSETODOCKING", MOOSTime()); // Return to acoustic positioning
              SetMOOSVar("CONSTHEIGHT", "true", MOOSTime());
              bDockingPhase = false;
              nDockingTryCount++;
            }
            else
            {
              SetMOOSVar("DOCKINGFAILED", "true", MOOSTime());
            }
          }
        }
      }


    }
    // else if (nPhaseCount==nPhaseNum)
    // {
    //   if ()
    // }

        // if((bDataFlag == 1)&&(nPhaseCount==nPhaseNum-1))
        // {
        //   if(((GetMOOSVar("DESIRED_SPEED")->GetDoubleVal())*7+distance)<dradius)
        //   {
        //     if(distance < previousdistance)
        //     {
        //       if(abs(dfNavHeading)<dinheading)
        //       {
        //         nPhaseCount++;
        //         SetMOOSVar("STATIONING","false",MOOSTime());
        //         dfCurrentDepth = dfDockDepth+dfFallDepth;
        //         string sTmp = "depth=" + to_string(dfCurrentDepth);
        //         SetMOOSVar("DOCKDEPTH_UPDATE",sTmp,MOOSTime());
        //       }
        //     }
        //   }    
        //   if(distance<0.3)
        //   {
        //     SetMOOSVar("DOCKHDG_UPDATES","pwt=50",MOOSTime());
        //   }
        //   else
        //   {
        //     SetMOOSVar("DOCKHDG_UPDATES","pwt=1",MOOSTime());            
        //   }

    if(bDataFlag)previousdistance=distance;

    if (nPhaseCount == nPhaseNum)
    {
      // if ((abs(dfDockDepth - GetMOOSVar("OPTICALNAVDEPTH")->GetDoubleVal() - m_dfDepthBias - m_dfDepthGroundBias - m_dfDockPanel) > 0.3 || (abs(GetMOOSVar("NAVPITCH")->GetDoubleVal() - dfDockPitch) > dfAngleBias) || (abs(GetMOOSVar("NAVROLL")->GetDoubleVal() - dfIMURollAmountBias - dfDockRoll) > dfAngleBias) ))
      if ((abs(dfLightDepth - dfCameraDepth) > m_dfDistanceBias || (abs(GetMOOSVar("NAVPITCH")->GetDoubleVal() - dfIMUPitchAmountBias - dfDockPitch) > dfAngleBias) || (abs(GetMOOSVar("NAVROLL")->GetDoubleVal() - dfIMURollAmountBias - dfDockRoll) > dfAngleBias) ))
      {
        nConstantDepthCount++;
        if ((nConstantDepthCount > nConstantDepthMinuteNum * 60 * m_dfFreq) || (nRetryLastPhase >= 5))
        {
          // SetMOOSVar("OPTICALNAV", "DOCKFINISHED", MOOSTime());
          if (nRetryLastPhase >= 5 || abs(dfLightDepth - dfCameraDepth) > 1) {
            nPhaseCount--;
          
          nRetryLastPhase = 0; bRetryLastPhase = false;
          nCtnuInvalidDataCount = 0;
          dfCurrentDepth = dfvAlignDepth[nPhaseCount-1][0];
          string sTmp = "depth=" + to_string(dfCurrentDepth);
          cout << sTmp << endl;
          SetMOOSVar("DOCKDEPTH_UPDATE", sTmp, MOOSTime());
          SetMOOSVar("DOCKING_FALLING","false",MOOSTime());
          // SetMOOSVar("STATIONING","false",MOOSTime());
          // SetMOOSVar("DOCKHDG_UPDATES","pwt=50",MOOSTime());  
          dfInnerRadius = dfvAlignDepth[nPhaseCount-1][1];
          dfOuterRadius = dfvAlignDepth[nPhaseCount-1][2]; // Otherwise return to previos phase
          nConstantDepthCount = 0;
          }
          else 
          {
            if(nFloatIteration < m_nLastPhaseFloatTime * m_dfFreq/1000)
            {
              nFloatIteration++;
              SetMOOSVar("MOOS_MANUAL_OVERIDE", "true", MOOSTime());
            }
            else{
              // usleep(m_nLastPhaseFloatTime);
              nRetryLastPhase++;
              SetMOOSVar("MOOS_MANUAL_OVERIDE", "false", MOOSTime());
              SetMOOSVar("DOCKING_FALLING", "true", MOOSTime());
              nConstantDepthCount = 0;
              nCtnuInvalidDataCount = 0;
              nFloatIteration = 0;
            }
          }
        }
      }
      else
      {
        SetMOOSVar("DOCKING_FALLING", "false", MOOSTime());
        // SetMOOSVar("OPTICALNAV", "DOCKFINISHED", MOOSTime()); // Return to acoustic positioning
      }
    }

    
    SetMOOSVar("OPTICALNAVDHEADING", dfOpticalNavLoc[0], MOOSTime());
    SetMOOSVar("OPTICALNAVTHETAX", dfOpticalNavLoc[1], MOOSTime());
    SetMOOSVar("OPTICALNAVTHETAY", dfOpticalNavLoc[2], MOOSTime());
    SetMOOSVar("OPTICALNAVX", dfNextX, MOOSTime());
    SetMOOSVar("OPTICALNAVY", dfNextY, MOOSTime());
    if (sMode == "DOCKING")
    {
      SetMOOSVar("OPTICALSIMX", dfNextX, MOOSTime());
      SetMOOSVar("OPTICALSIMY", dfNextY, MOOSTime());
    }
    SetMOOSVar("OPTICALNAVWPT", sTmp, MOOSTime());
    sprintf(chTmp, "%.2f, %.2f, %.2f", dfOpticalNavLoc[0], dfOpticalNavLoc[1], dfOpticalNavLoc[2]);
    sTmp = chTmp;
    SetMOOSVar("OPTICALNAVDATA", sTmp, MOOSTime());
    double dfHeading = dfOpticalNavLoc[0] + dfNavHeading;
    string sHeading = "HDG_VALUE=heading=" + to_string(dfHeading);
    // SetMOOSVar("OPTICALNAVHEADING", sHeading, MOOSTime());

    sTmp = to_string(m_iterations) + "|" + to_string(nPhaseCount)+"/"+to_string(nPhaseNum) +":depth-"+to_string(dfCurrentDepth);
    sprintf(chTmp, "%d,%d-%d|%d/%d,invalid count:%d/%d(%d)(Offset-depth:%.2f,roll:%.2f,pitch:%.2f),depth:%.2f,distance:%.4f,[%.4f,%.4f],heading:%2f,%.2f", m_iterations, bDataFlag, nCntuDockingCount, nPhaseCount, nPhaseNum, nCtnuInvalidDataCount, nConstantDepthCount, nRetryLastPhase, (dfLightDepth - dfCameraDepth), abs(GetMOOSVar("NAVROLL")->GetDoubleVal() - dfIMURollAmountBias - dfDockRoll), abs(GetMOOSVar("NAVPITCH")->GetDoubleVal() - dfIMUPitchAmountBias - dfDockPitch), dfCurrentDepth, distance, dfInnerRadius, dfOuterRadius, dfNavHeading, abs(dfNavHeading - dfDockHeading));
    cout << "OPTICALNAVSUMMARY: " << chTmp << endl;
    SetMOOSVar("OPTICALNAVSUMMARY", chTmp, MOOSTime());


    //Publish MOOS variables to MOOSDB
    PublishFreshMOOSVariables();
    sprintf(chTmp, "S:%f %f %f %f %f %f E\r\n", (dfLightDepth - dfCameraDepth) > m_dfDistanceBias ? (dfLightDepth - dfCameraDepth) : m_dfDistanceBias, dfNavHeading, dfNextX, dfNextY, dfCurrentIMUX-dfLastIMUX, dfCurrentIMUY-dfLastIMUY);
    ensureConnect(socketOPTServer);
    socketOPTServer->iSendMessage((void *)chTmp, strlen(chTmp));//.length());
    SetMOOSVar("OPTICALSENDINFO", chTmp, MOOSTime());
    cout << "Send message: " << sTmp << endl;
    dfLastIMUX = dfCurrentIMUX;
    dfLastIMUY = dfCurrentIMUY;

  }
  else 
  {
    ensureConnect(socketOPTServer);
    sprintf(chTmp, "S:%f %f %f %f %f %f E\r\n", 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    socketOPTServer->iSendMessage((void *)chTmp, strlen(chTmp));//.length());
  }


  #ifdef DEBUG
  AppCastingMOOSApp::PostReport();
  #endif
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void NavServer::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool NavServer::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "Optical Navigation Report:                  " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(6);
  actab << "Iteration | Phase | OpticalNav dHeading | OpticalNav ThetaX | OpticalNav ThetaY | (DeltaX, DeltaY)";
  // actab << "Iteration | OpticalNav dHeading | OpticalNav ThetaX | OpticalNav ThetaY | DeltaX | DeltaY";
  actab.addHeaderLines();
  actab << m_iterations << to_string(nPhaseCount)+"/"+to_string(nPhaseNum) << dfOpticalNavLoc[0] << dfOpticalNavLoc[1] << dfOpticalNavLoc[2] << "(" + to_string(dfDX) + "," + to_string(dfDY) + ")"; // "(" + to_string(dfDX) + "," + to_string(dfDY) + ")"
  m_msgs << actab.getFormattedString();

  return(true);
}

bool NavServer::tryConnect()
{
  // socketClient->iGetKeepAlive();
  // bool bBind = false;
  // bool bListen = false;
  // int nRetryCount = 0;
  // while (!bBind || !bListen)
  // {
  //   try
  //   {
  //     socketClient->vBindSocket();
  //     bBind = true;
  //     socketClient->vListen(1);
  //     bListen = true;
  //   }
  //   catch(XPCException &socketExcept)
  //   {
  //     string sTmp = socketExcept.sGetException();
  //     cout << socketExcept.sGetException() << " [" << nRetryCount++ << "] ," << bBind << "," << bListen << "," << strcmp(sTmp.c_str(), "Error binding") << "," << strcmp(sTmp.c_str(), "Error Listening")<< endl;
  //     if (strcmp(sTmp.c_str(), "Error binding") > 0)
  //     bBind = false;
  //     else if(strcmp(sTmp.c_str(), "Error Listening") > 0)
  //     bListen = false;
  //     MOOSPause(2000);
  //   }
  // }

  bool bConnected = false;
  int nConnectTryCount = 0;
  while (!bConnected)
  {
    try {
      socketOPTServer = socketClient->Accept((char *)sIPaddr.c_str());
      bConnected = true;
      // socketClient->vSetRecieveTimeOut(0.3); //MSG_NOSIGNAL
      // socketClient->vSetDebug(0);
      // socketClient->vSetKeepAlive(false);
      // socketClient->vSetSocketBlocking(true);
      MOOSTrace("socket ID: %d, blocking: %d, keepalive: %d, SendBuf: %d, RecBuf: %d\n", socketOPTServer->iGetSocketFd(), socketOPTServer->iGetSocketBlocking(), socketOPTServer->iGetKeepAlive(), socketOPTServer->iGetSendBuf(), socketOPTServer->iGetRecieveBuf());
    }
    catch (XPCException &socketExcept){
      MOOSTrace("Socket %s [%d] connection failed [%d] --> %s:%d\n", GetAppName().c_str(), socketClient->iGetSocketFd(), nConnectTryCount++, sIPaddr.c_str(), SOCK_PORT);
      MOOSPause(1000);
    }
  }

  return(bConnected);
}

int NavServer::isSocketConnected(XPCTcpSocket *_socket) 
{ 
  int sock = _socket->iGetSocketFd();
  if(sock<=0) 
  return 0; 
  struct tcp_info info; 
  int len=sizeof(info); 
  getsockopt(sock, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len); 
  if((info.tcpi_state==TCP_ESTABLISHED)) 
  { 
    #ifdef DEBUG
    printf("socket connected\n"); 
    #endif
    return 1; 
  } 
  else 
  { 
    #ifdef DEBUG
    printf("socket disConnected\n"); 
    #endif
    return 0; 
  } 
}

void NavServer::ensureConnect(XPCTcpSocket *_socket)
{
  if (!isSocketConnected(_socket))
  {
    _socket->vCloseSocket();
    _socket = new XPCTcpSocket((long)SOCK_PORT);
    tryConnect();
  }
  return;
}

// bool NavServer::ReceiveData(XPCTcpSocket *sock_fd, double * dfOpticalNavVal) //Only can be seen in the file
bool NavServer::ReceiveData() //Only can be seen in the file
{
  // XPCTcpSocket* fd = sock_fd;
  int i_recvBytes;
  double vx,vy,vz,navx,navy;
  int vflag;
  char data_recv[BUFFER_LENGTH];
  while(1)
  {
      //Reset data.
      memset(data_recv,0,BUFFER_LENGTH);

      // i_recvBytes = read(fd,data_recv,BUFFER_LENGTH);
      ensureConnect(socketOPTServer);
      i_recvBytes = socketOPTServer->iReadMessageWithTimeOut((void *)&(data_recv), BUFFER_LENGTH, 0.1);
      if(i_recvBytes == 0)
      {
          printf("Maybe the client has closed\n");
          return(false);
          // break;
          continue;
      }
      else if(i_recvBytes == -1)
      {
          fprintf(stderr,"read error!\n");
          return(false);
          // break;
          continue;
      }
      // if(strcmp(data_recv,"quit")==0)
      // {
      //     printf("Quit command!\n");
      //     break;                           //Break the while loop.
      //     continue;
      // }
      // printf("read from client : %s\n",data_recv);
      // else
      {
      data_recv[i_recvBytes]='\0';
      sscanf(data_recv,"S:%lf %lf %lf %d %lf %lf E\r\n",&vx,&vy,&vz,&vflag,&navx,&navy);
      // sscanf(data_recv,"S:%lf %lf %lf E\r\n",&dfOpticalNavLoc[0],&dfOpticalNavLoc[1],&dfOpticalNavLoc[2]);
      // printf("dHeading=%lf,thetax=%lf,thetay=%lf\n",vx,vy,vz);
      // double dfDX = (dfDockDepth - dfNavDepth) * tan(MOOSDeg2Rad(vy));
      // double dfDY = (dfDockDepth - dfNavDepth) * tan(MOOSDeg2Rad(vz));
      string sTmp = data_recv;
      MOOSTrimWhiteSpace(sTmp);
      // cout << "Optical WPT: " << sTmp << endl;
      SetMOOSVar("OPTICALRAW", sTmp, MOOSTime());
      // PublishFreshMOOSVariables();
      // printf("vx=%lf,vy=%lf,vz=%lf\n",vx,vy,vz);
      bDataFlag = vflag;
      dfOpticalNavLoc[0] = vx;
      if(vflag)
      {
        dfOpticalNavLoc[1] = vy;
        dfOpticalNavLoc[2] = vz;
      }
      else
      {
        dfOpticalNavLoc[1] = navx;
        dfOpticalNavLoc[2] = navy;
      }
      // printf("Function: dHeading=%lf,thetax=%lf,thetay=%lf\n",((thr_Param *)thrParam)->dfOpticalNavVal[0],((thr_Param *)thrParam)->dfOpticalNavVal[1],((thr_Param *)thrParam)->dfOpticalNavVal[2]);
      // printf("Function: dHeading=%lf,thetax=%lf,thetay=%lf,flag=%d\n",vx,vy,vz,vflag);
      printf("Function: dHeading=%lf,thetax=%lf,thetay=%lf,flag=%d\n",vx,dfOpticalNavLoc[1],dfOpticalNavLoc[2],vflag);
      break;
      }

  }

  //Clear
  return(true);

}

// static void Data_handle(void * sock_fd)
static void Data_handle(void * thrParam)
{
  // int fd = *((int *)sock_fd);
  // int fd = ((thr_Param *)thrParam)->sock_fd;
  XPCTcpSocket* fd = ((thr_Param *)thrParam)->sock_fd;
  int i_recvBytes;
  double vx,vy,vz;
  char data_recv[BUFFER_LENGTH];
  while(1)
  {
      //Reset data.
      memset(data_recv,0,BUFFER_LENGTH);

      // i_recvBytes = read(fd,data_recv,BUFFER_LENGTH);
      i_recvBytes = fd->iReadMessageWithTimeOut((void *)&(data_recv), BUFFER_LENGTH, 0.1);
      if(i_recvBytes == 0)
      {
          printf("Maybe the client has closed\n");
          // break;
          continue;
      }
      if(i_recvBytes == -1)
      {
          fprintf(stderr,"read error!\n");
          // break;
          continue;
      }
      // if(strcmp(data_recv,"quit")==0)
      // {
      //     printf("Quit command!\n");
      //     break;                           //Break the while loop.
      //     continue;
      // }
      // printf("read from client : %s\n",data_recv);
      data_recv[i_recvBytes]='\0';
      sscanf(data_recv,"S:%lf %lf %lf E\r\n",&vx,&vy,&vz);
      // sscanf(data_recv,"S:%lf %lf %lf E\r\n",&dfOpticalNavLoc[0],&dfOpticalNavLoc[1],&dfOpticalNavLoc[2]);
      // printf("dHeading=%lf,thetax=%lf,thetay=%lf\n",vx,vy,vz);
      // double dfDX = (dfDockDepth - dfNavDepth) * tan(MOOSDeg2Rad(vy));
      // double dfDY = (dfDockDepth - dfNavDepth) * tan(MOOSDeg2Rad(vz));
      // sTmp = "WPT_UPDATE=\"x=" + to_string(dfDX) + "#y=" + to_string(dfDY) + "\"";
      // cout << "Optical WPT: " << sTmp << endl;
      // SetMOOSVar("OPTICALNAVWPT", sTmp, MOOSTime());
      // PublishFreshMOOSVariables();
      // printf("vx=%lf,vy=%lf,vz=%lf\n",vx,vy,vz);
      ((thr_Param *)thrParam)->dfOpticalNavVal[0] = vx;
      ((thr_Param *)thrParam)->dfOpticalNavVal[1] = vy;
      ((thr_Param *)thrParam)->dfOpticalNavVal[2] = vz;
      // printf("Function: dHeading=%lf,thetax=%lf,thetay=%lf\n",((thr_Param *)thrParam)->dfOpticalNavVal[0],((thr_Param *)thrParam)->dfOpticalNavVal[1],((thr_Param *)thrParam)->dfOpticalNavVal[2]);
      break;

  }

  //Clear
  // printf("terminating current client_connection...\n");
  // close(fd);            //close a file descriptor.
  fd->vCloseSocket();
  pthread_exit(NULL);   //terminate calling thread!
}

