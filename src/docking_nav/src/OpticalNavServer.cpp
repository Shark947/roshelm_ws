/************************************************************/
/*    NAME: Xinyu An                                        */
/*    ORGN: Zhejiang University                             */
/*    FILE: OpticalNavServer.cpp                                        */
/*    DATE: 2021-11-28                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ColorParse.h"
#include "OpticalNavServer.h"

using namespace std;

//---------------------------------------------------------
// Constructor

OpticalNavServer::OpticalNavServer()
{
  m_iterations = 0;
  m_timewarp = 1;

  // FILE *fp = NULL;
  // if ((fp = popen("canSend 0x7e9 71010000", "r")) == NULL)
  // {
  //   cout << termColor("red") << "Open OPTICAL NAVIGATION power failed" << termColor() << endl;
  // }
  // pclose(fp);

}

//---------------------------------------------------------
// Destructor

OpticalNavServer::~OpticalNavServer()
{
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool OpticalNavServer::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool OpticalNavServer::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  m_timewarp = GetMOOSTimeWarp();

  double dOpticalNavServerPeriod = 1.0;
  bool   bbar = false;
  if (!m_MissionReader.GetValue("OpticalNavServerPeriod", dOpticalNavServerPeriod))
  {
    MOOSTrace("OpticalNavServer communication period not set!!!");
    return false;
  }

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
      if(param == tolower("OpticalNavServerPeriod")) {
        dOpticalNavServerPeriod = atof(value.c_str());
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

      else if(param == tolower("bar")) {
        setBooleanOnString(bbar, value);
        handled = true;
      }

      if(!handled)
        reportUnhandledConfigWarning(orig);
    }
  }
  else
    reportConfigWarning("No config block found for " + GetAppName());

  // Add MOOS dynamic/run variables: AddMOOSVariable(sName, sSubscribeName, sPublishName, dfCommsTime)
  AddMOOSVariable("OPTICALNAVSTART", "MODE", "", dOpticalNavServerPeriod);
  AddMOOSVariable("OPTICALNAV", "MODE", "", dOpticalNavServerPeriod);
  AddMOOSVariable("OPTICALNAVX", "", "SIM_X", dOpticalNavServerPeriod);
  AddMOOSVariable("OPTICALNAVY", "", "SIM_Y", dOpticalNavServerPeriod);
  AddMOOSVariable("OPTICALNAVZ", "", "SIM_Z", dOpticalNavServerPeriod);

  // Register MOOS variables to MOOSDB
  RegisterMOOSVariables();

  // RegisterVariables();
  cout << "IP: " << sIPaddr.c_str() << ", PORT: " << SOCK_PORT << endl;
  short isocketfd;
  // socketServer = new XPCTcpSocket((short)isocketfd);
  socketServer = new XPCTcpSocket((long)SOCK_PORT);
  // socketServer->vConnect((const char*)(sIPaddr.c_str()));
  // socketServer->vSetReuseAddr(1);
  socketServer->vBindSocket();
  cout << "vBindSocket" << endl;
  // socketServer->vListen(int _iNumPorts=5);
  socketServer->vListen();
  cout << "vListen" << endl;
  // socketClientOpticalNav = socketServer->Accept((char*)(sIPaddr.c_str()));
  // cout << "Accept" << endl;


  return(true);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool OpticalNavServer::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

  #if 0 // Keep these around just for template
      string key   = msg.GetKey();
      string comm  = msg.GetCommunity();
      double dval  = msg.GetDouble();
      string sval  = msg.GetString();
      string msrc  = msg.GetSource();
      double mtime = msg.GetTime();
      bool   mdbl  = msg.IsDouble();
      bool   mstr  = msg.IsString();
  #endif

  if(key == "FOO")
    cout << "great!";

  else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
    reportRunWarning("Unhandled Mail: " + key);
   }

  //Check MOOSDB for MOOS variables
  if( UpdateMOOSVariables(NewMail) )
  {
    //Get the value of MOOS variable: CMOOSVariable & MOOSVar = *GetMOOSVar(sName)
    // double dTemp = GetMOOSVar("")->GetDoubleVal();
    // string sTemp = GetMOOSVar("")->GetStringVal();
    string sMode = GetMOOSVar("DETECTSTART")->GetStringVal();
    // if (sMode == "SURVEY")
    if (sMode == "SURVEY")
    {
      startOpticalNav();
    }
    else if (sMode == "RETURN")
    {
      stopOpticalNav();
    }

  }

   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool OpticalNavServer::Iterate()
{
  AppCastingMOOSApp::Iterate();

  m_iterations++;
  //Set value/string to MOOS variables: SetMOOSVar(sName, dfVal/sVal, dfTime)
  // SetMOOSVar("", , MOOSTime());
  cout << "m_iterations: " << m_iterations << endl;
  // if (m_iterations == 1)
  // {
  //   startOpticalNav();
  //   bOpticalNavPowerStatus = true;
  //   cout << "Open detect sensor" << endl;
  // }

  // else if ((m_iterations%10) == 0)
  // {
  //   if (!bOpticalNavPowerStatus)
  //   {
  //     startOpticalNav();
  //     bOpticalNavPowerStatus = true;
  //     cout << "Open detect sensor" << endl;
  //   }
  //   else
  //   {
  //     stopOpticalNav();
  //     bOpticalNavPowerStatus = false;
  //     cout << "Close detect sensor" << endl;
  //   }
  // }

  #ifdef DEBUG
  	cout << "Debug information." << endl;
  #else
  	cout << "Release information." << endl;
  #endif

  // int index = socketClientOpticalNav->iReadMessageWithTimeOut((void *)&(m_ReceiveBuf), BUFFER_LENGTH, 0.1);
  // socketServer->vListen();
  // cout << "vListen" << endl;
  // char IP[] = "192.168.152.128";
  // socketClientOpticalNav = socketServer->Accept((char *)IP);
  if (socketClientOpticalNav == NULL)
  {
    cout << "Waiting for new connection" << endl;
    socketClientOpticalNav = socketServer->Accept((char*)(sIPaddr.c_str()));
    cout << "Accept" << endl;
  }
  tryConnect();
  memset(m_ReceiveBuf,0,BUFFER_LENGTH);
  int index = socketClientOpticalNav->iRecieveMessage((void *)&(m_ReceiveBuf), BUFFER_LENGTH, 0);
  pthread_t thread_id;
  // if(pthread_create(&thread_id,NULL,(void* (*)(void*))(&(socketClientOpticalNav->iRecieveMessage((void *)&(m_ReceiveBuf), BUFFER_LENGTH, 0))),(&socketClientOpticalNav)) == -1)
  // if(pthread_create(&thread_id,NULL,(void* (*)(void*))(&Data_handle),(&socketClientOpticalNav)) == -1)
  {
    // printf("pthread_create error!\n");
  }
  string sRecMessage(m_ReceiveBuf);
  cout << "Message length: " << sRecMessage.length() << " | " << index << endl;
  if (sRecMessage.length() > 0)
  {
    cout << "Received message, [" << sRecMessage.length() << "]: ";
    for (int i = 0; i < sRecMessage.length(); i++)
    {
      printf("%c", m_ReceiveBuf[i]);
    }
  }
  cout << endl;
  double vx,vy,vz;
  sscanf(m_ReceiveBuf,"S:%lf %lf %lf E\r\n",&vx,&vy,&vz);
  sscanf(m_ReceiveBuf,"S:%lf %lf %lf E\r\n",&dfOpticalNavLoc[0],&dfOpticalNavLoc[1],&(dfOpticalNavLoc[2]));
  printf("vx=%lf,vy=%lf,vz=%lf\n",vx,vy,vz);
  printf("dHeading=%lf,theta_x=%lf,theta_y=%lf\n",dfOpticalNavLoc[0],dfOpticalNavLoc[1],dfOpticalNavLoc[2]);
  // close(*((int *)socketClientOpticalNav));            //close a file descriptor.

  // if (tolower(sMode) == tolower("OPTICALNAV"))
  {
    SetMOOSVar("OPTICALNAVX", dfOpticalNavLoc[0], MOOSTime());
    SetMOOSVar("OPTICALNAVY", dfOpticalNavLoc[1], MOOSTime());
    SetMOOSVar("OPTICALNAVZ", dfOpticalNavLoc[2], MOOSTime());


    //Publish MOOS variables to MOOSDB
    PublishFreshMOOSVariables();
  }

  // delete socketClientOpticalNav;

  //Publish MOOS variables to MOOSDB
  PublishFreshMOOSVariables();

  #ifdef DEBUG
  AppCastingMOOSApp::PostReport();
  #endif
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void OpticalNavServer::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool OpticalNavServer::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "Optical Navigation Report:                  " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(5);
  actab << "Iteration | OpticalNavPowerStatus | OpticalNav X | OpticalNav Y | OpticalNav Z";
  actab.addHeaderLines();
  actab << m_iterations << bOpticalNavPowerStatus << dfOpticalNavLoc[0] << dfOpticalNavLoc[1] << dfOpticalNavLoc[2];
  m_msgs << actab.getFormattedString();

  return(true);
}

bool OpticalNavServer::startOpticalNav()
{
  cout << "start" << endl;
  char send_data[2] = {'1', '\0'};
  int index = socketClientOpticalNav->iSendMessage((void *)(send_data), 1);
  return(true);
}

bool OpticalNavServer::stopOpticalNav()
{
  cout << "stop" << endl;
  char send_data[2] = {'2', '\0'};
  int index = socketClientOpticalNav->iSendMessage((void*)(send_data), 1);

  return(true);

}

// static void OpticalNavServer::Data_handle(void * sock_fd)
// {
//   // int fd = *((int *)sock_fd);
//   int i_recvBytes;
//   double vx,vy,vz;
//   // char data_recv[BUFFER_LENGTH];
//   while(1)
//   {
//       //Reset data.
//       memset(m_ReceiveBuf,0,BUFFER_LENGTH);

//       i_recvBytes = socketClientOpticalNav->iRecieveMessage((void *)&(m_ReceiveBuf), BUFFER_LENGTH, 0);
//       if(i_recvBytes == 0)
//       {
//           printf("Maybe the client has closed\n");
//           break;
//       }
//       if(i_recvBytes == -1)
//       {
//           fprintf(stderr,"read error!\n");
//           break;
//       }
//       if(strcmp(m_ReceiveBuf,"quit")==0)
//       {
//           printf("Quit command!\n");
//           break;                           //Break the while loop.
//       }
//       printf("read from client : %s\n",m_ReceiveBuf);
//       m_ReceiveBuf[i_recvBytes]='\0';
//       sscanf(m_ReceiveBuf,"S:%lf %lf %lf E\r\n",&vx,&vy,&vz);
//       printf("vx=%lf,vy=%lf,vz=%lf\n",vx,vy,vz);
//   }

//   //Clear
//   printf("terminating current client_connection...\n");
//   close(*((int *)socketClientOpticalNav));            //close a file descriptor.
//   pthread_exit(NULL);   //terminate calling thread!
// }

bool OpticalNavServer::tryConnect()
{
  bool bConnected = false;
  int nConnectTryCount = 0;
  while (!bConnected)
  {
    try {
      socketClientOpticalNav = socketServer->Accept((char*)(sIPaddr.c_str()));
      bConnected = true;
      // socketClient->vSetRecieveTimeOut(0.3); //MSG_NOSIGNAL
      // socketClient->vSetDebug(0);
      // socketClient->vSetKeepAlive(false);
      // socketClient->vSetSocketBlocking(true);
      MOOSTrace("socket ID: %d, blocking: %d, keepalive: %d, SendBuf: %d, RecBuf: %d\n", socketClientOpticalNav->iGetSocketFd(), socketClientOpticalNav->iGetSocketBlocking(), socketClientOpticalNav->iGetKeepAlive(), socketClientOpticalNav->iGetSendBuf(), socketClientOpticalNav->iGetRecieveBuf());
    }
    catch (XPCException &socketExcept){
      MOOSTrace("Socket %d connection failed [%d]\n", socketClientOpticalNav->iGetSocketFd(), nConnectTryCount++);
      MOOSPause(1000);
    }
  }

  return(bConnected);
}

int OpticalNavServer::isSocketConnected(int sock) 
{ 
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

void OpticalNavServer::ensureConnect()
{
  if (!isSocketConnected(socketClientOpticalNav->iGetSocketFd()))
  {
    socketClientOpticalNav->vCloseSocket();
    socketClientOpticalNav = new XPCTcpSocket((long)SOCK_PORT);
    tryConnect();
  }
  return;
}