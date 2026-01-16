/************************************************************/
/*    NAME: Xinyu An                                        */
/*    ORGN: Zhejiang University                             */
/*    FILE: OpticalNavServer.h                                          */
/*    DATE: 2021-11-28                                      */
/************************************************************/

#ifndef OpticalNavServer_HEADER
#define OpticalNavServer_HEADER

#ifndef UNIX
#define UNIX
#endif

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOS/Comms/XPCTcpSocket.h"

#define BUFFER_LENGTH 1024

class OpticalNavServer : public AppCastingMOOSApp
{
 public:
   OpticalNavServer();
   ~OpticalNavServer();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   bool tryConnect();
   int  isSocketConnected(int sock);
   int  tryWrite();
   int  tryRead();
   void ensureConnect();

   void RegisterVariables();
   bool startOpticalNav();
   bool stopOpticalNav();
   static void Data_handle(void * sock_fd);

 private: // Configuration variables
   unsigned int m_iterations;
   double       m_timewarp;
   XPCTcpSocket *socketServer;
   XPCTcpSocket *socketClientOpticalNav = NULL;

 private: // State variables
   std::string sIPaddr;
   int  SOCK_PORT = 5000;
   bool bOpticalNavPowerStatus = false;
   char m_ReceiveBuf[BUFFER_LENGTH];
   double dfOpticalNavLoc[3];
};

#endif
