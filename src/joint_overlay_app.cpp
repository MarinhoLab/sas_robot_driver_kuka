//Heavily modified from the LBRJointSineOverlayApp by Kuka.
//Author: Murilo M. Marinho
//Email: murilo.marinho@manchester.ac.uk
/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.Connectivity FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.Connectivity FastRobotInterface” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2018 
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.16}
**/
#include <cstdlib>
#include <cstdio>
#include <cstring> // strstr
#include <csignal>
#include <atomic>
#include <iostream>
#include <memory>
#include <ostream>
#include "joint_overlay_client.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <thread>

constexpr int DEFAULT_PORTID = 30200; //Original Kuka code had a define
using namespace KUKA::FRI;

/**
 * @brief communication_thread_loop. This communication strategy for sas_robot_driver
 * derivatives is unconventional for the library itself but quite common for robotics
 * in general. The loop needs to be managed by a separate thread that idealy is completely
 * transparent to other programs.
 * @param break_loops
 * @return
 */
int communication_thread_loop(std::shared_ptr<LBRJointCommandOverlayClient> trafo_client,
                              std::atomic_bool* break_loops,
                              std::atomic_bool* connection_established)
{
    //pthread_attr_t attr;
    //int ret;
    //ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    //if (ret) {
    //    printf("pthread setschedpolicy failed\n");
    //}
   //SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

   char* hostname = NULL;
   int port = DEFAULT_PORTID;

   //LBRJointCommandOverlayClient trafo_client;
   //trafoClient->set_joint_value_update_callback(update_joint_value_list);
   //trafoClient->set_target_joint_value_update_callback(update_target_joint_value_list);

   //Configuration
   UdpConnection connection(1000);
   ClientApplication app(connection, *trafo_client.get());
   if (!app.connect(port, hostname))
   {
       std::cout << "Unable to connect to KUKA Sunrise controller" << std::endl;
       break_loops->store(true);
       connection_established->store(false);
   }
   connection_established->store(true);

   // repeatedly call the step routine to receive and process FRI packets
   bool success = true;
   while (success && !(*break_loops))
   {
      success = app.step();
      
      // check if we are in IDLE because the FRI session was closed
      if (trafo_client->robotState().getSessionState() == IDLE)
      {
         // In this demo application we simply quit.
         // Waiting for a new FRI session would be another possibility.
         break;
      }
   }
   break_loops->store(true);
   //Kill everything else in case the loop is broken for another reason
   //raise(SIGINT);

   // disconnect from controller
   app.disconnect();
   
   return 1;
}
