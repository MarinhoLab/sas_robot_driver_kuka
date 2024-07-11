//Modified from the LBRJointSineOverlayApp by Kuka.
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
#include <memory>
#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>
#include "joint_overlay_client.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include <QtCore/QList>
#include <QtCore/QByteArray>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtNetwork/QUdpSocket>

#define DEFAULT_PORTID 30200 //Original Kuka code had a define
using namespace KUKA::FRI;

int main_fri(int argc, char** argv); //Based on the original main which is now split in another thread.

/// Threading, application, and signal handling
static std::atomic_bool kill_application(false);
static QApplication* qt_application{nullptr};
static std::unique_ptr<LBRJointCommandOverlayClient> trafoClient;

void signal_handler(int signal)
{
	std::cout << "Signal received" << std::endl;
	if (signal == SIGINT)
	{
		kill_application = true;
		std::cout << "Kill signal received" << std::endl;
		if (qt_application != nullptr)
			qt_application->quit();
	}
}

// Qt Static Elements (TODO make a separate class)
static QList<QLabel*> joint_value_list;
void update_joint_value_list(const std::vector<double>& q)
{
    for (size_t i = 0; i < q.size(); i++)
		joint_value_list.at(i)->setText(QString::number(q.at(i)));
}

static QList<QLabel*> target_joint_value_list;
void update_target_joint_value_list(const std::vector<double>& q_target)
{
    for (size_t i = 0; i < q_target.size(); i++)
	{
		target_joint_value_list.at(i)->setText(QString::number(q_target.at(i)));
		//target_joint_value_list.at(i)->setText("<b>"
		//	+ QString::number(q_target.at(i))
		//	+ QString("</b>")
		//);
	}
}

static QList<QLabel*> joint_name_list;
static QList<QPushButton*> button_plus_list;
static QList<QPushButton*> button_minus_list;

int main(int argc, char** argv)
{
	signal(SIGINT, signal_handler);
	try
	{
		qt_application = new QApplication(argc,argv);
		qt_application->setApplicationName("MMM_LBRJointCommandOverlay [build " 
				+ QString("%1 %2").arg(__DATE__).arg(__TIME__)
				+ "]");

		std::thread fri_thread(main_fri, argc, argv);

		//We need the trafoclient to exist before doing anything else
		while (!trafoClient)
			std::this_thread::sleep_for(std::chrono::seconds(1));

		//GUI
		QFrame frame;
		QVBoxLayout main_layout;
		for (int i = 0; i < 7; i++)
		{
			QHBoxLayout* this_line_layout = new QHBoxLayout();

			joint_value_list.append(new QLabel("*.**", &frame));
			joint_value_list.at(i)->setMinimumSize(QSize(200, 20));
			target_joint_value_list.append(new QLabel("*.**", &frame));
			target_joint_value_list.at(i)->setMinimumSize(QSize(200, 20));
			joint_name_list.append(new QLabel("q" + QString::number(i), &frame));
			joint_name_list.at(i)->setMaximumSize(QSize(40, 40));
			button_plus_list.append(new QPushButton("+", &frame));
			button_plus_list.at(i)->setMaximumSize(QSize(40, 40));
			button_minus_list.append(new QPushButton("-", &frame));
			button_minus_list.at(i)->setMaximumSize(QSize(40, 40));

			this_line_layout->addWidget(joint_name_list.at(i));
			this_line_layout->addWidget(joint_value_list.at(i));
			this_line_layout->addWidget(target_joint_value_list.at(i));
            QObject::connect(button_plus_list.at(i), &QPushButton::pressed, &frame, [i]()
				{
					auto current_joint_value = trafoClient->get_measured_joint_value(i);
					trafoClient->set_target_joint_value(current_joint_value + 0.01, i);
				});
			this_line_layout->addWidget(button_plus_list.at(i));
            QObject::connect(button_minus_list.at(i), &QPushButton::pressed, &frame, [i]()
				{
					auto current_joint_value = trafoClient->get_measured_joint_value(i);
					trafoClient->set_target_joint_value(current_joint_value - 0.01, i);
				});
			this_line_layout->addWidget(button_minus_list.at(i));

			main_layout.addLayout(this_line_layout);
		}
		frame.setLayout(&main_layout);
		frame.setVisible(true);

        //TODO read ROS commands and turn them into commands like so
        //trafoClient->set_target_joint_values(external_command_values);

		qt_application->exec();

		kill_application = true;
		if (fri_thread.joinable())
			fri_thread.join();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Unhandled exception " << e.what() << "." << std::endl;
	}
	return 0;
}

int main_fri (int argc, char** argv)
{
   //SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

   // parse command line arguments
   if (argc > 1)
   {
	   if ( strstr (argv[1],"help") != NULL)
	   {
	      printf(
	            "\nKUKA LBR joint command overlay test application\n\n"
	            "\tCommand line arguments:\n"
	            "\t1) remote hostname (optional)\n"
	            "\t2) port ID (optional)\n"     
	      );
	      return 1;
	   }
   }
   char* hostname = (argc >= 2) ? argv[1] : NULL;
   int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORTID;

   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /**************************************************************************/
   
   // create new sine overlay client
   trafoClient = std::make_unique<LBRJointCommandOverlayClient>();
   trafoClient->set_joint_value_update_callback(update_joint_value_list);
   trafoClient->set_target_joint_value_update_callback(update_target_joint_value_list);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Configuration                                                         */
   /*                                                                         */
   /***************************************************************************/

   // create new udp connection
   UdpConnection connection(1000);

   // pass connection and client to a new FRI client application
   ClientApplication app(connection, *trafoClient.get());
   
   // connect client application to KUKA Sunrise controller
   if (!app.connect(port, hostname))
	   throw std::runtime_error("Unable to connect to KUKA Sunrise controller");

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Execution mainloop                                                    */
   /*                                                                         */
   /***************************************************************************/

   // repeatedly call the step routine to receive and process FRI packets
   bool success = true;
   while (success && !kill_application)
   {
      success = app.step();
      
      // check if we are in IDLE because the FRI session was closed
      if (trafoClient->robotState().getSessionState() == IDLE)
      {
         // In this demo application we simply quit.
         // Waiting for a new FRI session would be another possibility.
         break;
      }
   }
   //Kill everything else in case the loop is broken for another reason
   raise(SIGINT);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/

   // disconnect from controller
   app.disconnect();
   
   return 1;
}
