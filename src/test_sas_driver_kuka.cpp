int main(void)
{
    return 0;
}
/**

#include <QtCore/QList>
#include <QtCore/QByteArray>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtNetwork/QUdpSocket>
static QApplication* qt_application{nullptr};
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
**/
