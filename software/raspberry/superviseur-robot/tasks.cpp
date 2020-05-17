/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 20 // regular
#define PRIORITY_TOPENCOMROBOT 20 // regular
#define PRIORITY_TMOVE 25 // high prior
#define PRIORITY_TSENDTOMON 22 // low prior
#define PRIORITY_TRECEIVEFROMMON 25 // high prior
#define PRIORITY_TSTARTROBOT 20 // regular
#define PRIORITY_TCAMERA 21 // to see
#define PRIORITY_TBATTERYLEVEL 20 // regular | low prior
#define PRIORITY_TREFRESHWD 99 // highest prior
#define PRIORITY_TRESET 31 // very high prior


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_watchDog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_error_count, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
   /* if (err = rt_mutex_create(&mutex_reset, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_refreshWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_otherComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reset, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, T_JOINABLE)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, T_JOINABLE)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, T_JOINABLE)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    //Our tasks
    if (err = rt_task_create(&th_refreshWD, ",th_refreshWD", 0, PRIORITY_TREFRESHWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_batteryLevel, "th_batteryLevel", 0, PRIORITY_TBATTERYLEVEL, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_reset, "th_reset", 0, PRIORITY_TRESET, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_comCamera, "th_comCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_actionCamera, "th_actionCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    //Our Tasks
    if (err = rt_task_start(&th_refreshWD, (void(*)(void*)) & Tasks::RefreshWDTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_batteryLevel, (void(*)(void*)) & Tasks::UpdateBatteryTask, this)) {
        cerr << "Error task start (bat_level): " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_reset, (void(*)(void*)) & Tasks::ResetTask, this)) {
        cerr << "Error task start (reset): " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    /*if (err = rt_task_start(&th_comCamera, (void(*)(void*)) & Tasks::ComCameraTask, this)) {
        cerr << "Error task start (comCamera): " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_actionCamera, (void(*)(void*)) & Tasks::ActionCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    cout << "Ressources releases launched" << endl << flush;
    //if(mutex_monitor)
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;
    rt_mutex_release(&mutex_robotStarted);
    rt_mutex_acquire(&mutex_move, TM_INFINITE);
    move = MESSAGE_ROBOT_STOP;
    rt_mutex_release(&mutex_move);
    
    
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    monitor.Close();
    rt_mutex_release(&mutex_monitor);
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Close();
    rt_mutex_release(&mutex_robot);
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
        cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
        // Synchronization barrier (waiting that all tasks are started)
        rt_sem_p(&sem_barrier, TM_INFINITE);
        /**************************************************************************************/
        /* The task server starts here                                                        */
        /**************************************************************************************/
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);

        cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        status = monitor.Open(SERVER_PORT);
        rt_mutex_release(&mutex_monitor);

        cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

        if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };
        monitor.AcceptClient(); // Wait the monitor client
        cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
        
        rt_sem_broadcast(&sem_serverOk);
        cout << "Server terminated" << endl;
        /*rt_sem_p(&sem_disconnectServer, TM_INFINITE);
        cout << "Reinitialization of the server !" << endl;
        rt_mutex_acquire(&mutex_reset, TM_INFINITE);
        reset = true;
        rt_mutex_release(&mutex_reset);*/
    
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
    FlushQueue(&q_messageToMon);
    cout << "Queue flushed" <<endl;
    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    bool notrst = true;
    int rbStarted;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   

    
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    

    /*rt_mutex_acquire(&mutex_reset, TM_INFINITE);
    rst = reset;
    rt_mutex_release(&mutex_reset);*/
    
    //while (!rst) {
    cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
    while(notrst) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
        /*rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rbStarted = robotStarted;
        rt_mutex_release(&mutex_robotStarted);*/
        if (msgRcv != NULL ){
            if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
                    cout << "Perte de connexion avec le moniteur" << endl;
                    notrst = false;
                    rt_sem_v(&sem_reset);
                    
         
                

                //rt_mutex_acquire(&mutex_reset, TM_INFINITE);
                //reset = false;
                //rt_mutex_release(&mutex_reset);
                
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
                rt_sem_v(&sem_openComRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
                //Indicating that we are starting without WatchDog
                rt_mutex_acquire(&mutex_watchDog, TM_INFINITE);
                watchDog = false;
                rt_mutex_release(&mutex_watchDog);
                //start
                rt_sem_v(&sem_startRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
                //Indicating that we are starting with WatchDog
                rt_mutex_acquire(&mutex_watchDog, TM_INFINITE);
                watchDog = true;
                rt_mutex_release(&mutex_watchDog);
                //start
                rt_sem_v(&sem_startRobot);
                
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
            } 

            delete msgRcv; // must be deleted manually, no consumer
            msgRcv = NULL;
        }
        
    }
    cout << "ReceiveFromMonTask finished " << endl;
    //rt_sem_v(&sem_serverOk);
    //rt_sem_v(&sem_barrier);
    
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;
    //bool rst;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    /*rt_mutex_acquire(&mutex_reset, TM_INFINITE);
    rst = reset;
    rt_mutex_release(&mutex_reset);
    while (rst) {*/
    cout << "open is waiting "<< endl;
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;
        
        cout << "Queue fflushed " << endl;
        FlushQueue(&q_messageToMon);

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            cout << "ACK" << endl;
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        msgSend = NULL;
        //rt_sem_v(&sem_refreshWD);
        cout << "OpenComRobot terminated " << endl;
        
}

/**
 * @brief Thread starting the communication with the robot.
 */
    void Tasks::StartRobotTask(void *arg) {
    
    bool wd, rst;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    //Signal from ReceiveFromMon that info has been sent about the type of connexion needed with the robot
    rt_sem_p(&sem_startRobot, TM_INFINITE);
    startRobot_alive = true;
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
    Message * msgSend;
    //Mutex for boolean WatchDog
    rt_mutex_acquire(&mutex_watchDog, TM_INFINITE);
    wd = watchDog;
    rt_mutex_release(&mutex_watchDog);

    //WatchDog parameter distinction
    if (wd) {
        cout << "Start robot with watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;
    } else {
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;
    }
    
    
    cout << "Movement answer: " << msgSend->ToString() << endl << flush;
    WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

    if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 1;
        rt_mutex_release(&mutex_robotStarted);
    }
    rt_sem_broadcast(&sem_otherComRobot);//StartRobot and refreshWD have to start approximately at the same time since they both start their timer for the watchdog sync
    startRobot_alive = false;
    
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message* response;
    int cpt;
    bool rst;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)

    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(100000000));
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /*rt_mutex_acquire(&mutex_reset, TM_INFINITE);
    rst = reset;
    rt_mutex_release(&mutex_reset);
    while (rst) {*/
    rt_sem_p(&sem_otherComRobot, TM_INFINITE);
    cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
    while(1){
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove << endl;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            response = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            
           
        }
        cout << endl << flush;
    }
}

/**
 * Flush all the messages in a given queue
 * @param queue Queue identifier
 */
void Tasks::FlushQueue(RT_QUEUE *queue) {
    int err;
    if ((err = rt_queue_flush(queue)) < 0) {
        cerr << "Flush queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in flush queue"};
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

/**************************************Our Tasks********************************************/

//Refresh WatchDog Task
void Tasks::RefreshWDTask(void *arg)
{
    int rs_status;
    bool rst;
    //Synchronization barrier
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_otherComRobot, TM_INFINITE);
    //Beginning of the Task
    //rt_task_set_periodic(&th_refreshWD, TM_NOW, 955000000);
    rt_task_set_periodic(&th_refreshWD, TM_NOW, rt_timer_ns2ticks(1000000000));
   
    /*rt_mutex_acquire(&mutex_reset, TM_INFINITE);
    rst = reset;
    rt_mutex_release(&mutex_reset);
    while(rst) {*/
    // Point of sync between the robot and the supervisor for both timers to start approximately in the same time 
    //rt_sem_p(&sem_startRobot, TM_INFINITE);
    
    cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
    while(1) {
        rt_task_wait_period(NULL);
        cout << "\e[31mWatchDog Refresh\e[0m" << endl;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs_status = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs_status) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

void Tasks::UpdateBatteryTask(void * arg) {
    
    //Variables
    Message * msgSend;
    int rs;
    bool rst;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    //Synchronization barrier, awaiting for all the tasks to start
    rt_sem_p(&sem_barrier, TM_INFINITE);
    //rt_sem_p(&sem_batteryLevel, TM_INFINITE);
    rt_sem_p(&sem_otherComRobot, TM_INFINITE); // Waiting of the starting of the robot
    //Task
    rt_task_set_periodic(&th_batteryLevel, TM_NOW, rt_timer_ns2ticks(50000000));
    
    //cout << "Battery started" << endl;
    /*rt_mutex_acquire(&mutex_reset, TM_INFINITE);
    rst = reset;
    rt_mutex_release(&mutex_reset);
    while (rst) {*/
    
    cout << "Start of " << __PRETTY_FUNCTION__ << endl << flush;
    while(1){
        //cout << "Battery started while" << endl;
        rt_task_wait_period(NULL);
        
        //Verify if the Robot is active
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs) {
           cout << "Update battery" << endl << flush;
           
           //Get the battery level update
           rt_mutex_acquire(&mutex_robot, TM_INFINITE);
           msgSend = robot.Write(robot.GetBattery());
           rt_mutex_release(&mutex_robot);
           
           if(msgSend != NULL && !msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT))
           {
               //Send the battery level update to the monitor
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgSend);
                rt_mutex_release(&mutex_monitor);
           } else {
               cout << "Response unknown \n" << endl;
           }
        }
    }
}


/*void MessageRobot(Message * msg){
    Message * msgSend;
    
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    msgSend = robot.Write(robot.GetBattery());
    rt_mutex_release(&mutex_robot);
    
    
     //Error detection
    if (response->CompareID(MESSAGE_ANSWER_ROBOT_ERROR)) {
        //Incrementing the counter
        rt_mutex_acquire(&mutex_error_count, TM_INFINITE);
        error_count++;
        cpt = error_count;
        rt_mutex_release(&mutex_error_count);

        cout << "Communication error" << endl << flush;

        //If the number of errors exceeds 3
        if (cpt>=3) {
            cout << "Restart" << endl << flush;
            // Send message to the monitor 
            Message m = MESSAGE_ANSWER_COM_ERROR ;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(&m);
            rt_mutex_release(&mutex_monitor);
            //Close the ComRobot communication
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            robot.Reset();
            rt_mutex_release(&mutex_robot);
        }
    } else {
        //Error Count Reset back to 0
        rt_mutex_acquire(&mutex_error_count, TM_INFINITE);
        error_count = 0;
        rt_mutex_release(&mutex_error_count);
    }
}*/

void Tasks::ComCameraTask(void * arg) {
    //Waiting of the receivFromMon
}

void Tasks::ActionCameraTask(void * arg) {
    //Waiting of the receivFromMon
}

void Tasks::CloseRobotTask(void * arg) {
   /* while(1){
        rt_sem_p(&sem_closeRobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Close();
        rt_mutex_release(&mutex_monitor);  
    }*/
    
}

void Tasks::Quit(){
    cout << "Proper release of all ressources !" << endl << flush;
    Stop();
}

void Tasks::ResetTask(void * arg){
    //int status;
    int err;
    while(1)
    {
        rt_sem_p(&sem_reset, TM_INFINITE);
        cout <<  "******** Reinitialization actionned ******" << endl;
        if(err = rt_task_join(&th_openComRobot)){
            cout << "Error th_openComRobot : " << strerror(-err) << endl;
        }
        cout << "join com" << endl;
        if(err = rt_task_join(&th_server)){
            cout << "Error server : " << strerror(-err) << endl;
        }
        cout << "join server" << endl;
       /* if (!startRobot_alive){
             
            if(err = rt_task_join(&th_startRobot)){
                cout << "Error th_startRobot : " << strerror(-err) << endl;
            }
            cout << "join start" << endl;
        }*/
       
        
        
        Stop();
        cout << "Continue" << endl;
        /**************************************************************************************/
        /* Tasks ending                                                                     */
        /**************************************************************************************/
        /*if(err = rt_task_delete(&th_server)){
            cerr << "Error task deletion (server): " << strerror(-err) << endl << flush;
        }*/
        if(err = rt_task_delete(&th_sendToMon)){
            cerr << "Error task deletion (sendToMon): " << strerror(-err) << endl << flush;
        }
         cout << "Continue send " << startRobot_alive << endl;
         
        if(err = rt_task_unblock(&th_receiveFromMon)){
            cerr << "Error task unblock (receiveFromMon): " << strerror(-err) << endl << flush;
        }
        cout << "Unblock" << endl;
        if(err = rt_task_delete(&th_receiveFromMon)){
            cerr << "Error task deletion (receiveFromMon): " << strerror(-err) << endl << flush;
        }
        /*if(th_openComRobot != NULL && err = rt_task_delete(&th_openComRobot)){
            cerr << "Error task deletion (openComRobot): " << endl << flush;
        }*/
        cout << "Continue send afet receiv " << startRobot_alive << endl;
        if (startRobot_alive){
             cout << "START ROBOT DELETED " << endl;
            if(err = rt_task_delete(&th_startRobot)){
                cerr << "Error task deletion (startRobot): " << strerror(-err) << endl << flush;
            }
            cout << "START ROBOT DELETED " << endl;
        }
        if(err = rt_task_delete(&th_move)){
            cerr << "Error task deletion (move) : " << strerror(-err) << endl << flush;
        }
        if(err = rt_task_delete(&th_refreshWD)){
            cerr << "Error task deletion (refreshWD): " << strerror(-err) << endl << flush;
        }
        if(err = rt_task_delete(&th_batteryLevel)){
            cerr << "Error task deletion (batteryLevel): " << strerror(-err) << endl << flush;
        }
        if(err = rt_task_delete(&th_comCamera)){
            cerr << "Error task deletion (comCamera): " << strerror(-err) << endl << flush;
        }
        if(err = rt_task_delete(&th_actionCamera)){
            cerr << "Error task deletion (actionCamera): " << strerror(-err) << endl << flush;
        }


        /**************************************************************************************/
        /* Tasks creation                                                                     */
        /**************************************************************************************/
        if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
            cerr << "Error task create th_server: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
            cerr << "Error task create th_sendToMon: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
            cerr << "Error task create th_receiveFromMon: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, T_JOINABLE)) {
            cerr << "Error task create th_openComRobot: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
            cerr << "Error task create th_startRobot: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
            cerr << "Error task create th_move: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }

        if (err = rt_task_create(&th_refreshWD, ",th_refreshWD", 0, PRIORITY_TREFRESHWD, 0)) {
            cerr << "Error task create th_refreshWD: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_create(&th_batteryLevel, "th_batteryLevel", 0, PRIORITY_TBATTERYLEVEL, 0)) {
            cerr << "Error task create th_batteryLevel: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }

        if (err = rt_task_create(&th_comCamera, "th_comCamera", 0, PRIORITY_TCAMERA, 0)) {
            cerr << "Error task create th_comCamera: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }

        if (err = rt_task_create(&th_actionCamera, "th_actionCamera", 0, PRIORITY_TCAMERA, 0)) {
            cerr << "Error task create th_actionCamera: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        
        
        /**************************************************************************************/
        /* Tasks starting                                                                   */
        /**************************************************************************************/
        if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
        }
        if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
    
        /*if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }*/
        if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }

        //Our Tasks
        if (err = rt_task_start(&th_refreshWD, (void(*)(void*)) & Tasks::RefreshWDTask, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }

        if (err = rt_task_start(&th_batteryLevel, (void(*)(void*)) & Tasks::UpdateBatteryTask, this)) {
            cerr << "Error task start (bat_level): " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }

       /* if (err = rt_task_start(&th_batteryLevel, (void(*)(void*)) & Tasks::UpdateBatteryTask, this)) {
            cerr << "Error task start (bat_level): " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }*/
        
        
        rt_task_sleep(rt_timer_ns2ticks(1000000000));
        
        cout << "Tasks synchronized" << endl << flush;
        rt_sem_broadcast(&sem_barrier);
        cout <<  "******** Reinitialization performed ******" << endl;
        
        
    }
    
}
