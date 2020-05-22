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
#include <execinfo.h>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 20 // regular
#define PRIORITY_TOPENCOMROBOT 31// very high prior
#define PRIORITY_TMOVE 25 // high prior
#define PRIORITY_TSENDTOMON 31 // very high prior
#define PRIORITY_TRECEIVEFROMMON 31 // very high prior
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
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_continueStream, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_actionType, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraCommand, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_period, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arenaConfirmed, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_reset, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier_monitor, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
   /* if (err = rt_sem_create(&sem_barrier_robot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Creation of both sem_serverOk2 and 3 because of a high feeling of disfunctionnement of rt_sem_broadcast during some test
    // Precisely when it comes of the reinitialization of the semaphore to zero. It seems that other thread could pass rt_sem_p
    // just after a broadcast.
    if (err = rt_sem_create(&sem_serverOk2, NULL, 0, S_FIFO)) { 
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk3, NULL, 0, S_FIFO)) {
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
    if (err = rt_sem_create(&sem_camera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_comCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startStream, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arenaResult, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_sem_create(&sem_reset, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
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
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
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

    
    /*if (err = rt_task_create(&th_comCamera, "th_comCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_actionCamera, "th_actionCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    
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
    int err;
    cout << "Ressources releases" << endl << flush;
    rt_mutex_acquire(&mutex_move, TM_INFINITE);
    move = MESSAGE_ROBOT_STOP;
    //cout << "Move STOP" << endl << flush;
    rt_mutex_release(&mutex_move);
   
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;
    //cout << "RobotStarted = 0" << endl << flush;
    rt_mutex_release(&mutex_robotStarted);
    
    rt_mutex_delete(&mutex_monitor);
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    //cout << "Acqu mutex monitor" << endl << flush;
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    //cout << "Attempt close monitor" << endl << flush;
    monitor.Close();
    cout << "Monitor closed" << endl << flush;
    rt_mutex_release(&mutex_monitor);
    
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Close();
    cout << "Robot closed" << endl << flush;
    rt_mutex_release(&mutex_robot);
    
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier_monitor);
    pause();
}

void sigIgnor(int i){
    //ignore
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    while(1){
        //cout << "SeverTask is waiting barrier_monitor " << endl << flush;

        rt_sem_p(&sem_barrier_monitor, TM_INFINITE);
        //rt_task_sleep(rt_timer_ns2ticks(2000000000));
        cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
        /**************************************************************************************/
        /* The task server starts here                                                        */
        /**************************************************************************************/
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        status = monitor.Open(SERVER_PORT);
        rt_mutex_release(&mutex_monitor);

        //Before
        /*if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };*/
        //After
        if (status >= 0){
            cout << color("Open monitor opened on port ", COLOR_GREEN) << (SERVER_PORT) << endl << flush;
            monitor.AcceptClient(); // Wait the monitor client
            cout << color("Monitor connected !", COLOR_GREEN) << endl << flush;
            rt_sem_broadcast(&sem_serverOk);
            rt_sem_v(&sem_serverOk2);
            rt_sem_v(&sem_serverOk3);
            cout << color("Server terminated ", COLOR_BLEU) << endl << flush;
        }
        else{
            cout << color("Error when openening the communication with monitor", COLOR_RED) << endl << flush;
        }
        cout << color("SendToMon terminated", COLOR_BLEU) << endl;
    }
        
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    int monitorAlive = 1;
    while(1){
        cout << color("SendToMonTask is waiting barrier ", COLOR_BLEU) << endl << flush;
        // Synchronization barrier (waiting that all tasks are starting)
        //rt_sem_p(&sem_barrier, TM_INFINITE);
        //cout << color("SendToMonTask is waiting Server ", COLOR_BLEU) << endl << flush;
        
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        rt_sem_p(&sem_serverOk3, TM_INFINITE);
        
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;

        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitorAlive = monitor.isAlive(); 
        rt_mutex_release(&mutex_monitor);
       
        while (monitorAlive) {
            //cout << "wait msg to send" << endl << flush;
            try{
             
                msg = ReadInQueue(&q_messageToMon);
                
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msg); // The message is deleted with the Write
                monitorAlive = monitor.isAlive();
                rt_mutex_release(&mutex_monitor); 
            }
            catch(std::runtime_error& error ){ // This is unblock the SendToMonTask which is waiting for message to send
                //cout << "Unblock catched " << endl << flush;
                rt_mutex_release(&mutex_monitor); 
                monitorAlive = false; // We can go outside the internal while and restarts
            }
            
        }
        cout << color("SendToMon terminated", COLOR_BLEU) << endl;
    }
        
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    bool notreinit;
    int rob_rst;
    while(1){
        notreinit = true;
        cout << color("ReceivFromMon is waiting barrier ", COLOR_BLEU) << endl << flush;
        //cout << color("ReceivFromMon is waiting Server ", COLOR_BLEU) << endl << flush;
        
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        rt_sem_p(&sem_serverOk2, TM_INFINITE);
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;
        
        while (notreinit) {
            msgRcv = monitor.Read();
            cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
   
            if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
                if(!reset_ongoing){
                    cout << color("Perte de connexion avec le moniteur", COLOR_RED) << endl << flush;
                    notreinit = false; // When a lost is detected we restart the ReceivFromMonTask
                    rt_sem_v(&sem_reset); // We start the ResetTask  
                }
                else{
                    rt_mutex_acquire(&mutex_reset, TM_INFINITE);
                    rob_rst = reset_robot; // Reset Robot
                    rt_mutex_release(&mutex_reset);
                    if(rob_rst){
                        cout << color("Perte de connexion avec le moniteur", COLOR_RED) << endl << flush;
                        notreinit = false; // When a lost is detected we restart the ReceivFromMonTask
                        rt_sem_v(&sem_reset); // We start the ResetTask  
                    }
                    
                    cout << color("Double reset avoided", COLOR_GREEN) << endl << flush;
                    notreinit = false; // When a lost is detected we restart the ReceivFromMonTask
                }
                //exit(-1);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
                rt_sem_broadcast(&sem_openComRobot);
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
                //Refresh Watchdog with th_refreshWD
                rt_sem_v(&sem_refreshWD) ;
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA) ||
                    msgRcv->CompareID(MESSAGE_CAM_OPEN) ||
                    msgRcv->CompareID(MESSAGE_CAM_CLOSE) ||
                    msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM) ||
                    msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {

                rt_mutex_acquire(&mutex_cameraCommand, TM_INFINITE);
                cameraCommand = msgRcv->GetID();
                rt_mutex_release(&mutex_cameraCommand);

                rt_sem_v(&sem_comCamera);
            }

            delete(msgRcv); // mus be deleted manually, no consumer
        }
        cout << color("ReceiveFromMonTask finished ", COLOR_BLEU) << endl;
    }
        
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;
    while(1){
        //cout << color("OpenComRobot is waiting  barrier ", COLOR_BLEU) << endl << flush;
        // Synchronization barrier (waiting that all tasks are starting)
        //cout << color("OpenComRobot is waiting order from ReceivMonitor ", COLOR_BLEU) << endl << flush;

        rt_sem_p(&sem_serverOk, TM_INFINITE);
        cout << color("Ready to connect with robot ", COLOR_GREEN) << endl << flush;
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;

        
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        cout << color("Communication successfuly opened with the robot (", COLOR_GREEN) << status << ")" << endl << flush;
        rt_mutex_release(&mutex_robot);

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            cout << color("Robot NACK", COLOR_RED) << endl << flush;
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            cout << color("Robot ACK", COLOR_GREEN) << endl << flush;

        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        cout << color("OpenComRobot terminated ", COLOR_BLEU) << endl << flush;
    }
}



/**
 * @brief Thread starting the communication with the robot.
 */
    void Tasks::StartRobotTask(void *arg) {
    Message * msgSend;
    bool wd;
    while(1){
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << color("Ready to start robot ", COLOR_GREEN) << endl << flush;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;
        //Mutex for boolean WatchDog
        rt_mutex_acquire(&mutex_watchDog, TM_INFINITE);
        wd = watchDog;
        rt_mutex_release(&mutex_watchDog);

        //WatchDog parameter distinction
        if (wd) {
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_sem_v(&sem_refreshWD);
            rt_mutex_release(&mutex_robot);
            cout << "Start robot with watchdog ("<< msgSend->GetID() << ")" << endl;
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
        
        rt_sem_broadcast(&sem_otherComRobot);//StartRobot and refreshWD have to start approximately at the same time since they both start their timer for the watchdog synchronization
        
        //}
        //cout << color("StartRobot ternminated ", COLOR_BLEU) << endl;
    }
    
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message* response;
    rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(100000000));
    signal (SIGPIPE, SIG_IGN);
    signal (SIGSEGV, sigIgnor);
    while(1){    
        //cout << "MoveTask is waiting barrier" << endl << flush;
        rt_sem_p(&sem_otherComRobot, TM_INFINITE);
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        while (rs) {
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

                WriteRobot(new Message((MessageID)cpMove));
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                rs = robotStarted;
                rt_mutex_release(&mutex_robotStarted);
                
            }
            cout << endl << flush;
        }
        cout << color("MoveTask terminated", COLOR_BLEU) << endl << flush;

        
    }
}

/**
 * Handlke the lost of the communication with the robot
 * @param msg
 */
Message* Tasks::WriteRobot(Message* msg){
    Message* response = NULL ;
    int cpt;
    Message* m;
    rt_mutex_acquire(&mutex_error_count, TM_INFINITE);
    cpt = error_count;
    rt_mutex_release(&mutex_error_count);
    if(cpt< 3){
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        try{
        response = robot.Write(msg);
        }
        catch(std::runtime_error& error ){ 
            //cout << "Runtime Catcheed at WriteRobot" << endl << flush;
            response = new Message(MESSAGE_ANSWER_ROBOT_ERROR);
        }
        rt_mutex_release(&mutex_robot);
        
        //Error detection
                    if (response->CompareID(MESSAGE_ANSWER_ROBOT_ERROR) || response->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                        //Incrementing the counter
                        rt_mutex_acquire(&mutex_error_count, TM_INFINITE);
                        error_count++;
                        cpt = error_count;
                        rt_mutex_release(&mutex_error_count);

                        cout << color("Communication error ", COLOR_RED) << cpt << endl << flush;


                        //If the number of errors exceeds 3
                        if (cpt==3) {
                            cout << color("RESTART", COLOR_RED) << endl << flush;
                            // Send message to the monitor 
                            m = new Message(MESSAGE_ANSWER_COM_ERROR);
                            WriteInQueue(&q_messageToMon, m);
                            //Close the ComRobot communication
                            rt_mutex_acquire(&mutex_reset, TM_INFINITE);
                            reset_robot = 1; // Reset Robot
                            rt_mutex_release(&mutex_reset);
                            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                            robotStarted = 0;
                            rt_mutex_release(&mutex_robotStarted);
                            rt_sem_v(&sem_reset);
                        }
                    } else {
                        //Error Count Reset back to 0
                        rt_mutex_acquire(&mutex_error_count, TM_INFINITE);
                        error_count = 0;
                        rt_mutex_release(&mutex_error_count);
                    }
    }
        
    return response;
}

/**
 * Flush a given queue
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
        if(reset_ongoing == 0)
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
    }
    

    return msg;
}

/**************************************Our Tasks********************************************/

//Refresh WatchDog Task
void Tasks::RefreshWDTask(void *arg)
{
    int rs_status = 1;
    rt_task_set_periodic(&th_refreshWD, TM_NOW, rt_timer_ns2ticks(1000000000));
    while (1){
        cout << "RefreshWDTask is waiting barrier" << endl << flush;
        rt_sem_p(&sem_otherComRobot, TM_INFINITE);
        rt_sem_p(&sem_refreshWD, TM_INFINITE);
        
        //rt_task_sleep(rt_timer_ns2ticks(650000000)); // Sync
        //rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        //robot.Write(robot.ReloadWD());
        //rt_mutex_release(&mutex_robot);
        
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;
        /*rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs_status = robotStarted;
        rt_mutex_release(&mutex_robotStarted);*/
        while(rs_status) {
            rt_task_wait_period(NULL);
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs_status = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            if(rs_status){
                cout << color("WatchDog Refresh", COLOR_RED) << endl << flush;
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                WriteRobot(robot.ReloadWD());
                rt_mutex_release(&mutex_robot);
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                rs_status = robotStarted;
                rt_mutex_release(&mutex_robotStarted);
            }
            
        }
        cout << color("RefreshWatchDog terminated", COLOR_BLEU) << endl << flush;
    }
    
}

// Ours Tasks implementations
void Tasks::UpdateBatteryTask(void * arg) {
    
    //Variables
    Message * msgSend = NULL;
    int rs;
    rt_task_set_periodic(&th_batteryLevel, TM_NOW, rt_timer_ns2ticks(50000000));
    signal (SIGPIPE, SIG_IGN);
    while(1){
        //Synchronization barrier, awaiting for all the tasks to start
        //rt_sem_p(&sem_barrier, TM_INFINITE);
        
        rt_sem_p(&sem_otherComRobot, TM_INFINITE); // Waiting of the starting of the robot
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        while (rs) {
            rt_task_wait_period(NULL);

            

            cout << "Update battery" << endl << flush;

            //Get the battery level update
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            
            try{
                if(rs)
                msgSend = WriteRobot(robot.GetBattery());
            }catch(std::runtime_error& error ){ // This is unblock the SendToMonTask which is waiting for message to send
                cout << "Unblock catched " << error.what() <<  endl << flush;
                if(strcmp(error.what(), "broken_pipe") == 0){
                    cout << "RESSSSSSSSSSSSSEEEEEEEEEEEEETTTTTTT" << endl << flush;
                    /*rt_mutex_acquire(&mutex_reset, TM_INFINITE);
                    reset_robot = 1; // Reset Robot
                    rt_mutex_release(&mutex_reset);
                    rt_sem_v(&sem_reset);*/
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0; // Ensure that we leave the while at the next iteration
                    rt_mutex_release(&mutex_robotStarted);
                }
            }
            rt_mutex_release(&mutex_robot);

            if(msgSend != NULL && !msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT))
            {
                //Send the battery level update to the monitor
                 WriteInQueue(&q_messageToMon, msgSend);
            } else {
                cout << "Response unknown" << endl << flush;
            }
            
            //Verify if the Robot is active
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            
        }
        cout << color("UpdateBatteryTask terminated", COLOR_BLEU) << endl << flush;

    }
}


void Tasks::ComCameraTask(void * arg) {
    int auxCameraCommand;
    bool status;
    
    while(1){
        //Synchronization barrier, awaiting for all the tasks to start
        rt_sem_p(&sem_otherComRobot, TM_INFINITE); // Make sure that the connection with the robot has been established
        rt_sem_p(&sem_comCamera, TM_INFINITE);
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;


        rt_mutex_acquire(&mutex_cameraCommand, TM_INFINITE);
        auxCameraCommand = cameraCommand;
        rt_mutex_release(&mutex_cameraCommand);

        Message m;

        switch (auxCameraCommand) {

            case MESSAGE_CAM_OPEN:

                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                status = cam.Open();
                rt_mutex_release(&mutex_camera);

                if (status) {
                    m = MESSAGE_ANSWER_ACK ;
                    WriteInQueue(&q_messageToMon, &m);

                    rt_mutex_acquire(&mutex_continueStream, TM_INFINITE);
                    continueStream = true ;
                    rt_mutex_release(&mutex_continueStream);

                    rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                    actionType = CAMERA_STREAM ;
                    rt_mutex_release(&mutex_actionType);

                    rt_mutex_acquire(&mutex_period, TM_INFINITE);
                    period = rt_timer_ns2ticks(100000000);
                    rt_mutex_release(&mutex_period);

                    rt_sem_v(&sem_startStream);
                } else {
                    m = MESSAGE_ANSWER_NACK ;
                    WriteInQueue(&q_messageToMon, &m);
                }
                break;

            case MESSAGE_CAM_CLOSE:

                m = MESSAGE_ANSWER_ACK ;
                WriteInQueue(&q_messageToMon, &m);

                rt_mutex_acquire(&mutex_continueStream, TM_INFINITE);
                continueStream = false ;
                rt_mutex_release(&mutex_continueStream);

                break;

            case MESSAGE_CAM_ASK_ARENA:

                rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                actionType = CAMERA_ASK_ARENA ;
                rt_mutex_release(&mutex_actionType);

                break;

            case MESSAGE_CAM_ARENA_CONFIRM:

                rt_mutex_acquire(&mutex_arenaConfirmed, TM_INFINITE);
                arenaConfirmed = true ;
                rt_mutex_release(&mutex_arenaConfirmed);

                rt_sem_v(&sem_arenaResult);

                break;

            case MESSAGE_CAM_ARENA_INFIRM:

                rt_mutex_acquire(&mutex_arenaConfirmed, TM_INFINITE);
                arenaConfirmed = false ;
                rt_mutex_release(&mutex_arenaConfirmed);

                rt_sem_v(&sem_arenaResult);

                break;

            case MESSAGE_CAM_POSITION_COMPUTE_START:

                rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                actionType = CAMERA_FIND_POSITION ;
                rt_mutex_release(&mutex_actionType);

                break;

            case MESSAGE_CAM_POSITION_COMPUTE_STOP:

                rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                actionType = CAMERA_STREAM ;
                rt_mutex_release(&mutex_actionType);

                break;

            default:
                break;
        }

    }
}

void Tasks::ActionCameraTask(void * arg) {
    bool auxContinueStream;
    int auxActionType;
    Arena auxArena;
    bool auxArenaConfirmed;
    Position position;
    //Task
    rt_mutex_acquire(&mutex_period, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, period); 
    rt_mutex_release(&mutex_period);
    while (1){
        cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

        //Synchronization barrier, awaiting for all the tasks to start
        //rt_sem_p(&sem_barrier, TM_INFINITE);
        rt_sem_p(&sem_startStream, TM_INFINITE);
        cout << color("Start ", COLOR_BLEU) << __PRETTY_FUNCTION__ << endl << flush;

        




        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        Img img = cam.Grab();
        rt_mutex_release(&mutex_camera);

        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        Img imgArena = cam.Grab();
        rt_mutex_release(&mutex_camera);



        while (1) {

            rt_mutex_acquire(&mutex_continueStream, TM_INFINITE);
            auxContinueStream = continueStream;
            rt_mutex_release(&mutex_continueStream);

            if (auxContinueStream) {

                rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                auxActionType = actionType;
                rt_mutex_release(&mutex_actionType);

                switch(auxActionType) {

                    case CAMERA_FIND_POSITION:

                        rt_task_set_periodic(NULL, TM_NOW, 0); 

                        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                        auxArena = arena;
                        rt_mutex_release(&mutex_arena);

                        if (auxArena.IsEmpty()) {

                            WriteInQueue(&q_messageToMon, new MessagePosition());

                        } else {

                            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                            img = cam.Grab();
                            rt_mutex_release(&mutex_camera);

                            position = img.SearchRobot(auxArena).front();

                            WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION,position));

                            rt_task_set_periodic(NULL, TM_NOW, period);

                            rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                            actionType = CAMERA_STREAM;
                            rt_mutex_release(&mutex_actionType);

                        }

                        break;

                    case CAMERA_STREAM:

                        rt_task_wait_period(NULL);

                        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                        img = cam.Grab();
                        rt_mutex_release(&mutex_camera);

                        WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE,&img));

                        break;

                    case CAMERA_ASK_ARENA:

                        rt_task_set_periodic(NULL, TM_NOW, 0);

                        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                        imgArena = cam.Grab();
                        rt_mutex_release(&mutex_camera);

                        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                        arena = imgArena.SearchArena();
                        auxArena = arena;
                        rt_mutex_release(&mutex_arena);

                        if (auxArena.IsEmpty()) {

                            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));

                            rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                            actionType = CAMERA_STREAM;
                            rt_mutex_release(&mutex_actionType);

                        } else {

                            imgArena.DrawArena(auxArena);

                            WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE,&imgArena));

                            rt_sem_p(&sem_arenaResult, TM_INFINITE);

                            rt_mutex_acquire(&mutex_arenaConfirmed, TM_INFINITE);
                            auxArenaConfirmed = arenaConfirmed ;
                            rt_mutex_release(&mutex_arenaConfirmed);

                            if (auxArenaConfirmed) {
                                cout << "Arena saved" << endl;
                            }

                            rt_task_set_periodic(NULL, TM_NOW, period);

                            rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                            actionType = CAMERA_STREAM;
                            rt_mutex_release(&mutex_actionType);

                        }

                        break;

                    default:
                        break;

                }

            } else {

                return;

            }

        }
    }

}
        


void Tasks::ResetTask(void * arg) {
    int err;
    char rst_robot;
    while(1){
        rt_sem_p(&sem_reset, TM_INFINITE);
        cout << color("Reset started", COLOR_BLEU) << endl << flush;
        rt_mutex_acquire(&mutex_reset, TM_INFINITE);
        reset_ongoing = 1;
        rt_mutex_release(&mutex_reset);
        /*if(rst_robot){ // Reset robot
            
            
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = MESSAGE_ROBOT_STOP;
            //cout << "Move STOP" << endl << flush;
            rt_mutex_release(&mutex_move);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            //Flush receiving buffer of server dedicated for the monitor
            monitor.FlushReceivBuffer();
            rt_mutex_release(&mutex_monitor);
          
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            cout << "Robot closed" << endl << flush;
            rt_mutex_release(&mutex_robot);
            
            rt_mutex_acquire(&mutex_reset, TM_INFINITE);
            reset_robot = 0; // Restart init state; Process must require robot reset
            rt_mutex_release(&mutex_reset);
            
        } else{ // Global reset*/
            cout << "Reset global" << endl << flush;
            FlushQueue(&q_messageToMon);
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            //cout << "RobotStarted = 0" << endl << flush;
            rt_mutex_release(&mutex_robotStarted);
            rt_task_unblock(&th_sendToMon);
            
            Stop();
            rt_sem_broadcast(&sem_barrier_monitor);
        //}
        rt_mutex_acquire(&mutex_reset, TM_INFINITE);
        reset_ongoing = 0;
        rt_mutex_release(&mutex_reset);
        cout << color("Reset finished", COLOR_BLEU) << endl << flush;
    }
    
}

std::string Tasks::color(string stg, int color){
    switch(color){
        case COLOR_RED:
            return "\e[31m" + stg + "\e[0m";
        case COLOR_GREEN:
            return "\e[32m" + stg + "\e[0m";
        case COLOR_BLEU:
            return "\e[34m" + stg + "\e[0m";
        default:
            return stg;
            break;
            
    }
}

