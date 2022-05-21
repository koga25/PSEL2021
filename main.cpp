#include "modules/structs/structs.h"
#include <QCoreApplication>
#include <math.h>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <modules/math/fastMath.h>
#include <modules/RobotLogic/robotLogic.h>



/*orientation = output of atan2()
                              𝝿/2
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
 -𝝿 | 𝝿 ───────────────────────┼─────────────────────────── 0
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                               │
                             -𝝿/2

*/


int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    //initializing Class that we will use to program the logic of the robots and send to the Actuator.
    RobotLogic *robotLogic = new RobotLogic(actuator, vision);

    // Desired frequency
    int desiredFrequency = 60;
    bool circleAround = false;
    while(true) {
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();


        //robotLogic->runToBall(false, 2, 5.0);
        //robotLogic->walkAroundPosition(false, 0, &circleAround);
        robotLogic->dribble(false, 0, true);

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }



    return a.exec();
}
