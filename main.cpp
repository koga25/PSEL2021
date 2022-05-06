#include "modules/structs/structs.h"
#include <QCoreApplication>
#include <math.h>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <modules/math/fastMath.h>



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

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);
    // Desired frequency
    int desiredFrequency = 60;

    while(true) {
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        actuator->walkAroundPosition();

        SSL_DetectionBall ball = vision->getLastBallDetection();
        struct Position ballPosition = { ball.x(), ball.y()};
        SSL_DetectionRobot robot = vision->getLastRobotDetection(false, 2);

        struct Position robotPosition = {robot.x(), robot.y()};
        float angle = fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        actuator->runToBall(false, robot.orientation(), 2, &ballPosition, &robotPosition, angle);

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }



    return a.exec();
}
