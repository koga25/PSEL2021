#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <modules/math/fastMath.h>
#include <modules/structs/structs.h>



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


robot length ≃ 0.7f
wheel radius ≃ 0.05f
*/

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;
    //flag for knowing if it should begin circling around or not.
    bool circleOrientation = false;
    bool fixPosition = true;
    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        fira_message::Ball ball = vision->getLastBallDetection();
        struct Position ballPosition = { ball.x(), ball.y() };
        fira_message::Robot robot = vision->getLastRobotDetection(true, 1);


        struct Position robotPosition = { robot.x(), robot.y() };
        double distance = calculateDistanceBetweenPoints(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        double angle = fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        actuator->circleTheBall(true, 1, distance, robot.orientation(), angle, 0.3, &circleOrientation);

        //actuator->makeGoal(true, 1, robot.orientation(), angle, &fixPosition, robotPosition, ballPosition);


        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
