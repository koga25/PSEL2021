#include "modules/structs/structs.h"
#include <QCoreApplication>
#include <math.h>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

using namespace std;
using namespace std::chrono;

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
//if you want to benchmark the two functions, and compare the outputs from them, remove the comments, in my pc it is 6 times faster.
float calculateAngle(struct Position robot, struct Position ball) {
    float angle = atan2(ball.y - robot.y, ball.x - robot.x);
    //const float angle = acos(((robot.x*ball.x) + (robot.y * ball.y))/(sqrt((robot.x * robot.x) + (robot.y * robot.y)) * sqrt((ball.x * ball.x) + (ball.y * ball.y))));
    //std::cout << angle << std::endl;
    return angle;
}

float fastCalculateAngle(float y, float x)
{
  float t0, t1, t2, t3, t4;
  t3 = abs(x);
  t1 = abs(y);
  t0 = max(t3, t1);
  t1 = min(t3, t1);
  t3 = float(1) / t0;
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 =         - float(0.013480470);
  t0 = t0 * t4 + float(0.057477314);
  t0 = t0 * t4 - float(0.121239071);
  t0 = t0 * t4 + float(0.195635925);
  t0 = t0 * t4 - float(0.332994597);
  t0 = t0 * t4 + float(0.999995630);
  t3 = t0 * t3;

  t3 = (abs(y) > abs(x)) ? float(1.570796327) - t3 : t3;
  t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  if(std::isnan(t3)) {
      t3 = 0;
  }
  //std::cout << t3 << std::endl;
  return t3;
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);
    // Desired frequency
    int desiredFrequency = 60;
    //int m = 0;
    //const int iterations = 1000;
    //uint64_t calculateAngleSum = 0;
    //uint64_t fastCalculateAngleSum = 0;
    while(true) {
        //m++;
        //SSL_DetectionBall ball = vision->getLastBallDetection();
        //struct Position ballPosition = { ball.x(), ball.y()};
        //SSL_DetectionRobot robot = vision->getLastRobotDetection(false, 2);
        //struct Position robotPosition = {robot.x(), robot.y()};
        //std::chrono::high_resolution_clock::time_point normal = std::chrono::high_resolution_clock::now();
        //calculateAngle(robotPosition, ballPosition);
        //calculateAngleSum += time_point_cast<nanoseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count() - time_point_cast<nanoseconds>(normal).time_since_epoch().count();
        //std::chrono::high_resolution_clock::time_point fast = std::chrono::high_resolution_clock::now();
        //fastCalculateAngle(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        //fastCalculateAngleSum += time_point_cast<nanoseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count() - time_point_cast<nanoseconds>(fast).time_since_epoch().count();
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();


        //You spin my head right round, right round
        actuator->sendCommand(false, 0, 0.0, 0.0, 20.0);

        //dribble and shoot
        SSL_DetectionBall ball = vision->getLastBallDetection();
        struct Position ballPosition = { ball.x(), ball.y()};
        SSL_DetectionRobot robot = vision->getLastRobotDetection(false, 2);

        struct Position robotPosition = {robot.x(), robot.y()};
        //std::cout << robot.x() << "   " << robot.y() << "   " << robot.orientation() << std::endl;
        float angle = fastCalculateAngle(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        actuator->runToBall(false, robot.orientation(), 2, &ballPosition, &robotPosition, angle);

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    //std::cout << "fastCalculate mean is: " << fastCalculateAngleSum/iterations << std::endl << "calculate mean is: "<< calculateAngleSum/iterations << std::endl;

    return a.exec();
}
