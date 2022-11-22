#include <stdio.h>
#include <string>
#include "src/CYdLidar.h"

//todo: 实现一个ros2的talker，将scan对象发布到一个topic中，再实现一个listener，监听scan对象
void handler(LaserScan scan) {
  fprintf(stdout, "Scan received[%lu]: %u ranges is [%f]Hz\n",
          scan.stamp,
          (unsigned int)scan.points.size(), 
          1.0 / scan.config.scan_time);
  fflush(stdout);
}
int scan(std::string, void(*)(LaserScan));

int main() {
  scan("/dev/ttyUSB0", handler);
  return 0;
}