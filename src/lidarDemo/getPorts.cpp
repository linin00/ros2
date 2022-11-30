#include "CYdLidar.h"
#include <string>
#include <stdio.h>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

int main(int argc, char *argv[]) {
  // init system signal
  ydlidar::os_init();
  CYdLidar laser;
  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::string port;
  printf("num of ports: %ld\n", ports.size());
  for (auto item : ports) {
    printf("%s\t%s\n", item.first.c_str(), item.second.c_str());
  }
  return 0;
}