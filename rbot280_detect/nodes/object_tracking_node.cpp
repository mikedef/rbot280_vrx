// Node to overlay pointcloud returns over image

#include "rbot280_detect/object_tracking.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  ObjectTracking app;

  app.Run();

  return 0;
}
