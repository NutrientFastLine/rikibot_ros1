#include "rikibot_pid/rikibot_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pid_configure");
  ros::NodeHandle nh;

  RikibotPID *rikibot_pid = new RikibotPID();

  dynamic_reconfigure::Server<rikibot_pid::rikibotPIDConfig> dr_srv;
  dynamic_reconfigure::Server<rikibot_pid::rikibotPIDConfig>::CallbackType cb;
  cb = boost::bind(&RikibotPID::configCallback, rikibot_pid, _1, _2);
  dr_srv.setCallback(cb);

  double p;
  double d;
  double i;
  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("p", p, 0.1);
  pnh.param("d", d, 0.10);
  pnh.param("i", i, 0.10);
  pnh.param("rate", rate, 1);

  ros::Publisher pub_message = nh.advertise<riki_msgs::PID>("pid", 10);

  ros::Rate r(rate);

  while (nh.ok())
  {
    rikibot_pid->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
