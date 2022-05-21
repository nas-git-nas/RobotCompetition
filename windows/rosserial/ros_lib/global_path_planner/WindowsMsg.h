#ifndef _ROS_global_path_planner_WindowsMsg_h
#define _ROS_global_path_planner_WindowsMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

namespace global_path_planner
{

  class WindowsMsg : public ros::Msg
  {
    public:
      typedef std_msgs::Float32 _var1_type;
      _var1_type var1;
      typedef std_msgs::Int32 _var2_type;
      _var2_type var2;

    WindowsMsg():
      var1(),
      var2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->var1.serialize(outbuffer + offset);
      offset += this->var2.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->var1.deserialize(inbuffer + offset);
      offset += this->var2.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "global_path_planner/WindowsMsg"; };
    virtual const char * getMD5() override { return "348eb56e7b29715f3bc3e110e2245551"; };

  };

}
#endif
