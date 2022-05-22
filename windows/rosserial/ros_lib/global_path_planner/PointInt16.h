#ifndef _ROS_global_path_planner_PointInt16_h
#define _ROS_global_path_planner_PointInt16_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int16.h"

namespace global_path_planner
{

  class PointInt16 : public ros::Msg
  {
    public:
      typedef std_msgs::Int16 _x_type;
      _x_type x;
      typedef std_msgs::Int16 _y_type;
      _y_type y;

    PointInt16():
      x(),
      y()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->x.serialize(outbuffer + offset);
      offset += this->y.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->x.deserialize(inbuffer + offset);
      offset += this->y.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "global_path_planner/PointInt16"; };
    virtual const char * getMD5() override { return "433d102530e92c15cb4f0c173e2dfceb"; };

  };

}
#endif
