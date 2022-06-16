#ifndef _ROS_SERVICE_SetTrajectorySRV_h
#define _ROS_SERVICE_SetTrajectorySRV_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot
{

static const char SETTRAJECTORYSRV[] = "robot/SetTrajectorySRV";

  class SetTrajectorySRVRequest : public ros::Msg
  {
    public:
      typedef uint16_t _nb_nodes_type;
      _nb_nodes_type nb_nodes;
      uint32_t trajectory_x_length;
      typedef uint16_t _trajectory_x_type;
      _trajectory_x_type st_trajectory_x;
      _trajectory_x_type * trajectory_x;
      uint32_t trajectory_y_length;
      typedef uint16_t _trajectory_y_type;
      _trajectory_y_type st_trajectory_y;
      _trajectory_y_type * trajectory_y;
      typedef uint8_t _stop_motor_type;
      _stop_motor_type stop_motor;

    SetTrajectorySRVRequest():
      nb_nodes(0),
      trajectory_x_length(0), st_trajectory_x(), trajectory_x(nullptr),
      trajectory_y_length(0), st_trajectory_y(), trajectory_y(nullptr),
      stop_motor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nb_nodes >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nb_nodes >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nb_nodes);
      *(outbuffer + offset + 0) = (this->trajectory_x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_x_length);
      for( uint32_t i = 0; i < trajectory_x_length; i++){
      *(outbuffer + offset + 0) = (this->trajectory_x[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_x[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->trajectory_x[i]);
      }
      *(outbuffer + offset + 0) = (this->trajectory_y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_y_length);
      for( uint32_t i = 0; i < trajectory_y_length; i++){
      *(outbuffer + offset + 0) = (this->trajectory_y[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_y[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->trajectory_y[i]);
      }
      *(outbuffer + offset + 0) = (this->stop_motor >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop_motor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->nb_nodes =  ((uint16_t) (*(inbuffer + offset)));
      this->nb_nodes |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->nb_nodes);
      uint32_t trajectory_x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      trajectory_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      trajectory_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      trajectory_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->trajectory_x_length);
      if(trajectory_x_lengthT > trajectory_x_length)
        this->trajectory_x = (uint16_t*)realloc(this->trajectory_x, trajectory_x_lengthT * sizeof(uint16_t));
      trajectory_x_length = trajectory_x_lengthT;
      for( uint32_t i = 0; i < trajectory_x_length; i++){
      this->st_trajectory_x =  ((uint16_t) (*(inbuffer + offset)));
      this->st_trajectory_x |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_trajectory_x);
        memcpy( &(this->trajectory_x[i]), &(this->st_trajectory_x), sizeof(uint16_t));
      }
      uint32_t trajectory_y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      trajectory_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      trajectory_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      trajectory_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->trajectory_y_length);
      if(trajectory_y_lengthT > trajectory_y_length)
        this->trajectory_y = (uint16_t*)realloc(this->trajectory_y, trajectory_y_lengthT * sizeof(uint16_t));
      trajectory_y_length = trajectory_y_lengthT;
      for( uint32_t i = 0; i < trajectory_y_length; i++){
      this->st_trajectory_y =  ((uint16_t) (*(inbuffer + offset)));
      this->st_trajectory_y |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_trajectory_y);
        memcpy( &(this->trajectory_y[i]), &(this->st_trajectory_y), sizeof(uint16_t));
      }
      this->stop_motor =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->stop_motor);
     return offset;
    }

    virtual const char * getType() override { return SETTRAJECTORYSRV; };
    virtual const char * getMD5() override { return "35265fdab7debd6e9795d08f546c7ebd"; };

  };

  class SetTrajectorySRVResponse : public ros::Msg
  {
    public:

    SetTrajectorySRVResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return SETTRAJECTORYSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTrajectorySRV {
    public:
    typedef SetTrajectorySRVRequest Request;
    typedef SetTrajectorySRVResponse Response;
  };

}
#endif
