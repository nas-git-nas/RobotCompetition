#ifndef _ROS_SERVICE_AddTwoInts_h
#define _ROS_SERVICE_AddTwoInts_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace global_path_planner
{

static const char ADDTWOINTS[] = "global_path_planner/AddTwoInts";

  class AddTwoIntsRequest : public ros::Msg
  {
    public:
      typedef uint16_t _nb_set_points_type;
      _nb_set_points_type nb_set_points;
      uint32_t set_points_x_length;
      typedef uint16_t _set_points_x_type;
      _set_points_x_type st_set_points_x;
      _set_points_x_type * set_points_x;

    AddTwoIntsRequest():
      nb_set_points(0),
      set_points_x_length(0), st_set_points_x(), set_points_x(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nb_set_points >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nb_set_points >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nb_set_points);
      *(outbuffer + offset + 0) = (this->set_points_x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->set_points_x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->set_points_x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->set_points_x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_points_x_length);
      for( uint32_t i = 0; i < set_points_x_length; i++){
      *(outbuffer + offset + 0) = (this->set_points_x[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->set_points_x[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->set_points_x[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->nb_set_points =  ((uint16_t) (*(inbuffer + offset)));
      this->nb_set_points |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->nb_set_points);
      uint32_t set_points_x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      set_points_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      set_points_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      set_points_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->set_points_x_length);
      if(set_points_x_lengthT > set_points_x_length)
        this->set_points_x = (uint16_t*)realloc(this->set_points_x, set_points_x_lengthT * sizeof(uint16_t));
      set_points_x_length = set_points_x_lengthT;
      for( uint32_t i = 0; i < set_points_x_length; i++){
      this->st_set_points_x =  ((uint16_t) (*(inbuffer + offset)));
      this->st_set_points_x |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_set_points_x);
        memcpy( &(this->set_points_x[i]), &(this->st_set_points_x), sizeof(uint16_t));
      }
     return offset;
    }

    virtual const char * getType() override { return ADDTWOINTS; };
    virtual const char * getMD5() override { return "dfe9b25eec1f33d573731b0ea41c9db2"; };

  };

  class AddTwoIntsResponse : public ros::Msg
  {
    public:
      float motor_vel[4];

    AddTwoIntsResponse():
      motor_vel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_motor_veli;
      u_motor_veli.real = this->motor_vel[i];
      *(outbuffer + offset + 0) = (u_motor_veli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_veli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_veli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_veli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_vel[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_motor_veli;
      u_motor_veli.base = 0;
      u_motor_veli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_veli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_veli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_veli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_vel[i] = u_motor_veli.real;
      offset += sizeof(this->motor_vel[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return ADDTWOINTS; };
    virtual const char * getMD5() override { return "394ea2281e6ce52e3f72e252e5ce21dc"; };

  };

  class AddTwoInts {
    public:
    typedef AddTwoIntsRequest Request;
    typedef AddTwoIntsResponse Response;
  };

}
#endif
