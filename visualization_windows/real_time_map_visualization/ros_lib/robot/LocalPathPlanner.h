#ifndef _ROS_SERVICE_LocalPathPlanner_h
#define _ROS_SERVICE_LocalPathPlanner_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot
{

static const char LOCALPATHPLANNER[] = "robot/LocalPathPlanner";

  class LocalPathPlannerRequest : public ros::Msg
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
      typedef double _heading_type;
      _heading_type heading;
      typedef uint8_t _stop_motor_type;
      _stop_motor_type stop_motor;

    LocalPathPlannerRequest():
      nb_nodes(0),
      trajectory_x_length(0), st_trajectory_x(), trajectory_x(nullptr),
      trajectory_y_length(0), st_trajectory_y(), trajectory_y(nullptr),
      heading(0),
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
      union {
        double real;
        uint64_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_heading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_heading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_heading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_heading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->heading);
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
      union {
        double real;
        uint64_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      this->stop_motor =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->stop_motor);
     return offset;
    }

    virtual const char * getType() override { return LOCALPATHPLANNER; };
    virtual const char * getMD5() override { return "fe43951bcf4c687d250115f5db0a2264"; };

  };

  class LocalPathPlannerResponse : public ros::Msg
  {
    public:
      float motor_vel[4];

    LocalPathPlannerResponse():
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

    virtual const char * getType() override { return LOCALPATHPLANNER; };
    virtual const char * getMD5() override { return "394ea2281e6ce52e3f72e252e5ce21dc"; };

  };

  class LocalPathPlanner {
    public:
    typedef LocalPathPlannerRequest Request;
    typedef LocalPathPlannerResponse Response;
  };

}
#endif
