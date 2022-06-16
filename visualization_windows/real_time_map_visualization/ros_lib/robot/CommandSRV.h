#ifndef _ROS_SERVICE_CommandSRV_h
#define _ROS_SERVICE_CommandSRV_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot
{

static const char COMMANDSRV[] = "robot/CommandSRV";

  class CommandSRVRequest : public ros::Msg
  {
    public:
      typedef uint8_t _dm_state_type;
      _dm_state_type dm_state;
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
      typedef float _arm_angle_type;
      _arm_angle_type arm_angle;
      typedef float _basket_angle_type;
      _basket_angle_type basket_angle;
      typedef bool _air_pump_type;
      _air_pump_type air_pump;
      uint8_t meas[7];

    CommandSRVRequest():
      dm_state(0),
      nb_nodes(0),
      trajectory_x_length(0), st_trajectory_x(), trajectory_x(nullptr),
      trajectory_y_length(0), st_trajectory_y(), trajectory_y(nullptr),
      stop_motor(0),
      arm_angle(0),
      basket_angle(0),
      air_pump(0),
      meas()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dm_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dm_state);
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
      union {
        float real;
        uint32_t base;
      } u_arm_angle;
      u_arm_angle.real = this->arm_angle;
      *(outbuffer + offset + 0) = (u_arm_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arm_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arm_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arm_angle);
      union {
        float real;
        uint32_t base;
      } u_basket_angle;
      u_basket_angle.real = this->basket_angle;
      *(outbuffer + offset + 0) = (u_basket_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_basket_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_basket_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_basket_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->basket_angle);
      union {
        bool real;
        uint8_t base;
      } u_air_pump;
      u_air_pump.real = this->air_pump;
      *(outbuffer + offset + 0) = (u_air_pump.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->air_pump);
      for( uint32_t i = 0; i < 7; i++){
      *(outbuffer + offset + 0) = (this->meas[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->meas[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->dm_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dm_state);
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
      union {
        float real;
        uint32_t base;
      } u_arm_angle;
      u_arm_angle.base = 0;
      u_arm_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arm_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arm_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arm_angle = u_arm_angle.real;
      offset += sizeof(this->arm_angle);
      union {
        float real;
        uint32_t base;
      } u_basket_angle;
      u_basket_angle.base = 0;
      u_basket_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_basket_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_basket_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_basket_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->basket_angle = u_basket_angle.real;
      offset += sizeof(this->basket_angle);
      union {
        bool real;
        uint8_t base;
      } u_air_pump;
      u_air_pump.base = 0;
      u_air_pump.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->air_pump = u_air_pump.real;
      offset += sizeof(this->air_pump);
      for( uint32_t i = 0; i < 7; i++){
      this->meas[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->meas[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return COMMANDSRV; };
    virtual const char * getMD5() override { return "a2ed7b0d144cf77650bc1de608920f6c"; };

  };

  class CommandSRVResponse : public ros::Msg
  {
    public:

    CommandSRVResponse()
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

    virtual const char * getType() override { return COMMANDSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CommandSRV {
    public:
    typedef CommandSRVRequest Request;
    typedef CommandSRVResponse Response;
  };

}
#endif
