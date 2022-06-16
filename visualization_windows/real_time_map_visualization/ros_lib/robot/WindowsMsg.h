#ifndef _ROS_robot_WindowsMsg_h
#define _ROS_robot_WindowsMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot
{

  class WindowsMsg : public ros::Msg
  {
    public:
      typedef uint16_t _nb_nodes_type;
      _nb_nodes_type nb_nodes;
      uint32_t polygons_length;
      typedef uint16_t _polygons_type;
      _polygons_type st_polygons;
      _polygons_type * polygons;
      uint32_t nodes_x_length;
      typedef uint16_t _nodes_x_type;
      _nodes_x_type st_nodes_x;
      _nodes_x_type * nodes_x;
      uint32_t nodes_y_length;
      typedef uint16_t _nodes_y_type;
      _nodes_y_type st_nodes_y;
      _nodes_y_type * nodes_y;
      typedef uint16_t _nb_path_nodes_type;
      _nb_path_nodes_type nb_path_nodes;
      uint32_t path_length;
      typedef uint16_t _path_type;
      _path_type st_path;
      _path_type * path;
      typedef float _heading_type;
      _heading_type heading;
      typedef uint16_t _bottle_x_type;
      _bottle_x_type bottle_x;
      typedef uint16_t _bottle_y_type;
      _bottle_y_type bottle_y;
      typedef uint16_t _bottle_nb_meas_type;
      _bottle_nb_meas_type bottle_nb_meas;
      typedef uint16_t _state_type;
      _state_type state;

    WindowsMsg():
      nb_nodes(0),
      polygons_length(0), st_polygons(), polygons(nullptr),
      nodes_x_length(0), st_nodes_x(), nodes_x(nullptr),
      nodes_y_length(0), st_nodes_y(), nodes_y(nullptr),
      nb_path_nodes(0),
      path_length(0), st_path(), path(nullptr),
      heading(0),
      bottle_x(0),
      bottle_y(0),
      bottle_nb_meas(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nb_nodes >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nb_nodes >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nb_nodes);
      *(outbuffer + offset + 0) = (this->polygons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygons_length);
      for( uint32_t i = 0; i < polygons_length; i++){
      *(outbuffer + offset + 0) = (this->polygons[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygons[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->polygons[i]);
      }
      *(outbuffer + offset + 0) = (this->nodes_x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_x_length);
      for( uint32_t i = 0; i < nodes_x_length; i++){
      *(outbuffer + offset + 0) = (this->nodes_x[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_x[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nodes_x[i]);
      }
      *(outbuffer + offset + 0) = (this->nodes_y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_y_length);
      for( uint32_t i = 0; i < nodes_y_length; i++){
      *(outbuffer + offset + 0) = (this->nodes_y[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_y[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nodes_y[i]);
      }
      *(outbuffer + offset + 0) = (this->nb_path_nodes >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nb_path_nodes >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nb_path_nodes);
      *(outbuffer + offset + 0) = (this->path_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_length);
      for( uint32_t i = 0; i < path_length; i++){
      *(outbuffer + offset + 0) = (this->path[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->path[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      *(outbuffer + offset + 0) = (this->bottle_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bottle_x >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bottle_x);
      *(outbuffer + offset + 0) = (this->bottle_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bottle_y >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bottle_y);
      *(outbuffer + offset + 0) = (this->bottle_nb_meas >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bottle_nb_meas >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bottle_nb_meas);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->nb_nodes =  ((uint16_t) (*(inbuffer + offset)));
      this->nb_nodes |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->nb_nodes);
      uint32_t polygons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygons_length);
      if(polygons_lengthT > polygons_length)
        this->polygons = (uint16_t*)realloc(this->polygons, polygons_lengthT * sizeof(uint16_t));
      polygons_length = polygons_lengthT;
      for( uint32_t i = 0; i < polygons_length; i++){
      this->st_polygons =  ((uint16_t) (*(inbuffer + offset)));
      this->st_polygons |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_polygons);
        memcpy( &(this->polygons[i]), &(this->st_polygons), sizeof(uint16_t));
      }
      uint32_t nodes_x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_x_length);
      if(nodes_x_lengthT > nodes_x_length)
        this->nodes_x = (uint16_t*)realloc(this->nodes_x, nodes_x_lengthT * sizeof(uint16_t));
      nodes_x_length = nodes_x_lengthT;
      for( uint32_t i = 0; i < nodes_x_length; i++){
      this->st_nodes_x =  ((uint16_t) (*(inbuffer + offset)));
      this->st_nodes_x |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_nodes_x);
        memcpy( &(this->nodes_x[i]), &(this->st_nodes_x), sizeof(uint16_t));
      }
      uint32_t nodes_y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_y_length);
      if(nodes_y_lengthT > nodes_y_length)
        this->nodes_y = (uint16_t*)realloc(this->nodes_y, nodes_y_lengthT * sizeof(uint16_t));
      nodes_y_length = nodes_y_lengthT;
      for( uint32_t i = 0; i < nodes_y_length; i++){
      this->st_nodes_y =  ((uint16_t) (*(inbuffer + offset)));
      this->st_nodes_y |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_nodes_y);
        memcpy( &(this->nodes_y[i]), &(this->st_nodes_y), sizeof(uint16_t));
      }
      this->nb_path_nodes =  ((uint16_t) (*(inbuffer + offset)));
      this->nb_path_nodes |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->nb_path_nodes);
      uint32_t path_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_length);
      if(path_lengthT > path_length)
        this->path = (uint16_t*)realloc(this->path, path_lengthT * sizeof(uint16_t));
      path_length = path_lengthT;
      for( uint32_t i = 0; i < path_length; i++){
      this->st_path =  ((uint16_t) (*(inbuffer + offset)));
      this->st_path |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_path);
        memcpy( &(this->path[i]), &(this->st_path), sizeof(uint16_t));
      }
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      this->bottle_x =  ((uint16_t) (*(inbuffer + offset)));
      this->bottle_x |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->bottle_x);
      this->bottle_y =  ((uint16_t) (*(inbuffer + offset)));
      this->bottle_y |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->bottle_y);
      this->bottle_nb_meas =  ((uint16_t) (*(inbuffer + offset)));
      this->bottle_nb_meas |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->bottle_nb_meas);
      this->state =  ((uint16_t) (*(inbuffer + offset)));
      this->state |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return "robot/WindowsMsg"; };
    virtual const char * getMD5() override { return "31d83d5194c04b2dbc0f59eb93cfe0d7"; };

  };

}
#endif
