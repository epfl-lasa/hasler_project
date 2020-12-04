#ifndef _ROS_custom_msgs_FootOutputMsg_h
#define _ROS_custom_msgs_FootOutputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace custom_msgs
{

  class FootOutputMsg : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int16_t _id_type;
      _id_type id;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _phi_type;
      _phi_type phi;
      typedef float _theta_type;
      _theta_type theta;
      typedef float _psi_type;
      _psi_type psi;
      typedef float _Fx_d_type;
      _Fx_d_type Fx_d;
      typedef float _Fy_d_type;
      _Fy_d_type Fy_d;
      typedef float _Tphi_d_type;
      _Tphi_d_type Tphi_d;
      typedef float _Ttheta_d_type;
      _Ttheta_d_type Ttheta_d;
      typedef float _Tpsi_d_type;
      _Tpsi_d_type Tpsi_d;
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _wphi_type;
      _wphi_type wphi;
      typedef float _wtheta_type;
      _wtheta_type wtheta;
      typedef float _wpsi_type;
      _wpsi_type wpsi;
      typedef float _Fx_m_type;
      _Fx_m_type Fx_m;
      typedef float _Fy_m_type;
      _Fy_m_type Fy_m;
      typedef float _Tphi_m_type;
      _Tphi_m_type Tphi_m;
      typedef float _Ttheta_m_type;
      _Ttheta_m_type Ttheta_m;
      typedef float _Tpsi_m_type;
      _Tpsi_m_type Tpsi_m;
      typedef int16_t _state_type;
      _state_type state;

    FootOutputMsg():
      stamp(),
      id(0),
      x(0),
      y(0),
      phi(0),
      theta(0),
      psi(0),
      Fx_d(0),
      Fy_d(0),
      Tphi_d(0),
      Ttheta_d(0),
      Tpsi_d(0),
      vx(0),
      vy(0),
      wphi(0),
      wtheta(0),
      wpsi(0),
      Fx_m(0),
      Fy_m(0),
      Tphi_m(0),
      Ttheta_m(0),
      Tpsi_m(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.real = this->phi;
      *(outbuffer + offset + 0) = (u_phi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phi);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_psi;
      u_psi.real = this->psi;
      *(outbuffer + offset + 0) = (u_psi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_psi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_psi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_psi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->psi);
      union {
        float real;
        uint32_t base;
      } u_Fx_d;
      u_Fx_d.real = this->Fx_d;
      *(outbuffer + offset + 0) = (u_Fx_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fx_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fx_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fx_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fx_d);
      union {
        float real;
        uint32_t base;
      } u_Fy_d;
      u_Fy_d.real = this->Fy_d;
      *(outbuffer + offset + 0) = (u_Fy_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fy_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fy_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fy_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fy_d);
      union {
        float real;
        uint32_t base;
      } u_Tphi_d;
      u_Tphi_d.real = this->Tphi_d;
      *(outbuffer + offset + 0) = (u_Tphi_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tphi_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tphi_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tphi_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tphi_d);
      union {
        float real;
        uint32_t base;
      } u_Ttheta_d;
      u_Ttheta_d.real = this->Ttheta_d;
      *(outbuffer + offset + 0) = (u_Ttheta_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ttheta_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ttheta_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ttheta_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ttheta_d);
      union {
        float real;
        uint32_t base;
      } u_Tpsi_d;
      u_Tpsi_d.real = this->Tpsi_d;
      *(outbuffer + offset + 0) = (u_Tpsi_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tpsi_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tpsi_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tpsi_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tpsi_d);
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_wphi;
      u_wphi.real = this->wphi;
      *(outbuffer + offset + 0) = (u_wphi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wphi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wphi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wphi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wphi);
      union {
        float real;
        uint32_t base;
      } u_wtheta;
      u_wtheta.real = this->wtheta;
      *(outbuffer + offset + 0) = (u_wtheta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wtheta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wtheta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wtheta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wtheta);
      union {
        float real;
        uint32_t base;
      } u_wpsi;
      u_wpsi.real = this->wpsi;
      *(outbuffer + offset + 0) = (u_wpsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wpsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wpsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wpsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wpsi);
      union {
        float real;
        uint32_t base;
      } u_Fx_m;
      u_Fx_m.real = this->Fx_m;
      *(outbuffer + offset + 0) = (u_Fx_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fx_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fx_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fx_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fx_m);
      union {
        float real;
        uint32_t base;
      } u_Fy_m;
      u_Fy_m.real = this->Fy_m;
      *(outbuffer + offset + 0) = (u_Fy_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fy_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fy_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fy_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fy_m);
      union {
        float real;
        uint32_t base;
      } u_Tphi_m;
      u_Tphi_m.real = this->Tphi_m;
      *(outbuffer + offset + 0) = (u_Tphi_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tphi_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tphi_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tphi_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tphi_m);
      union {
        float real;
        uint32_t base;
      } u_Ttheta_m;
      u_Ttheta_m.real = this->Ttheta_m;
      *(outbuffer + offset + 0) = (u_Ttheta_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ttheta_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ttheta_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ttheta_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ttheta_m);
      union {
        float real;
        uint32_t base;
      } u_Tpsi_m;
      u_Tpsi_m.real = this->Tpsi_m;
      *(outbuffer + offset + 0) = (u_Tpsi_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tpsi_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tpsi_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tpsi_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tpsi_m);
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.base = 0;
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->phi = u_phi.real;
      offset += sizeof(this->phi);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_psi;
      u_psi.base = 0;
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->psi = u_psi.real;
      offset += sizeof(this->psi);
      union {
        float real;
        uint32_t base;
      } u_Fx_d;
      u_Fx_d.base = 0;
      u_Fx_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fx_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fx_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fx_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fx_d = u_Fx_d.real;
      offset += sizeof(this->Fx_d);
      union {
        float real;
        uint32_t base;
      } u_Fy_d;
      u_Fy_d.base = 0;
      u_Fy_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fy_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fy_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fy_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fy_d = u_Fy_d.real;
      offset += sizeof(this->Fy_d);
      union {
        float real;
        uint32_t base;
      } u_Tphi_d;
      u_Tphi_d.base = 0;
      u_Tphi_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tphi_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tphi_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tphi_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tphi_d = u_Tphi_d.real;
      offset += sizeof(this->Tphi_d);
      union {
        float real;
        uint32_t base;
      } u_Ttheta_d;
      u_Ttheta_d.base = 0;
      u_Ttheta_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ttheta_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ttheta_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ttheta_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Ttheta_d = u_Ttheta_d.real;
      offset += sizeof(this->Ttheta_d);
      union {
        float real;
        uint32_t base;
      } u_Tpsi_d;
      u_Tpsi_d.base = 0;
      u_Tpsi_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tpsi_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tpsi_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tpsi_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tpsi_d = u_Tpsi_d.real;
      offset += sizeof(this->Tpsi_d);
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_wphi;
      u_wphi.base = 0;
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wphi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wphi = u_wphi.real;
      offset += sizeof(this->wphi);
      union {
        float real;
        uint32_t base;
      } u_wtheta;
      u_wtheta.base = 0;
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wtheta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wtheta = u_wtheta.real;
      offset += sizeof(this->wtheta);
      union {
        float real;
        uint32_t base;
      } u_wpsi;
      u_wpsi.base = 0;
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wpsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wpsi = u_wpsi.real;
      offset += sizeof(this->wpsi);
      union {
        float real;
        uint32_t base;
      } u_Fx_m;
      u_Fx_m.base = 0;
      u_Fx_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fx_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fx_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fx_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fx_m = u_Fx_m.real;
      offset += sizeof(this->Fx_m);
      union {
        float real;
        uint32_t base;
      } u_Fy_m;
      u_Fy_m.base = 0;
      u_Fy_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fy_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fy_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fy_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fy_m = u_Fy_m.real;
      offset += sizeof(this->Fy_m);
      union {
        float real;
        uint32_t base;
      } u_Tphi_m;
      u_Tphi_m.base = 0;
      u_Tphi_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tphi_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tphi_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tphi_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tphi_m = u_Tphi_m.real;
      offset += sizeof(this->Tphi_m);
      union {
        float real;
        uint32_t base;
      } u_Ttheta_m;
      u_Ttheta_m.base = 0;
      u_Ttheta_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ttheta_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ttheta_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ttheta_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Ttheta_m = u_Ttheta_m.real;
      offset += sizeof(this->Ttheta_m);
      union {
        float real;
        uint32_t base;
      } u_Tpsi_m;
      u_Tpsi_m.base = 0;
      u_Tpsi_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tpsi_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tpsi_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tpsi_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tpsi_m = u_Tpsi_m.real;
      offset += sizeof(this->Tpsi_m);
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootOutputMsg"; };
    const char * getMD5(){ return "abcff2c619e990de9f64e1851406adcc"; };

  };

}
#endif