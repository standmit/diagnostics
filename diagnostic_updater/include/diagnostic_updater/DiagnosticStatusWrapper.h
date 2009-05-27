/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**
 * @author Blaise Gassend
 */

#ifndef DIAGNOSTICMESSAGEWRAPPER_HH
#define DIAGNOSTICMESSAGEWRAPPER_HH

#include <vector>
#include <string>
#include <sstream>
#include <stdarg.h>

#include "ros/node.h"

#include "robot_msgs/DiagnosticStatus.h"

namespace diagnostic_updater
{

class DiagnosticStatusWrapper : public robot_msgs::DiagnosticStatus
{
public:
  void summary(unsigned char lvl, const std::string s)
  {
    level = lvl;
    message = s;
  }

  template<class T>
  void adds(const std::string &key, const T &val)
  {
    std::stringstream ss;
    ss << val;
    std::string sval = ss.str();
    adds(key, sval);
  }
  
  void addsf(const std::string &key, const char *format, ...); // In practice format will always be a char *

  void addv(const std::string &key, double v)
  {
    robot_msgs::DiagnosticValue dv;
    dv.label = key;
    dv.value = v;
    values.push_back(dv);
  }
};

template<>
void DiagnosticStatusWrapper::adds<std::string>(const std::string &key, const std::string &s)
{
  robot_msgs::DiagnosticString ds;
  ds.label = key;
  ds.value = s;
  strings.push_back(ds);
}
  
// Need to place addsf after DiagnosticStatusWrapper::adds<std::string> or
// gcc complains that the specialization occurs after instatiation.
void DiagnosticStatusWrapper::addsf(const std::string &key, const char *format, ...) // In practice format will always be a char *
{
  va_list va;
  char buff[1000]; // @todo This could be done more elegantly.
  va_start(va, format);
  if (vsnprintf(buff, 1000, format, va) >= 1000)
    ROS_DEBUG("Really long string in DiagnosticStatusWrapper::addsf, it was truncated.");
  std::string value = std::string(buff);
  adds(key, value);
  va_end(va);
}


}
#endif