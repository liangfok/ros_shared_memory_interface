/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Joshua James
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
 */

#include <ros/ros.h>
#include "shared_memory_interface/shared_memory_interface.hpp"
#include <signal.h>
#include <boost/python.hpp>

//BOOST_PYTHON_MODULE(hello_ext)
//{
//    using namespace boost::python;
//    def("greet", greet);
//}

using namespace boost::python;

void logPythonException()
{
  using namespace boost::python;

  PyObject *exc, *val, *tb;
  PyErr_Fetch(&exc, &val, &tb);
  PyErr_NormalizeException(&exc, &val, &tb);
  handle<> hexc(exc), hval(allow_null(val)), htb(allow_null(tb));
  if(!hval)
  {
    // With log4cxx::LoggerPtr logger previously declared and
    // initialized with log4cxx::Logger::getLogger("some.name")
//LOG4CXX_ERROR  (logger, std::string(extract<std::string>(str(hexc)));
  }
  else
  {
    object traceback(import("traceback"));
    object format_exception(traceback.attr("format_exception"));
    list formatted_list(format_exception(hexc, hval, htb));
    for(int count = 0; count < len(formatted_list); ++count)
      LOG4CXX_ERROR(logger, std::string(extract<std::string>(formatted_list[count].slice(0, -1))));
  }
}

void echo()
{
  std::string echo_string = "
  sys.stdout.write("%"+_str_plot_fields(data, 'field', self.field_filter)+'\n')
  sys.stdout.write(self.prefix+\
                    self.str_fn(data,
                                current_time=current_time, field_filter=self.field_filter, type_information=type_information) + \
                    self.suffix + '\n')
                    "

  try
  {
    Py_Initialize();

    object main_module((handle<>(borrowed(PyImport_AddModule("__main__")))));

    object main_namespace = main_module.attr("__dict__");

    handle<> ignored((PyRun_String("print \"Hello, World\"", Py_file_input, main_namespace.ptr(), main_namespace.ptr())));
  }
  catch(error_already_set )
  {
    PyErr_Print();
  }
}

int main(int argc, char **argv)
{
//  if(argc != 2)
//  {
//    std::cerr << "Target pelvis height must be specified!" << std::endl;
//  }

  ros::init(argc, argv, "test_locomotion");

  ros::NodeHandle nh;

//  actionlib::SimpleActionClient<rapid_locomotion_planner::ChangePelvisHeightAction> client(nh, "/change_pelvis_height", true);
//
//  ros::Duration(1.0).sleep();
//
//  // wait for action client to come up
//  while(!client.waitForServer(ros::Duration(1.0)) && ros::ok())
//  {
//    ROS_INFO("Waiting for the server to start");
//  }
//
//  ROS_INFO("Connected to action server.");
//
//  rapid_locomotion_planner::ChangePelvisHeightGoal goal;
//  goal.target_height = atof(argv[1]);
//
//  // Start the trajectory
//  client.sendGoal(goal);
//
//  client.waitForResult(ros::Duration(10));
//
//  actionlib::SimpleClientGoalState state = client.getState();
//  ROS_INFO("Action returned: %s", state.toString().c_str());



  return 0;
}
