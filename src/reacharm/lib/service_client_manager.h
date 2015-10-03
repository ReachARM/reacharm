/**
 * \file	service_client_manager.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_SERVICE_CLIENT_MANAGER_H_
#define REACHARM_LIB_SERVICE_CLIENT_MANAGER_H_

#include <map>
#include <string>
#include <ros/ros.h>

class ServiceClientManager {
 public:
  static constexpr uint8_t kConnectionAttempts = 3;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ServiceClientManager() noexcept;

  virtual ~ServiceClientManager();

  //============================================================================
  // P U B L I C   M E T H O D S

  template <typename M>
  void RegisterService(const std::string &service_name);

  bool ShutdownService(const std::string &service_name);

  ros::ServiceClient *const GetService(const std::string &service_name);

  template <typename T>
  bool SecureCall(const T &service, const std::string &node);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle node_handler_;

  std::map<std::string, ros::ServiceClient> services_;
};

#include "reacharm/lib/service_client_manager_inl.h"

#endif  // REACHARM_LIB_SERVICE_CLIENT_MANAGER_H_
