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

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ServiceClientManager>;

  using ServiceClient = std::shared_ptr<ros::ServiceClient>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ServiceClientManager() noexcept;

  virtual ~ServiceClientManager() noexcept;

  //============================================================================
  // P U B L I C   M E T H O D S

  template <typename M>
  void RegisterService(const std::string &service_name);

  bool ShutdownService(const std::string &service_name);

  ServiceClient GetService(const std::string &service_name);

  template <typename T>
  bool SecureCall(T &service, const std::string &node);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle node_handler_;

  std::map<std::string, ServiceClient> services_;
};

#include "reacharm/lib/service_client_manager_inl.h"

#endif  // REACHARM_LIB_SERVICE_CLIENT_MANAGER_H_
