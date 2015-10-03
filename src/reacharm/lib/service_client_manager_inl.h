/**
 * \file	service_client_manager_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#ifndef REACHARM_LIB_SERVICE_CLIENT_MANAGER_H_
#error This file may only be included from service_client_manager.h
#endif

#include <assert.h>
#include <algorithm>

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
inline ServiceClientManager::ServiceClientManager() noexcept : node_handler_(ros::NodeHandle()) {}

//------------------------------------------------------------------------------
//
inline ServiceClientManager::~ServiceClientManager() {
  for (auto &service : services_) {
    service.second.shutdown();
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename M>
inline void ServiceClientManager::RegisterService(const std::string &service_name) {
  auto result_advertise = node_handler_.serviceClient<M>(service_name);
  auto pair = std::pair<std::string, ros::ServiceClient>(service_name,
                                                         result_advertise);
  services_.insert(pair);
}

//------------------------------------------------------------------------------
//
inline bool ServiceClientManager::ShutdownService(const std::string &service_name) {
  for (auto &service : services_) {
    if (service.first == service_name) {
      service.second.shutdown();
      services_.erase(service.first);
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline ros::ServiceClient *const ServiceClientManager::GetService(
    const std::string &service_name) {
  for (auto &service : services_) {
    if (service.first == service_name) {
      return &(service.second);
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
template <typename T>
inline bool ServiceClientManager::SecureCall(const T &service,
                                      const std::string &node) {
  for (int i = 0; i < kConnectionAttempts; ++i) {
    if (services_.at(node).call(service)) {
      return true;
    }
  }
  return false;
}
