/**
 * \file	reacharm_server.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \author  Mathieu Benoit <mathben963@gmail.com>
 * \date	03/10/2015
 */

#include "reacharm/server/reacharm_server.h"

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ReachArmServer::ReachArmServer() noexcept {}

//------------------------------------------------------------------------------
//
ReachArmServer::~ReachArmServer() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ReachArmServer::OnSubjectNotify(Subject<> &subject) {
  // Handle the response from myo
}