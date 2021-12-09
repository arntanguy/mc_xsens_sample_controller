#include "XsensSampleController_Initial.h"

#include "../XsensSampleController.h"

void XsensSampleController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void XsensSampleController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<XsensSampleController &>(ctl_);
}

bool XsensSampleController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<XsensSampleController &>(ctl_);
  output("OK");
  return true;
}

void XsensSampleController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<XsensSampleController &>(ctl_);
}

EXPORT_SINGLE_STATE("XsensSampleController_Initial", XsensSampleController_Initial)
