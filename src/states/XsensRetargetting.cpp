#include "XsensRetargetting.h"

#include "../XsensSampleController.h"

void XsensRetargetting::configure(const mc_rtc::Configuration & config)
{
}

void XsensRetargetting::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<XsensSampleController &>(ctl_);
  if(!ctl.config()("Xsens").has(ctl.robot().name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot {} not supported (missing Xsens->{} configuration)", ctl.robot().name(), name(), ctl.robot().name());
  }
  if(!ctl.datastore().has("XsensPlugin"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] This state requires the XsensPlugin", name());
  }
  bodyToSegment_ = ctl.config()("Xsens")(ctl.robot().name());

  for(const auto & body: bodyToSegment_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second;
    mc_rtc::log::info("body: {}, segment: {}", bodyName, segmentName);
    if(ctl.robot().hasBody(bodyName))
    {
      tasks_[bodyName] = std::unique_ptr<mc_tasks::EndEffectorTask>(new mc_tasks::EndEffectorTask(bodyName, ctl.robots(), ctl.robots().robotIndex(), stiffness_, weight_));
      ctl.solver().addTask(tasks_[bodyName].get());
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", bodyName);
    }
  }
  run(ctl);
}

bool XsensRetargetting::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<XsensSampleController &>(ctl_);
  for(const auto & body: bodyToSegment_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second;
    if(ctl.robot().hasBody(bodyName))
    {
      try
      {
        const auto & segmentPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName); 
        tasks_[bodyName]->set_ef_pose(segmentPose); 
      }
      catch(...)
      {
        mc_rtc::log::error("[{}] No pose for segment {}", name(), segmentName);
      }
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", name(), bodyName);
    }
  }
  output("OK");
  return true;
}

void XsensRetargetting::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<XsensSampleController &>(ctl_);
  for(const auto & task : tasks_)
  {
    ctl.solver().removeTask(task.second.get());
  }
}

EXPORT_SINGLE_STATE("XsensRetargetting", XsensRetargetting)
