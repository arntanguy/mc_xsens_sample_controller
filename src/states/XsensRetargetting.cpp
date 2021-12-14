#include "XsensRetargetting.h"

#include "../XsensSampleController.h"

void XsensRetargetting::configure(const mc_rtc::Configuration & config)
{
  config("stiffness", stiffness_);
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
  
  auto robotConfig = static_cast<std::map<std::string, mc_rtc::Configuration>>(ctl.config()("Xsens")(ctl.robot().name()));
  for(const auto & bodyConfig : robotConfig)
  {
    const auto & bodyName = bodyConfig.first;
    const auto & bodyConf = bodyConfig.second;
    bodyConfigurations_[bodyName] = XsensBodyConfiguration{};
    auto & bodyC = bodyConfigurations_[bodyName];
    bodyC.bodyName = bodyName;
    bodyC.segmentName = static_cast<std::string>(bodyConf("segment"));
    bodyConf("offset", bodyC.offset);
    ctl.gui()->addElement({"Xsens", "Bodies", bodyName},
      mc_rtc::gui::ArrayInput("Offset translation [m]",
      [&bodyC]()
      {
        return bodyC.offset.translation();
      },
      [&bodyC](const Eigen::Vector3d & offset)
      {
        bodyC.offset.translation() = offset;
      }),
      mc_rtc::gui::ArrayInput("Offset rotation [rad]",
      [&bodyC]()
      {
        return mc_rbdyn::rpyFromMat(bodyC.offset.rotation());
      },
      [&bodyC](const Eigen::Vector3d & offset)
      {
        bodyC.offset.rotation() = mc_rbdyn::rpyToMat(offset);
      })
    );
  }

  // Initialize tasks
  for(auto & body: bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second.bodyName;
    if(ctl.robot().hasBody(bodyName))
    {
      auto task = std::unique_ptr<mc_tasks::EndEffectorTask>(new mc_tasks::EndEffectorTask(bodyName, ctl.robots(), ctl.robots().robotIndex(), stiffness_, weight_));
      task->reset();
      ctl.solver().addTask(task.get());
      tasks_[bodyName] = std::move(task); 
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
  for(const auto & body: bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second.segmentName;
    if(ctl.robot().hasBody(bodyName))
    {
      try
      {
        const auto segmentPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName); 
        tasks_[bodyName]->set_ef_pose(body.second.offset * segmentPose); 
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
