#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>

struct XsensRetargetting : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    std::map<std::string, std::string> bodyToSegment_;
    std::map<std::string, std::unique_ptr<mc_tasks::EndEffectorTask>> tasks_;
    double stiffness_ = 2;
    double weight_ = 1000;
};
