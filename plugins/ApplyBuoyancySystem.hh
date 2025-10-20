#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>
#include <memory>
#include <string>

namespace blucy_plugins
{
  class ApplyBuoyancySystem
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
  {
  public:
    ApplyBuoyancySystem() = default;
    ~ApplyBuoyancySystem() override = default;

    void Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                   ignition::gazebo::EntityComponentManager &ecm) override;

  private:
    std::string modelName;   // Read from SDF
    std::string linkName;    // Read from SDF
    ignition::math::Vector3d localForce;     // Read from SDF
    ignition::math::Vector3d localPoint;     // Read from SDF (COB)
    ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};
  };
}  // namespace blucy_plugins