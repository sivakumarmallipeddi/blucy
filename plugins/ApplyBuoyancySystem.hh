#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>

#include <memory>
#include <string>

namespace blucy_plugins
{
  /// @brief A Gazebo system plugin that applies a constant buoyancy force
  /// to a link at a specified point (center of buoyancy) in the link’s local frame.
  class ApplyBuoyancySystem
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
  public:
    ApplyBuoyancySystem() = default;
    ~ApplyBuoyancySystem() override = default;

    /// @brief Called when the plugin is loaded and configured.
    void Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    /// @brief Called before each simulation update to apply buoyancy forces.
    void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                   ignition::gazebo::EntityComponentManager &ecm) override;

  private:
    // === SDF Parameters ===
    std::string modelName_;                         ///< Model name (optional)
    std::string linkName_;                          ///< Link name (required)
    ignition::math::Vector3d localForce_{0, 0, 0};  ///< Buoyancy force (local frame)
    ignition::math::Vector3d localPoint_{0, 0, 0};  ///< Center of buoyancy (local frame)

    // === Cached entities ===
    ignition::gazebo::Entity modelEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity linkEntity_{ignition::gazebo::kNullEntity};
  };
}  // namespace blucy_plugins
