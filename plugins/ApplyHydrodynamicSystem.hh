#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>


#include <Eigen/Dense>
#include <memory>
#include <string>

namespace blucy_plugins
{
  /// @brief A Gazebo system plugin that applies hydrodynamic forces and moments
  /// (added mass, linear + quadratic drag, Coriolis) to a link.
  class ApplyHydrodynamicSystem
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
  public:
    ApplyHydrodynamicSystem() = default;
    ~ApplyHydrodynamicSystem() override = default;

    /// \brief Called when the plugin is loaded and configured.
    void Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    /// \brief Called before each simulation update to apply hydrodynamic forces.
    void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                   ignition::gazebo::EntityComponentManager &ecm) override;


  private:
    // === SDF Parameters ===
    std::string modelName_;   ///< Optional model name
    std::string linkName_;    ///< Required link name

    Eigen::MatrixXd Ma_;       // Added mass 6x6
    Eigen::MatrixXd D_lin_;    // Linear damping 6x6
    Eigen::MatrixXd D_quad_;   // Quadratic damping 6x6

    // Water density (optional, default 1025 kg/m³)
    double waterDensity_{1025.0};

    // --- Cached entities ---
    ignition::gazebo::Entity modelEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity linkEntity_{ignition::gazebo::kNullEntity};

    
    Eigen::MatrixXd ComputeCoriolisMatrix(const Eigen::MatrixXd &M,
                                         const Eigen::Vector3d &v,
                                         const Eigen::Vector3d &w);

  };
}  // namespace blucy_plugins
