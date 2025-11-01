#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/double.pb.h>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace blucy_plugins
{
  /// @brief A Gazebo system plugin that applies thrust and torque to a link based on propeller dynamics.
  class ApplyThrusterSystem
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
  public:
    ApplyThrusterSystem() = default;
    ~ApplyThrusterSystem() override = default;

    // === Ignition plugin lifecycle methods ===
    void Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                   ignition::gazebo::EntityComponentManager &ecm) override;

  private:
    // === Thrust calculation utilities ===
    double ComputeVr(double Va, double rps);
    double ComputeBeta(double Va, double rps);
    double ComputeCt(double Beta);
    double ComputeCq(double Beta);

    // === Ocean current callback ===
    void OnOceanCurrentMsg(const ignition::msgs::Vector3d &_msg);

    // === RPM command callback (NEW) ===
    void OnRpmCmdMsg(const ignition::msgs::Double &_msg);

    // === Internal entity references ===
    ignition::gazebo::Entity modelEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity linkEntity_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity jointEntity_{ignition::gazebo::kNullEntity};

    // === Configuration parameters ===
    std::string linkName_;
    std::string jointName_;
    ignition::math::Vector3d thrusterDirection_{0, 0, 1};  // Default +Z

    // === Thruster physical parameters ===
    double D_{0.1};                 // Propeller diameter [m]
    double rho_{1025.0};            // Water density [kg/m³]
    double rpm_{0.0};               // Current commanded RPM
    double rpmMax_{3000.0};         // Max RPM
    double rpmMin_{-3000.0};        // Min RPM
    double rotationDirection_{1.0}; // +1 normal, -1 counter-rotating

    // === Coefficients (Fourier series terms) ===
    std::vector<double> CtCoeffs_;
    std::vector<double> CqCoeffs_;

    // === Ocean current state ===
    ignition::math::Vector3d oceanCurrent_{0, 0, 0};
    std::mutex mutex_;

    // === Ignition communication node ===
    ignition::transport::Node node_;
  };
}

