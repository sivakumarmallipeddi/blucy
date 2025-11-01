#include "ApplyBuoyancySystem.hh"

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

using namespace blucy_plugins;
using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
void ApplyBuoyancySystem::Configure(const Entity &entity,
                                    const std::shared_ptr<const sdf::Element> &sdf,
                                    EntityComponentManager &ecm,
                                    EventManager &)
{
  this->modelEntity_ = entity;

  // === Read SDF parameters ===
  if (sdf->HasElement("model_name"))
  {
    this->modelName_ = sdf->Get<std::string>("model_name");
  }
  else
  {
    ignwarn << "[ApplyBuoyancySystem] <model_name> not specified. Using default model entity.\n";
  }

  if (sdf->HasElement("link_name"))
  {
    this->linkName_ = sdf->Get<std::string>("link_name");
  }
  else
  {
    ignerr << "[ApplyBuoyancySystem] <link_name> must be specified in plugin.\n";
    return;
  }

  if (sdf->HasElement("force"))
  {
    this->localForce_ = sdf->Get<ignition::math::Vector3d>("force");
  }

  if (sdf->HasElement("center_of_buoyancy"))
  {
    this->localPoint_ = sdf->Get<ignition::math::Vector3d>("center_of_buoyancy");
  }

  // === Find link entity ===
  Model model(this->modelEntity_);
  this->linkEntity_ = model.LinkByName(ecm, this->linkName_);

  if (this->linkEntity_ == kNullEntity)
  {
    ignwarn << "[ApplyBuoyancySystem] Link [" << this->linkName_
            << "] not found in model [" << this->modelName_ << "]\n";
    return;
  }

  // Enable velocity checks to ensure components exist (important for applying forces)
  Link link(this->linkEntity_);
  link.EnableVelocityChecks(ecm);

  // === Configuration summary ===
  ignmsg << "[ApplyBuoyancySystem] Configured successfully:\n"
         << "  Model: " << (this->modelName_.empty() ? "<unknown>" : this->modelName_) << "\n"
         << "  Link: " << this->linkName_ << "\n"
         << "  Local Force: " << this->localForce_ << " [N]\n"
         << "  Center of Buoyancy: " << this->localPoint_ << " [m]\n";
}

//////////////////////////////////////////////////
void ApplyBuoyancySystem::PreUpdate(const UpdateInfo &info,
                                    EntityComponentManager &ecm)
{
  // Skip if simulation is paused or link invalid
  if (info.paused || this->linkEntity_ == kNullEntity)
    return;

  Link link(this->linkEntity_);
  if (!link.Valid(ecm))
    return;

  // === Retrieve world pose of link ===
  auto poseComp = ecm.Component<components::Pose>(this->linkEntity_);
  if (!poseComp)
    return;

  const ignition::math::Pose3d &linkPose = poseComp->Data();

  // === Convert buoyancy force from local link frame to world frame ===
  ignition::math::Vector3d forceWorld = linkPose.Rot().RotateVector(this->localForce_);

  // === Apply the buoyancy force at the specified local offset ===
  link.AddWorldForce(ecm, forceWorld, this->localPoint_);
}

//////////////////////////////////////////////////
// === Plugin registration ===
IGNITION_ADD_PLUGIN(
  blucy_plugins::ApplyBuoyancySystem,
  ignition::gazebo::System,
  blucy_plugins::ApplyBuoyancySystem::ISystemConfigure,
  blucy_plugins::ApplyBuoyancySystem::ISystemPreUpdate)
