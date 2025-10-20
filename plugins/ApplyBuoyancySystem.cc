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

/////////////////////////////////////////////////
void ApplyBuoyancySystem::Configure(const Entity &entity,
                                    const std::shared_ptr<const sdf::Element> &sdf,
                                    EntityComponentManager &ecm,
                                    EventManager &)
{
  this->modelEntity = entity;

  // === Read parameters from URDF/SDF ===
  if (sdf->HasElement("model_name"))
    this->modelName = sdf->Get<std::string>("model_name");
  else
    ignwarn << "[ApplyBuoyancySystem] <model_name> not specified, using default model entity.\n";

  if (sdf->HasElement("link_name"))
    this->linkName = sdf->Get<std::string>("link_name");
  else
  {
    ignerr << "[ApplyBuoyancySystem] <link_name> must be specified in plugin.\n";
    return;
  }

  if (sdf->HasElement("force"))
  {
    this->localForce = sdf->Get<ignition::math::Vector3d>("force");
    ignmsg << "[ApplyBuoyancySystem]  Force (local): " << this->localForce << " N\n";
  }
  else
    this->localForce = ignition::math::Vector3d(0, 0, 0);

  if (sdf->HasElement("center_of_buoyancy"))
  {
    this->localPoint = sdf->Get<ignition::math::Vector3d>("center_of_buoyancy");
    ignmsg << "[ApplyBuoyancySystem]  Center of Buoyancy (local): " << this->localPoint << " m\n";
  }
  else
    this->localPoint = ignition::math::Vector3d(0, 0, 0);

  // === Find link entity ===
  Model model(this->modelEntity);
  this->linkEntity = model.LinkByName(ecm, this->linkName);

  if (this->linkEntity == kNullEntity)
  {
    ignwarn << "[ApplyBuoyancySystem] Link [" << this->linkName
            << "] not found in model [" << this->modelName << "]\n";
    return;
  }

  // === Enable velocity checks to create required components ===
  Link link(this->linkEntity);
  link.EnableVelocityChecks(ecm);  // <---- ESSENTIAL LINE!

  ignmsg << "[ApplyBuoyancySystem] Configured for model [" << this->modelName
         << "], link [" << this->linkName << "]\n";
  ignmsg << "  Force: " << this->localForce << " N\n";
  ignmsg << "  Center of Buoyancy (local): " << this->localPoint << " m\n";
}

/////////////////////////////////////////////////
void ApplyBuoyancySystem::PreUpdate(const UpdateInfo &info,
                                    EntityComponentManager &ecm)
{
  // Skip if simulation is paused or link invalid
  if (info.paused || this->linkEntity == kNullEntity)
    return;

  Link link(this->linkEntity);
  if (!link.Valid(ecm))
    return;

  // Get link world pose
  auto poseComp = ecm.Component<components::Pose>(this->linkEntity);
  if (!poseComp)
    return;

  const ignition::math::Pose3d &linkPose = poseComp->Data();

  // Convert buoyancy force from link frame to world frame
  ignition::math::Vector3d forceWorld = linkPose.Rot().RotateVector(this->localForce);

  // Apply the force at the offset relative to the link (link frame)
  link.AddWorldForce(ecm, forceWorld, this->localPoint);
}

/////////////////////////////////////////////////
// Plugin registration
IGNITION_ADD_PLUGIN(
  blucy_plugins::ApplyBuoyancySystem,
  ignition::gazebo::System,
  blucy_plugins::ApplyBuoyancySystem::ISystemConfigure,
  blucy_plugins::ApplyBuoyancySystem::ISystemPreUpdate)