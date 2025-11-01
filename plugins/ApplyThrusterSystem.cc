#include "ApplyThrusterSystem.hh"

#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

#include <cmath>
#include <mutex>

using namespace blucy_plugins;

 #define DEBUG_THRUSTER  // Uncomment to enable debug output

//////////////////////////////////////////////////
void ApplyThrusterSystem::Configure(const ignition::gazebo::Entity &entity,
                                    const std::shared_ptr<const sdf::Element> &sdf,
                                    ignition::gazebo::EntityComponentManager &ecm,
                                    ignition::gazebo::EventManager &)
{
  this->modelEntity_ = entity;

  // === Load SDF parameters ===
  if (sdf->HasElement("link_name"))
    this->linkName_ = sdf->Get<std::string>("link_name");

  if (sdf->HasElement("joint_name"))
    this->jointName_ = sdf->Get<std::string>("joint_name");

  if (sdf->HasElement("direction"))
    this->thrusterDirection_ = sdf->Get<ignition::math::Vector3d>("direction");

  if (sdf->HasElement("rotation_direction"))
    this->rotationDirection_ = sdf->Get<double>("rotation_direction");

  if (sdf->HasElement("D"))
    this->D_ = sdf->Get<double>("D");

  if (sdf->HasElement("rho"))
    this->rho_ = sdf->Get<double>("rho");

  if (sdf->HasElement("rpm_max"))
    this->rpmMax_ = sdf->Get<double>("rpm_max");

  if (sdf->HasElement("rpm_min"))
    this->rpmMin_ = sdf->Get<double>("rpm_min");

  // if (sdf->HasElement("rpm"))
  //   this->rpm_ = sdf->Get<double>("rpm");


  if (sdf->HasElement("rpm_topic"))
  {
    std::string rpmTopic = sdf->Get<std::string>("rpm_topic");
    this->node_.Subscribe(rpmTopic, &ApplyThrusterSystem::OnRpmCmdMsg, this);
    ignmsg << "[ApplyThrusterSystem] Subscribed to RPM topic: " << rpmTopic << std::endl;
  }

  // === Load hardcoded propeller coefficients ===
  this->CtCoeffs_ = {
      -0.0920602798812772, 0.201806874333569, 0.138380839202652, 0.0567666866895539,
      0.000552618761341551, -0.01324647804393, -0.00118734941842702, 0.0100611554135105,
      0.00491891559601254, -0.00241436893087301, 0.00599285423586754, 0.0105859809628183,
      -0.00893708793137409, 0.00445881600429356, -0.00204943289497132, 0.00714124339549381,
      -0.00617021618361912, 0.00197036262632045, 0.000568146477830656, -0.00283607997847183,
      -4.51226202969564e-05, -0.00388799191210186, -0.995959409668625, 0.0550457784959115,
      0.142521569617051, -0.0289822737837091, 0.0888594262091123, -0.00866054938526347,
      -0.0241473541871448, -0.0167365110928257, 0.000587180703995022, -0.00271681739133361,
      -0.00781590922311246, -0.00369872938629403, -0.00756684881978166, -0.00256741965711828,
      -0.00508364303052057, -0.000449218664901781, 0.000134269469342103, -0.00337976932886412,
      -0.00365705493942719, -0.00116185265331361, -0.00320905327620034};

  this->CqCoeffs_ = {
      0.00551495097295745, 0.0428191886206060, -0.00453554947415678, 0.00464805919333817,
      -0.00192652806133915, -0.00308513309618565, -0.00029857765185542, -0.00245483115669336,
      9.84837573130861e-05, -0.000968516905710252, 0.000985468013751284, -0.00138867595426838,
      -0.000115002775338861, 0.000320868488118498, 0.000576066217427196, 0.00153777235509347,
      -0.000506495148080680, 0.000957161807167533, 0.000690446189554404, 1.14839323241813e-05,
      0.000114791862679037, -0.000146515941896736, -0.132030861522271, 0.00603567471128339,
      0.0114823728390140, -0.00223487893406297, 0.0164108752846150, 0.000890739464637819,
      -0.00512016361291717, -0.00166617563995746, 0.00117258197445382, 0.00118159843944044,
      0.00342105219319382, 1.99311320564195e-05, 0.000960855839266091, 0.000224063623789609,
      0.000104667563776210, 0.000211731171392085, 0.000383063372359297, -0.00047074180900633,
      -0.000661250557548647, -0.000407227834974763, -0.000172062706413526};

  // === Find the link and joint entities ===
  ignition::gazebo::Model model(this->modelEntity_);
  this->linkEntity_ = model.LinkByName(ecm, this->linkName_);

  if (this->linkEntity_ == ignition::gazebo::kNullEntity)
  {
    ignerr << "[ApplyThrusterSystem] Could not find link [" << this->linkName_ << "]\n";
    return;
  }

  if (!this->jointName_.empty())
  {
    this->jointEntity_ = model.JointByName(ecm, this->jointName_);
    if (this->jointEntity_ == ignition::gazebo::kNullEntity)
      ignwarn << "[ApplyThrusterSystem] Could not find joint [" << this->jointName_ << "]\n";
  }

  // Enable velocity tracking
  ignition::gazebo::Link link(this->linkEntity_);
  link.EnableVelocityChecks(ecm);

  // === Subscribe to ocean current topic ===
  this->node_.Subscribe("/ocean_current", &ApplyThrusterSystem::OnOceanCurrentMsg, this);
}

//////////////////////////////////////////////////
// ===  Ocean current Callback ===
//////////////////////////////////////////////////
void ApplyThrusterSystem::OnOceanCurrentMsg(const ignition::msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->oceanCurrent_.Set(_msg.x(), _msg.y(), _msg.z());
}

//////////////////////////////////////////////////
// ===  RPM Command Callback ===
//////////////////////////////////////////////////
void ApplyThrusterSystem::OnRpmCmdMsg(const ignition::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->rpm_ = _msg.data();
}

//////////////////////////////////////////////////
// === Utility Computation Methods ===
//////////////////////////////////////////////////
double ApplyThrusterSystem::ComputeVr(double Va, double rps)
{
  return std::sqrt(Va * Va + std::pow(0.7 * M_PI * rps * this->D_, 2));
}

double ApplyThrusterSystem::ComputeBeta(double Va, double rps)
{
  return std::atan2(Va, 0.7 * M_PI * rps * this->D_) * 180.0 / M_PI;
}

double ApplyThrusterSystem::ComputeCt(double Beta)
{
  const int n = (this->CtCoeffs_.size() - 1) / 2;
  double sum = this->CtCoeffs_[0];
  for (int i = 0; i < n; ++i)
  {
    sum += this->CtCoeffs_[1 + i] * std::cos(2 * M_PI / 360 * (i + 1) * Beta)
          + this->CtCoeffs_[1 + n + i] * std::sin(2 * M_PI / 360 * (i + 1) * Beta);
  }
  return sum;
}

double ApplyThrusterSystem::ComputeCq(double Beta)
{
  const int n = (this->CqCoeffs_.size() - 1) / 2;
  double sum = this->CqCoeffs_[0];
  for (int i = 0; i < n; ++i)
  {
    sum += this->CqCoeffs_[1 + i] * std::cos(2 * M_PI / 360 * (i + 1) * Beta)
          + this->CqCoeffs_[1 + n + i] * std::sin(2 * M_PI / 360 * (i + 1) * Beta);
  }
  return sum;
}

//////////////////////////////////////////////////
// === Simulation Update Loop ===
//////////////////////////////////////////////////
void ApplyThrusterSystem::PreUpdate(const ignition::gazebo::UpdateInfo &,
                                    ignition::gazebo::EntityComponentManager &ecm)
{
  if (this->linkEntity_ == ignition::gazebo::kNullEntity)
    return;

  auto lvComp = ecm.Component<ignition::gazebo::components::LinearVelocity>(this->linkEntity_);
  if (!lvComp)
    return;

  ignition::math::Pose3d pose = ignition::gazebo::worldPose(this->linkEntity_, ecm);
  ignition::math::Vector3d vLinear = lvComp->Data();

  // Apply ocean current correction (in local frame)
  ignition::math::Vector3d currentWorld;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    currentWorld = this->oceanCurrent_;
  }

  ignition::math::Vector3d currentLocal = pose.Rot().RotateVectorReverse(currentWorld);
  ignition::math::Vector3d vLocal = pose.Rot().RotateVectorReverse(vLinear);
  ignition::math::Vector3d relVelocity = vLocal - currentLocal;

  const double Va = relVelocity.Dot(this->thrusterDirection_);
  this->rpm_ = std::clamp(this->rpm_, this->rpmMin_, this->rpmMax_);

  const double rps = this->rotationDirection_ * this->rpm_ / 60.0;
  const double Vr = ComputeVr(Va, rps);
  const double Beta = ComputeBeta(Va, rps);
  const double Ct = ComputeCt(Beta);
  const double Cq = ComputeCq(Beta);

  const double A0 = M_PI * this->D_ * this->D_ / 4.0;
  const double T = 0.5 * this->rho_ * A0 * Vr * Vr * Ct;
  const double Q = 0.5 * this->rho_ * A0 * this->D_ * Vr * Vr * Cq;

  ignition::math::Vector3d dirWorld = pose.Rot().RotateVector(this->thrusterDirection_);

#ifdef DEBUG_THRUSTER
  ignmsg << "[THRUSTER DEBUG] link=" << this->linkName_
         << " Va=" << Va << " T=" << T << " dir=" << dirWorld << "rpm"<< this->rpm_<<std::endl;
#endif

  // === Apply thrust as world wrench ===
  ignition::gazebo::Link link(this->linkEntity_);
  link.AddWorldWrench(ecm, dirWorld * T, ignition::math::Vector3d::Zero);

  // === Apply propeller torque to joint (if available) ===
  if (this->jointEntity_ != ignition::gazebo::kNullEntity)
  {
    ignition::gazebo::Joint joint(this->jointEntity_);
    auto axisComp = ecm.Component<ignition::gazebo::components::JointAxis>(this->jointEntity_);

    if (axisComp)
    {
      ignition::math::Vector3d localAxis = axisComp->Data().Xyz();
      ignition::math::Vector3d worldAxis = pose.Rot().RotateVector(localAxis);
      const double torqueScalar = Q * this->rotationDirection_;

#ifdef DEBUG_THRUSTER
      ignmsg << "[THRUSTER DEBUG] Applying joint torque to " << this->jointName_
             << " torque=" << torqueScalar << " axis=" << worldAxis << std::endl;
#endif

      joint.SetForce(ecm, {torqueScalar});
    }
  }
}

IGNITION_ADD_PLUGIN(
  blucy_plugins::ApplyThrusterSystem,
  ignition::gazebo::System,
  blucy_plugins::ApplyThrusterSystem::ISystemConfigure,
  blucy_plugins::ApplyThrusterSystem::ISystemPreUpdate)
