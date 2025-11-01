#include "ApplyHydrodynamicSystem.hh"
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include <Eigen/Dense>
#include <sstream>

using namespace blucy_plugins;
using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
void ApplyHydrodynamicSystem::Configure(const Entity &entity,
                                        const std::shared_ptr<const sdf::Element> &sdf,
                                        EntityComponentManager &ecm,
                                        EventManager &)
{
    // Store model entity
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


    // Water density (default 1025 kg/m³)
    this->waterDensity_ = sdf->Get<double>("water_density", 1025.0).first;

    // --- Added mass matrix
    this->Ma_ = Eigen::MatrixXd::Zero(6, 6);
    if (sdf->HasElement("added_mass"))
    {
        auto text = sdf->Get<std::string>("added_mass");
        std::stringstream ss(text);
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
            {
                double val = 0.0;
                ss >> val;
                this->Ma_(i,j) = val;
            }
    }

    // --- Linear damping
    this->D_lin_ = Eigen::MatrixXd::Zero(6, 6);
    if (sdf->HasElement("linear_damping"))
    {
        auto text = sdf->Get<std::string>("linear_damping");
        std::stringstream ss(text);
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
            {
                double val = 0.0;
                ss >> val;
                this->D_lin_(i,j) = val;
            }
    }

    // --- Quadratic damping
    this->D_quad_ = Eigen::MatrixXd::Zero(6, 6);
    if (sdf->HasElement("quadratic_damping"))
    {
        auto text = sdf->Get<std::string>("quadratic_damping");
        std::stringstream ss(text);
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
            {
                double val = 0.0;
                ss >> val;
                this->D_quad_(i,j) = val;
            }
    }

    // --- Get the link entity from model
    Model model(this->modelEntity_);
    if (!model.Valid(ecm))
    {
        ignerr << "[ApplyHydrodynamicSystem] Invalid model entity!\n";
        return;
    }

    this->linkEntity_ = model.LinkByName(ecm, this->linkName_);
   
    if (this->linkEntity_ == kNullEntity)
    {
        ignwarn << "[ApplyHydrodynamicSystem] Link [" << this->linkName_
                 << "] not found in model [" << this->modelName_ << "]\n";
        return;
    }

    // --- Enable velocity tracking for the link
    ignition::gazebo::Link link(this->linkEntity_);
    link.EnableVelocityChecks(ecm);
    link.EnableAccelerationChecks(ecm);

    ignmsg << "[ApplyHydrodynamicSystem] Configured successfully:\n"
           << "  Model: " << (this->modelName_.empty() ? "<unknown>" : this->modelName_) << "\n"
           << "  Link: " << this->linkName_ << "\n"
           << "  Water Density: " << this->waterDensity_ << " kg/m^3\n";

}

///////////helper function //////////////////////
Eigen::MatrixXd ApplyHydrodynamicSystem::ComputeCoriolisMatrix(const Eigen::MatrixXd &M, 
                                                               const Eigen::Vector3d &v,
                                                               const Eigen::Vector3d &w)
{
    // Symmetrize inertia matrix (same as 0.5*(M+M'))
    Eigen::MatrixXd Msym = 0.5 * (M + M.transpose());

    // Split into 3x3 submatrices
    Eigen::Matrix3d M11 = Msym.block<3,3>(0,0);
    Eigen::Matrix3d M12 = Msym.block<3,3>(0,3);
    Eigen::Matrix3d M21 = Msym.block<3,3>(3,0);
    Eigen::Matrix3d M22 = Msym.block<3,3>(3,3);

    // Compute intermediate terms
    Eigen::Vector3d nu1_dot = M11 * v + M12 * w;
    Eigen::Vector3d nu2_dot = M21 * v + M22 * w;

    // Helper: skew-symmetric matrix
    auto Smtrx = [](const Eigen::Vector3d &x) {
        Eigen::Matrix3d S;
        S <<  0,    -x.z(),  x.y(),
              x.z(),   0,   -x.x(),
             -x.y(),  x.x(),   0;
        return S;
    };

    // Construct 6x6 Coriolis matrix
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6,6);
    C.block<3,3>(0,3) = -Smtrx(nu1_dot);
    C.block<3,3>(3,0) = -Smtrx(nu1_dot);
    C.block<3,3>(3,3) = -Smtrx(nu2_dot);

    return C;
}


//////////////////////////////////////////////////
void ApplyHydrodynamicSystem::PreUpdate( const UpdateInfo &info,
                                         EntityComponentManager &ecm)
{

    if (info.paused)
    {
        return;
    }

    if (this->linkEntity_ == kNullEntity)
    {
        return;
    }

    Link link(this->linkEntity_);
    if (!link.Valid(ecm))
    {
        return;
    }

    // Component checks
    auto poseComp = ecm.Component<components::Pose>(this->linkEntity_);
    auto linVelComp = ecm.Component<components::LinearVelocity>(this->linkEntity_);
    auto angVelComp = ecm.Component<components::AngularVelocity>(this->linkEntity_);
    auto linAccComp = ecm.Component<components::LinearAcceleration>(this->linkEntity_);
    auto angAccComp = ecm.Component<components::AngularAcceleration>(this->linkEntity_);

    if (!poseComp)        igndbg << "Pose component missing\n";
    if (!linVelComp)      igndbg << "LinearVelocity component missing\n";
    if (!angVelComp)      igndbg << "AngularVelocity component missing\n";
    if (!linAccComp)      igndbg << "LinearAcceleration component missing\n";
    if (!angAccComp)      igndbg << "AngularAcceleration component missing\n";

    if (!poseComp || !linVelComp || !angVelComp || !linAccComp || !angAccComp)
        return;


    const auto &pose = poseComp->Data();
    const auto &wLinVel = linVelComp->Data();
    const auto &wAngVel = angVelComp->Data();
    const auto &wLinAcc = linAccComp->Data();
    const auto &wAngAcc = angAccComp->Data();

    // --- Transform to body frame
    const auto R = pose.Rot();
    const auto Rinv = R.Inverse();

    Eigen::Vector3d v((Rinv * wLinVel).X(), (Rinv * wLinVel).Y(), (Rinv * wLinVel).Z());
    Eigen::Vector3d w((Rinv * wAngVel).X(), (Rinv * wAngVel).Y(), (Rinv * wAngVel).Z());
    Eigen::Vector3d vDot((Rinv * wLinAcc).X(), (Rinv * wLinAcc).Y(), (Rinv * wLinAcc).Z());
    Eigen::Vector3d wDot((Rinv * wAngAcc).X(), (Rinv * wAngAcc).Y(), (Rinv * wAngAcc).Z());

    Eigen::VectorXd state(6), stateDot(6);
    state << v(0), v(1), v(2), w(0), w(1), w(2);
    stateDot << vDot(0), vDot(1), vDot(2), wDot(0), wDot(1), wDot(2);

    // --- Added mass
    Eigen::VectorXd F_am = - this->Ma_ * stateDot;

    // --- Coriolis from added mass (Fossen 2002, p.37)
    Eigen::MatrixXd C = ComputeCoriolisMatrix(this->Ma_, v, w);

    Eigen::VectorXd F_c = C * state;

    // --- Linear + quadratic damping
    Eigen::MatrixXd D_total = D_lin_;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            D_total(i,j) += D_quad_(i,j) * std::abs(state(j));

    Eigen::VectorXd F_d = - D_total * state;

    // --- Total force in body frame
    Eigen::VectorXd F_total = F_am + F_c + F_d;

    // --- Rotate to world frame
    math::Vector3d forceWorld(F_total(0), F_total(1), F_total(2));
    math::Vector3d torqueWorld(F_total(3), F_total(4), F_total(5));
    forceWorld = R * forceWorld;
    torqueWorld = R * torqueWorld;

    // --- Apply wrench
    link.AddWorldWrench(ecm, forceWorld, torqueWorld);
}

//////////////////////////////////////////////////
// Plugin registration
IGNITION_ADD_PLUGIN(
  blucy_plugins::ApplyHydrodynamicSystem,
  ignition::gazebo::System,
  blucy_plugins::ApplyHydrodynamicSystem::ISystemConfigure,
  blucy_plugins::ApplyHydrodynamicSystem::ISystemPreUpdate)