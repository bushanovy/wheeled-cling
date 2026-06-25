#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace robot_3d3s
{
class Kmw100AdhesionSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "Kmw100AdhesionSystem must be attached to a model.\n";
      return;
    }

    this->enabled = this->Param(_sdf, "enabled", this->enabled);
    this->adhesionModel = this->Param(_sdf, "adhesion_model", this->adhesionModel);
    this->forceN = this->Param(_sdf, "force_n", this->forceN);
    this->minForceN = this->Param(_sdf, "min_force_n", this->minForceN);
    this->gapReferenceMm =
        this->Param(_sdf, "gap_reference_mm", this->gapReferenceMm);
    this->gapBiasMm = this->Param(_sdf, "gap_bias_mm", this->gapBiasMm);
    this->wheelRadiusM = this->Param(_sdf, "wheel_radius_m", this->wheelRadiusM);
    this->maxGapM = this->Param(_sdf, "max_gap_m", this->maxGapM);
    this->minWallForceN = this->Param(_sdf, "min_wall_force_n", this->minWallForceN);
    this->fullContactDepthM =
        this->Param(_sdf, "full_contact_depth_m", this->fullContactDepthM);
    this->minContactFraction =
        this->Param(_sdf, "min_contact_fraction", this->minContactFraction);
    this->cornerContactFraction =
        this->Param(_sdf, "corner_contact_fraction", this->cornerContactFraction);
    this->cornerCaptureM = this->Param(_sdf, "corner_capture_m", this->cornerCaptureM);
    this->safeTiltDeg = this->Param(_sdf, "safe_tilt_deg", this->safeTiltDeg);
    this->cutoffTiltDeg = this->Param(_sdf, "cutoff_tilt_deg", this->cutoffTiltDeg);
    this->tiltFloor = this->Param(_sdf, "tilt_floor", this->tiltFloor);
    this->tiltRecovery90 =
        this->Param(_sdf, "tilt_recovery_90", this->tiltRecovery90);
    this->slideAngleDeg = this->Param(_sdf, "slide_angle_deg", this->slideAngleDeg);
    this->slideStartX = this->Param(_sdf, "slide_start_x", this->slideStartX);
    this->slideStartZ = this->Param(_sdf, "slide_start_z", this->slideStartZ);
    this->slideWallX = this->Param(_sdf, "slide_wall_x", this->slideWallX);
    this->flatPanelThicknessM =
        this->Param(_sdf, "flat_panel_thickness_m", this->flatPanelThicknessM);
    this->enableOppositeWall =
        this->Param(_sdf, "enable_opposite_wall", this->enableOppositeWall);
    this->enableWall = this->Param(_sdf, "enable_wall", this->enableWall);
    this->enableGround = this->Param(_sdf, "enable_ground", this->enableGround);
    this->enableSlide = this->Param(_sdf, "enable_slide", this->enableSlide);
    this->enablePipe = this->Param(_sdf, "enable_pipe", this->enablePipe);
    this->pipeAxis = this->Param(_sdf, "pipe_axis", this->pipeAxis);
    this->pipeCenterX = this->Param(_sdf, "pipe_center_x", this->pipeCenterX);
    this->pipeCenterY = this->Param(_sdf, "pipe_center_y", this->pipeCenterY);
    this->pipeCenterZ = this->Param(_sdf, "pipe_center_z", this->pipeCenterZ);
    this->pipeRadiusM = this->Param(_sdf, "pipe_radius_m", this->pipeRadiusM);
    this->pipeLengthM = this->Param(_sdf, "pipe_length_m", this->pipeLengthM);
    this->groundZ = this->Param(_sdf, "ground_z", this->groundZ);
    this->surfaceMarginM = this->Param(_sdf, "surface_margin_m", this->surfaceMarginM);
    this->debugPeriodS = this->Param(_sdf, "debug_period_s", this->debugPeriodS);
    this->statusPeriodS = this->Param(_sdf, "status_period_s", this->statusPeriodS);
    this->statusTopic = this->Param(_sdf, "status_topic", this->statusTopic);

    this->wheelLinks.clear();
    for (const auto &name : this->wheelNames)
    {
      const auto entity = this->model.LinkByName(_ecm, name);
      if (entity == gz::sim::kNullEntity)
      {
        gzerr << "Kmw100AdhesionSystem could not find link [" << name << "].\n";
        continue;
      }
      this->wheelLinks.emplace_back(entity);
    }

    // The root/canonical link, whose world pose we republish so RViz can track
    // the real Gazebo robot (the bridged Pose_V topics arrive with empty frame
    // names, so we carry the base pose in our own status message instead).
    auto baseEntity = this->model.LinkByName(_ecm, "base_footprint");
    this->baseFrame = "base_footprint";
    if (baseEntity == gz::sim::kNullEntity)
    {
      baseEntity = this->model.LinkByName(_ecm, "base_link");
      this->baseFrame = "base_link";
    }
    if (baseEntity != gz::sim::kNullEntity)
      this->baseLink = gz::sim::Link(baseEntity);

    this->statusPub =
        this->transportNode.Advertise<gz::msgs::StringMsg>(this->statusTopic);

    gzmsg << "Kmw100AdhesionSystem "
          << (this->enabled ? "active" : "disabled")
          << " on " << this->wheelLinks.size()
          << " wheel links. model=" << this->adhesionModel
          << ", full_contact_force=" << this->forceN
          << " N, wall_force_min=" << this->minWallForceN
          << " N, max_gap=" << this->maxGapM
          << " m, both_wall_faces=" << (this->enableOppositeWall ? "true" : "false")
          << " m, status_topic=" << this->statusTopic
          << ". gap_decay follows F=F0/(1+s^3) with s=gap_mm/gap_reference_mm.\n";
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->enabled)
      return;

    const double angle = this->slideAngleDeg * M_PI / 180.0;
    const gz::math::Vector3d groundNormal(0.0, 0.0, 1.0);
    const gz::math::Vector3d slideNormal(-std::sin(angle), 0.0, std::cos(angle));
    const gz::math::Vector3d slideDir(std::cos(angle), 0.0, std::sin(angle));
    const double slideLen = std::max(
        0.0, (this->slideWallX - this->slideStartX) /
        std::max(1e-6, slideDir.X()));
    const double slideTopZ = this->slideStartZ + slideLen * slideDir.Z();

    std::vector<std::string> debugParts;
    std::vector<WheelStatus> wheelStatuses;
    for (std::size_t i = 0; i < this->wheelLinks.size(); ++i)
    {
      WheelStatus status;
      status.wheel = this->wheelNames[i];

      const auto poseOpt = this->wheelLinks[i].WorldPose(_ecm);
      if (!poseOpt)
      {
        wheelStatuses.push_back(status);
        continue;
      }

      const auto p = poseOpt->Pos();
      std::vector<Surface> candidates;

      this->ConsiderSurface(
          candidates, "ground", "ground", groundNormal,
          gz::math::Vector3d(0.0, 0.0, this->groundZ), p,
          this->enableGround &&
          p.X() <= this->slideStartX + this->surfaceMarginM &&
          p.Z() <= this->slideStartZ + this->wheelRadiusM + 2.0 * this->surfaceMarginM);

      const double slideProgress =
          (p.X() - this->slideStartX) * slideDir.X() +
          (p.Z() - this->slideStartZ) * slideDir.Z();
      this->ConsiderSurface(
          candidates, "slide", "slide", slideNormal,
          gz::math::Vector3d(this->slideStartX, 0.0, this->slideStartZ), p,
          this->enableSlide &&
          -this->surfaceMarginM <= slideProgress &&
          slideProgress <= slideLen + this->surfaceMarginM);

      // With the slide enabled the wall only starts above the ramp top; with a
      // standalone flat wall (slide disabled) the whole face is climbable.
      const bool wallValid = this->enableWall && (
          !this->enableSlide ||
          p.Z() >= slideTopZ - 3.0 * this->surfaceMarginM);
      this->ConsiderSurface(
          candidates, "wall_neg_x", "wall", gz::math::Vector3d(-1.0, 0.0, 0.0),
          gz::math::Vector3d(this->slideWallX, 0.0, 0.0), p,
          wallValid);

      if (this->enableOppositeWall)
      {
        this->ConsiderSurface(
            candidates, "wall_pos_x", "wall", gz::math::Vector3d(1.0, 0.0, 0.0),
            gz::math::Vector3d(
                this->slideWallX + this->flatPanelThicknessM, 0.0, 0.0),
            p,
            p.Z() >= slideTopZ - 3.0 * this->surfaceMarginM);
      }

      this->ConsiderPipeSurface(candidates, p);

      if (candidates.empty())
      {
        wheelStatuses.push_back(status);
        continue;
      }

      std::sort(candidates.begin(), candidates.end(),
          [](const Surface &_a, const Surface &_b)
          {
            return _a.score < _b.score;
          });

      const double bestScore = candidates.front().score;
      // The wheel spins about its joint axis, which is X in the wheel link frame
      // (URDF wheel_*_joint axis="1 0 0"). A seated wheel has this axle parallel
      // to the surface, so tilt = asin(|axle . normal|) is ~0 when seated on ANY
      // surface (ground/slide/wall). Using local Z here made the ground/slide
      // tilt read ~50-80 deg and clamp adhesion to the floor.
      const auto wheelAxis = poseOpt->Rot().RotateVector(
          gz::math::Vector3d(1.0, 0.0, 0.0));
      gz::math::Vector3d totalForce(0.0, 0.0, 0.0);
      double totalForceN = 0.0;
      std::string surfaceNames;
      bool multiSurface = false;

      for (auto surface : candidates)
      {
        if (surface.gapM > this->maxGapM ||
            surface.score > bestScore + this->cornerCaptureM)
        {
          continue;
        }

        const int activeCount = static_cast<int>(std::count_if(
            candidates.begin(), candidates.end(),
            [&](const Surface &_surface)
            {
              return _surface.gapM <= this->maxGapM &&
                  _surface.score <= bestScore + this->cornerCaptureM;
            }));
        multiSurface = activeCount > 1;
        if (multiSurface)
          surface.contactFraction =
              std::min(surface.contactFraction, this->cornerContactFraction);

        const double tiltDeg = this->TiltDeg(wheelAxis, -surface.normal);
        double force = this->AdhesionForceN(
            surface.gapM, surface.contactFraction, tiltDeg);
        if (surface.statusName == "wall")
          force = std::max(force, this->minWallForceN * surface.contactFraction);

        const gz::math::Vector3d forceVec = -surface.normal * force;
        totalForce += forceVec;
        totalForceN += force;

        if (!surfaceNames.empty())
          surfaceNames += "+";
        surfaceNames += surface.statusName;

        debugParts.push_back(
            this->wheelNames[i] + "=" + surface.name +
            ",gap=" + std::to_string(surface.gapM * 1000.0) +
            "mm,cf=" + std::to_string(surface.contactFraction) +
            ",tilt=" + std::to_string(tiltDeg) +
            "deg,F=(" + std::to_string(forceVec.X()) + "," +
            std::to_string(forceVec.Y()) + "," +
            std::to_string(forceVec.Z()) + ")");
      }

      if (totalForceN <= 0.0)
      {
        wheelStatuses.push_back(status);
        continue;
      }

      this->wheelLinks[i].AddWorldForce(_ecm, totalForce);

      status.surface = surfaceNames;
      status.gapM = candidates.front().gapM;
      status.normal = totalForce.Normalized();
      status.attached = true;
      status.forceN = totalForceN;
      status.contactFraction = multiSurface ?
          this->cornerContactFraction : candidates.front().contactFraction;
      wheelStatuses.push_back(status);
    }

    if (this->debugPeriodS > 0.0 &&
        _info.simTime >= this->nextDebugTime)
    {
      this->nextDebugTime =
          _info.simTime + std::chrono::duration_cast<
              std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(this->debugPeriodS));
      std::string joined;
      for (const auto &part : debugParts)
      {
        if (!joined.empty())
          joined += "; ";
        joined += part;
      }
      gzmsg << "Kmw100AdhesionSystem: " << joined << "\n";
    }

    if (this->statusPeriodS > 0.0 &&
        _info.simTime >= this->nextStatusTime)
    {
      this->nextStatusTime =
          _info.simTime + std::chrono::duration_cast<
              std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(this->statusPeriodS));

      gz::math::Pose3d basePose;
      bool haveBasePose = false;
      if (this->baseLink.Valid(_ecm))
      {
        const auto basePoseOpt = this->baseLink.WorldPose(_ecm);
        if (basePoseOpt)
        {
          basePose = *basePoseOpt;
          haveBasePose = true;
        }
      }

      gz::msgs::StringMsg msg;
      msg.set_data(this->StatusJson(
          wheelStatuses, _info.simTime.count(), basePose, haveBasePose));
      this->statusPub.Publish(msg);
    }
  }

  private: template <typename T>
  T Param(const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name, const T &_defaultValue) const
  {
    if (_sdf && _sdf->HasElement(_name))
      return _sdf->Get<T>(_name);
    return _defaultValue;
  }

  private: struct Surface
  {
    std::string name;
    std::string statusName;
    gz::math::Vector3d normal{1.0, 0.0, 0.0};
    double gapM{0.0};
    double score{1e9};
    double contactFraction{1.0};
  };

  private: struct WheelStatus
  {
    std::string wheel;
    std::string surface{"none"};
    bool attached{false};
    double gapM{0.0};
    double forceN{0.0};
    double contactFraction{0.0};
    gz::math::Vector3d normal{0.0, 0.0, 0.0};
  };

  private: void ConsiderSurface(
      std::vector<Surface> &_candidates,
      const std::string &_name,
      const std::string &_statusName,
      const gz::math::Vector3d &_normal,
      const gz::math::Vector3d &_point,
      const gz::math::Vector3d &_wheelPos,
      bool _valid) const
  {
    if (!_valid)
      return;

    const double signedCenterGap = (_wheelPos - _point).Dot(_normal);
    const double gap = std::max(0.0, signedCenterGap - this->wheelRadiusM);
    const double score = std::abs(signedCenterGap - this->wheelRadiusM);
    if (score > this->surfaceMarginM + this->maxGapM)
      return;

    Surface surface;
    surface.name = _name;
    surface.statusName = _statusName;
    surface.normal = _normal.Normalized();
    surface.gapM = gap;
    surface.score = score;
    surface.contactFraction = this->ContactFraction(score, gap);
    _candidates.push_back(surface);
  }

  private: void ConsiderPipeSurface(
      std::vector<Surface> &_candidates,
      const gz::math::Vector3d &_wheelPos) const
  {
    if (!this->enablePipe || this->pipeRadiusM <= 0.0)
      return;

    const gz::math::Vector3d center(
        this->pipeCenterX, this->pipeCenterY, this->pipeCenterZ);
    gz::math::Vector3d axis(1.0, 0.0, 0.0);
    const std::string lowerAxis = this->Lower(this->pipeAxis);
    if (lowerAxis == "y")
      axis = gz::math::Vector3d(0.0, 1.0, 0.0);
    else if (lowerAxis == "z")
      axis = gz::math::Vector3d(0.0, 0.0, 1.0);

    const gz::math::Vector3d rel = _wheelPos - center;
    const double axial = rel.Dot(axis);
    if (std::abs(axial) > 0.5 * this->pipeLengthM + this->surfaceMarginM)
      return;

    const gz::math::Vector3d radial = rel - axis * axial;
    const double radialLen = radial.Length();
    if (radialLen <= 1e-9)
      return;

    const gz::math::Vector3d normal = radial / radialLen;
    const gz::math::Vector3d surfacePoint = center + axis * axial + normal * this->pipeRadiusM;
    this->ConsiderSurface(
        _candidates, "pipe_" + lowerAxis, "pipe",
        normal, surfacePoint, _wheelPos, true);
  }

  private: double ContactFraction(const double _score, const double _gap) const
  {
    if (_gap > this->maxGapM)
      return 0.0;
    if (this->fullContactDepthM <= 0.0 || _score <= this->fullContactDepthM)
      return 1.0;
    const double span = std::max(1e-9, this->maxGapM + this->surfaceMarginM);
    const double fraction = 1.0 - std::min(1.0, _score / span);
    return std::max(this->minContactFraction, std::min(1.0, fraction));
  }

  private: double GapDecayScale(const double _gapM) const
  {
    if (this->gapReferenceMm <= 0.0)
      return 1.0;
    const double gapMm = std::max(0.0, _gapM * 1000.0 + this->gapBiasMm);
    const double s = gapMm / this->gapReferenceMm;
    return 1.0 / (1.0 + s * s * s);
  }

  private: double TiltDeg(
      const gz::math::Vector3d &_wheelAxis,
      const gz::math::Vector3d &_forceDirection) const
  {
    // _wheelAxis is the wheel's spin axis (local z); _forceDirection is into the
    // surface (= -surface normal). A correctly SEATED wheel rolls with its spin
    // axis parallel to the surface, i.e. perpendicular to the normal, so the
    // component of the spin axis along the normal is ~0 => tilt 0 deg => full
    // force. As the wheel cambers off the surface, that component grows toward 1
    // => tilt 90 deg. Hence asin(|spinAxis . normal|), NOT acos. (The previous
    // acos reported a seated wheel as 90 deg tilt, which with the old
    // linear-to-zero model zeroed adhesion on perfectly-seated wheels.)
    const double axisLen = _wheelAxis.Length();
    const double forceLen = _forceDirection.Length();
    if (axisLen <= 1e-9 || forceLen <= 1e-9)
      return 0.0;
    const double normalComponent = std::abs(
        _wheelAxis.Dot(_forceDirection) / (axisLen * forceLen));
    const double clamped = std::max(0.0, std::min(1.0, normalComponent));
    return std::asin(clamped) * 180.0 / M_PI;
  }

  private: double TiltScale(const double _tiltDeg) const
  {
    // OmniClimbers (Tavakoli 2013) Fig. 10 — magnetic force vs. tilt angle.
    // The force is ~100% when the magnet is aligned with the surface, dips to a
    // floor (~45%) near 45-60 deg, then RECOVERS toward ~55% at 90 deg because
    // attraction is then summed over the front and lateral poles of the magnet.
    // It never drops to zero (the old linear-to-zero model peeled the robot off
    // whenever a wheel tilted during a transition).
    const double theta = std::min(M_PI_2, std::abs(_tiltDeg) * M_PI / 180.0);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double front = c * c;                       // 1 at 0 deg, 0 at 90 deg
    const double lateral = this->tiltRecovery90 * s * s * s * s;  // grows near 90
    const double scale = std::max(front, lateral);
    return std::max(this->tiltFloor, std::min(1.0, scale));
  }

  private: double AdhesionForceN(
      const double _gapM,
      const double _contactFraction,
      const double _tiltDeg) const
  {
    double force = this->forceN;
    if (this->adhesionModel == "gap_decay")
      force *= this->GapDecayScale(_gapM);
    force *= std::max(0.0, std::min(1.0, _contactFraction));
    force *= this->TiltScale(_tiltDeg);
    return std::max(force, this->minForceN);
  }

  private: std::string DominantSurface(
      int _groundCount, int _slideCount, int _wallCount, int _pipeCount) const
  {
    if (_pipeCount >= _wallCount && _pipeCount >= _slideCount &&
        _pipeCount >= _groundCount && _pipeCount > 0)
      return "pipe";
    if (_wallCount >= _slideCount && _wallCount >= _groundCount && _wallCount > 0)
      return "wall";
    if (_slideCount >= _groundCount && _slideCount > 0)
      return "slide";
    if (_groundCount > 0)
      return "ground";
    return "none";
  }

  private: std::string StatusJson(
      const std::vector<WheelStatus> &_statuses,
      const int64_t _simTimeNs,
      const gz::math::Pose3d &_basePose,
      const bool _haveBasePose) const
  {
    int attachedCount = 0;
    int groundCount = 0;
    int slideCount = 0;
    int wallCount = 0;
    int pipeCount = 0;

    for (const auto &status : _statuses)
    {
      if (!status.attached)
        continue;

      ++attachedCount;
      if (status.surface.find("ground") != std::string::npos)
        ++groundCount;
      else if (status.surface.find("slide") != std::string::npos)
        ++slideCount;
      else if (status.surface.find("wall") != std::string::npos)
        ++wallCount;
      else if (status.surface.find("pipe") != std::string::npos)
        ++pipeCount;
    }

    std::ostringstream out;
    out << "{\"sim_time_s\":" << (static_cast<double>(_simTimeNs) * 1e-9)
        << ",\"dominant_surface\":\""
        << this->DominantSurface(groundCount, slideCount, wallCount, pipeCount)
        << "\",\"attached_count\":" << attachedCount
        << ",\"ground_count\":" << groundCount
        << ",\"slide_count\":" << slideCount
        << ",\"wall_count\":" << wallCount
        << ",\"pipe_count\":" << pipeCount
        << ",\"boundary_risk\":0.0"
        << ",\"boundary_state\":\"ok\""
        << ",\"wheels\":[";

    for (std::size_t i = 0; i < _statuses.size(); ++i)
    {
      const auto &status = _statuses[i];
      if (i > 0)
        out << ",";
      out << "{\"wheel\":\"" << status.wheel
          << "\",\"surface\":\"" << status.surface
          << "\",\"attached\":" << (status.attached ? "true" : "false")
          << ",\"gap_mm\":" << (status.gapM * 1000.0)
          << ",\"force_n\":" << status.forceN
          << ",\"contact_fraction\":" << status.contactFraction
          << ",\"normal\":[" << status.normal.X()
          << "," << status.normal.Y()
          << "," << status.normal.Z() << "]}";
    }

    out << "]";

    if (_haveBasePose)
    {
      const auto &p = _basePose.Pos();
      const auto &q = _basePose.Rot();
      out << ",\"base_frame\":\"" << this->baseFrame << "\""
          << ",\"base_pose\":{\"p\":[" << p.X() << "," << p.Y() << "," << p.Z()
          << "],\"q\":[" << q.X() << "," << q.Y() << "," << q.Z() << ","
          << q.W() << "]}";
    }

    out << "}";
    return out.str();
  }

  private: gz::sim::Model model;
  private: std::vector<std::string> wheelNames{
      "wheel_1_link", "wheel_2_link", "wheel_3_link"};
  private: std::vector<gz::sim::Link> wheelLinks;
  private: gz::sim::Link baseLink;
  private: std::string baseFrame{"base_footprint"};
  private: bool enabled{true};
  private: std::string adhesionModel{"gap_decay"};
  private: double forceN{900.0};
  private: double minForceN{0.0};
  private: double minWallForceN{350.0};
  private: double gapReferenceMm{3.0};
  private: double gapBiasMm{0.0};
  private: double wheelRadiusM{0.05};
  private: double maxGapM{0.06};
  private: double fullContactDepthM{0.002};
  private: double minContactFraction{0.10};
  private: double cornerContactFraction{0.85};
  private: double cornerCaptureM{0.08};
  private: double safeTiltDeg{20.0};
  private: double cutoffTiltDeg{60.0};
  private: double tiltFloor{0.45};
  private: double tiltRecovery90{0.55};
  private: double slideAngleDeg{45.0};
  private: double slideStartX{-0.15};
  private: double slideStartZ{0.0};
  private: double slideWallX{0.95};
  private: double flatPanelThicknessM{0.10};
  private: bool enableOppositeWall{true};
  private: bool enableWall{true};
  private: bool enableGround{true};
  private: bool enableSlide{true};
  private: bool enablePipe{false};
  private: std::string pipeAxis{"x"};
  private: double pipeCenterX{0.0};
  private: double pipeCenterY{0.0};
  private: double pipeCenterZ{1.20};
  private: double pipeRadiusM{1.20};
  private: double pipeLengthM{6.0};
  private: double groundZ{0.0};
  private: double surfaceMarginM{0.08};
  private: double debugPeriodS{1.0};
  private: double statusPeriodS{0.05};
  private: std::string statusTopic{"/model/robot_3d3s/adhesion_status"};
  private: std::chrono::steady_clock::duration nextDebugTime{0};
  private: std::chrono::steady_clock::duration nextStatusTime{0};
  private: gz::transport::Node transportNode;
  private: gz::transport::Node::Publisher statusPub;

  private: static std::string Lower(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return value;
  }
};
}  // namespace robot_3d3s

GZ_ADD_PLUGIN(
    robot_3d3s::Kmw100AdhesionSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    robot_3d3s::Kmw100AdhesionSystem,
    "robot_3d3s::Kmw100AdhesionSystem")
