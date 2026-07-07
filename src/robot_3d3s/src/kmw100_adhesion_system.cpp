#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
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
    this->forceTableCsv = this->Param(_sdf, "force_table_csv", this->forceTableCsv);
    this->wallThicknessMm =
        this->Param(_sdf, "wall_thickness_mm", this->wallThicknessMm);
    this->maxLookupGapMm =
        this->Param(_sdf, "max_lookup_gap_mm", this->maxLookupGapMm);
    this->gapReferenceMm =
        this->Param(_sdf, "gap_reference_mm", this->gapReferenceMm);
    this->gapBiasMm = this->Param(_sdf, "gap_bias_mm", this->gapBiasMm);
    this->wheelRadiusM = this->Param(_sdf, "wheel_radius_m", this->wheelRadiusM);
    this->wheelWidthM = this->Param(_sdf, "wheel_width_m", this->wheelWidthM);
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

    if (this->adhesionModel == "lookup")
    {
      if (this->forceTableCsv.empty() || !this->LoadForceTable(this->forceTableCsv))
      {
        gzerr << "Kmw100AdhesionSystem could not load lookup table ["
              << this->forceTableCsv
              << "]; falling back to gap_decay.\n";
        this->adhesionModel = "gap_decay";
      }
    }

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
          << ", table_rows=" << this->forceTable.size()
          << ", status_topic=" << this->statusTopic
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

      // The wheel spins about its joint axis, which is X in the wheel link frame
      // (URDF wheel_*_joint axis="1 0 0"). A seated wheel has this axle parallel
      // to the surface, so tilt = asin(|axle . normal|) is ~0 when seated on ANY
      // surface (ground/slide/wall). Using local Z here made the ground/slide
      // tilt read ~50-80 deg and clamp adhesion to the floor.
      const auto wheelAxis = poseOpt->Rot().RotateVector(
          gz::math::Vector3d(1.0, 0.0, 0.0));

      this->ConsiderPipeSurface(candidates, p, wheelAxis);

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
      gz::math::Vector3d totalForce(0.0, 0.0, 0.0);
      double totalForceN = 0.0;
      std::string surfaceNames;
      bool multiSurface = false;

      for (auto surface : candidates)
      {
        if (surface.forceGapM > this->maxGapM ||
            surface.score > bestScore + this->cornerCaptureM)
        {
          continue;
        }

        const int activeCount = static_cast<int>(std::count_if(
            candidates.begin(), candidates.end(),
            [&](const Surface &_surface)
            {
              return _surface.forceGapM <= this->maxGapM &&
                  _surface.score <= bestScore + this->cornerCaptureM;
            }));
        multiSurface = activeCount > 1;
        if (multiSurface)
          surface.contactFraction =
              std::min(surface.contactFraction, this->cornerContactFraction);

        const double tiltDeg = this->TiltDeg(wheelAxis, -surface.normal);
        double force = this->AdhesionForceN(
            surface.forceGapM, surface.contactFraction, tiltDeg);
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
            ",gap=" + std::to_string(surface.forceGapM * 1000.0) +
            "mm,radial_gap=" + std::to_string(surface.gapM * 1000.0) +
            "mm,curv_gap=" + std::to_string(surface.curvatureGapM * 1000.0) +
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
      status.gapM = candidates.front().forceGapM;
      status.radialGapM = candidates.front().gapM;
      status.curvatureGapM = candidates.front().curvatureGapM;
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
    double forceGapM{0.0};
    double curvatureGapM{0.0};
    double score{1e9};
    double contactFraction{1.0};
  };

  private: struct WheelStatus
  {
    std::string wheel;
    std::string surface{"none"};
    bool attached{false};
    double gapM{0.0};
    double radialGapM{0.0};
    double curvatureGapM{0.0};
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
    surface.forceGapM = gap;
    surface.score = score;
    surface.contactFraction = this->ContactFraction(score, gap);
    _candidates.push_back(surface);
  }

  private: void ConsiderPipeSurface(
      std::vector<Surface> &_candidates,
      const gz::math::Vector3d &_wheelPos,
      const gz::math::Vector3d &_wheelAxis) const
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
    const double signedCenterGap = radialLen - this->pipeRadiusM;
    const double radialGap = std::max(0.0, signedCenterGap - this->wheelRadiusM);
    const double radialScore = std::abs(signedCenterGap - this->wheelRadiusM);

    const double axisAlignment = this->AxisAlignment(_wheelAxis, axis);
    const double circumferentialSpan =
        0.5 * this->wheelWidthM *
        std::sqrt(std::max(0.0, 1.0 - axisAlignment * axisAlignment));
    double curvatureGap = 0.0;
    if (circumferentialSpan > 0.0 && circumferentialSpan < this->pipeRadiusM)
    {
      curvatureGap = this->pipeRadiusM -
          std::sqrt(std::max(
              0.0,
              this->pipeRadiusM * this->pipeRadiusM -
              circumferentialSpan * circumferentialSpan));
    }
    else if (circumferentialSpan >= this->pipeRadiusM)
    {
      curvatureGap = this->maxGapM + this->surfaceMarginM;
    }

    const double forceGap = radialGap + curvatureGap;
    const double score = radialScore + curvatureGap;
    if (score > this->surfaceMarginM + this->maxGapM)
      return;

    Surface surface;
    surface.name = "pipe_" + lowerAxis;
    surface.statusName = "pipe";
    surface.normal = normal;
    surface.gapM = radialGap;
    surface.forceGapM = forceGap;
    surface.curvatureGapM = curvatureGap;
    surface.score = score;
    surface.contactFraction =
        this->ContactFraction(radialScore, radialGap) *
        this->ContactFraction(curvatureGap, curvatureGap);
    _candidates.push_back(surface);
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

  private: double AxisAlignment(
      const gz::math::Vector3d &_a,
      const gz::math::Vector3d &_b) const
  {
    const double aLen = _a.Length();
    const double bLen = _b.Length();
    if (aLen <= 1e-9 || bLen <= 1e-9)
      return 1.0;
    return std::abs(_a.Dot(_b) / (aLen * bLen));
  }

  private: struct ForceKey
  {
    double gapMm{0.0};
    double thicknessMm{0.0};
    double pipeRadiusM{0.0};
    double contactFraction{1.0};

    bool operator<(const ForceKey &_other) const
    {
      return std::tie(
          this->gapMm, this->thicknessMm,
          this->pipeRadiusM, this->contactFraction) <
          std::tie(
              _other.gapMm, _other.thicknessMm,
              _other.pipeRadiusM, _other.contactFraction);
    }
  };

  private: static std::string Trim(const std::string &_value)
  {
    const auto begin = std::find_if_not(_value.begin(), _value.end(),
        [](unsigned char c){ return std::isspace(c); });
    const auto end = std::find_if_not(_value.rbegin(), _value.rend(),
        [](unsigned char c){ return std::isspace(c); }).base();
    if (begin >= end)
      return "";
    return std::string(begin, end);
  }

  private: static std::vector<std::string> SplitCsvLine(const std::string &_line)
  {
    std::vector<std::string> fields;
    std::stringstream stream(_line);
    std::string field;
    while (std::getline(stream, field, ','))
      fields.push_back(Trim(field));
    return fields;
  }

  private: static bool HasColumn(
      const std::map<std::string, std::size_t> &_columns,
      const std::string &_name)
  {
    return _columns.find(_name) != _columns.end();
  }

  private: static double CsvDouble(
      const std::vector<std::string> &_row,
      const std::map<std::string, std::size_t> &_columns,
      const std::string &_name,
      const double _defaultValue)
  {
    const auto it = _columns.find(_name);
    if (it == _columns.end() || it->second >= _row.size() || _row[it->second].empty())
      return _defaultValue;
    return std::stod(_row[it->second]);
  }

  private: bool LoadForceTable(const std::string &_path)
  {
    std::ifstream file(_path);
    if (!file)
      return false;

    std::string line;
    if (!std::getline(file, line))
      return false;

    const auto headers = SplitCsvLine(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
      columns[headers[i]] = i;

    if (!HasColumn(columns, "gap_mm") || !HasColumn(columns, "force_n") ||
        (!HasColumn(columns, "thickness_mm") &&
         !HasColumn(columns, "wall_thickness_mm")))
    {
      gzerr << "Kmw100AdhesionSystem lookup CSV [" << _path
            << "] needs gap_mm, force_n, and thickness_mm or wall_thickness_mm.\n";
      return false;
    }

    this->hasPipeRadiusAxis = HasColumn(columns, "pipe_radius_m");
    this->hasContactFractionAxis = HasColumn(columns, "contact_fraction");
    this->forceTable.clear();
    this->gapAxisMm.clear();
    this->thicknessAxisMm.clear();
    this->pipeRadiusAxisM.clear();
    this->contactFractionAxis.clear();

    std::set<double> gaps;
    std::set<double> thicknesses;
    std::set<double> radii;
    std::set<double> contacts;
    while (std::getline(file, line))
    {
      if (Trim(line).empty())
        continue;
      const auto row = SplitCsvLine(line);
      const double gapMm = CsvDouble(row, columns, "gap_mm", 0.0);
      const double thicknessMm = HasColumn(columns, "wall_thickness_mm") ?
          CsvDouble(row, columns, "wall_thickness_mm", 0.0) :
          CsvDouble(row, columns, "thickness_mm", 0.0);
      const double pipeRadiusM = CsvDouble(row, columns, "pipe_radius_m", 0.0);
      const double contactFraction =
          CsvDouble(row, columns, "contact_fraction", 1.0);
      const double forceN = CsvDouble(row, columns, "force_n", 0.0);
      gaps.insert(gapMm);
      thicknesses.insert(thicknessMm);
      radii.insert(pipeRadiusM);
      contacts.insert(contactFraction);
      this->forceTable[{gapMm, thicknessMm, pipeRadiusM, contactFraction}] = forceN;
    }

    this->gapAxisMm.assign(gaps.begin(), gaps.end());
    this->thicknessAxisMm.assign(thicknesses.begin(), thicknesses.end());
    this->pipeRadiusAxisM.assign(radii.begin(), radii.end());
    this->contactFractionAxis.assign(contacts.begin(), contacts.end());
    return !this->forceTable.empty();
  }

  private: static std::tuple<double, double, double> Bounds(
      const std::vector<double> &_axis,
      const double _query)
  {
    if (_axis.empty())
      return {0.0, 0.0, 0.0};
    if (_query <= _axis.front())
      return {_axis.front(), _axis.front(), 0.0};
    if (_query >= _axis.back())
      return {_axis.back(), _axis.back(), 0.0};
    for (std::size_t i = 0; i + 1 < _axis.size(); ++i)
    {
      if (_axis[i] <= _query && _query <= _axis[i + 1])
      {
        const double ratio = (_query - _axis[i]) / (_axis[i + 1] - _axis[i]);
        return {_axis[i], _axis[i + 1], ratio};
      }
    }
    return {_axis.back(), _axis.back(), 0.0};
  }

  private: double LookupForceN(
      const double _gapMm,
      const double _contactFraction) const
  {
    const auto [g0, g1, gr] = Bounds(this->gapAxisMm, _gapMm);
    const auto [t0, t1, tr] = Bounds(this->thicknessAxisMm, this->wallThicknessMm);
    const auto [r0, r1, rr] = Bounds(this->pipeRadiusAxisM, this->pipeRadiusM);
    const auto [c0, c1, cr] =
        Bounds(this->contactFractionAxis, _contactFraction);

    double total = 0.0;
    for (int gi = 0; gi < 2; ++gi)
    {
      const double gv = gi == 0 ? g0 : g1;
      const double gw = gi == 0 ? 1.0 - gr : gr;
      for (int ti = 0; ti < 2; ++ti)
      {
        const double tv = ti == 0 ? t0 : t1;
        const double tw = ti == 0 ? 1.0 - tr : tr;
        for (int ri = 0; ri < 2; ++ri)
        {
          const double rv = ri == 0 ? r0 : r1;
          const double rw = ri == 0 ? 1.0 - rr : rr;
          for (int ci = 0; ci < 2; ++ci)
          {
            const double cv = ci == 0 ? c0 : c1;
            const double cw = ci == 0 ? 1.0 - cr : cr;
            const auto it = this->forceTable.find({gv, tv, rv, cv});
            if (it != this->forceTable.end())
              total += it->second * gw * tw * rw * cw;
          }
        }
      }
    }

    if (!this->hasContactFractionAxis)
      total *= _contactFraction;
    return total;
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
    const double contactFraction = std::max(0.0, std::min(1.0, _contactFraction));
    double force = this->forceN;
    if (this->adhesionModel == "lookup" && !this->forceTable.empty())
    {
      double gapMm = std::max(0.0, _gapM * 1000.0 + this->gapBiasMm);
      if (this->maxLookupGapMm >= 0.0)
        gapMm = std::min(gapMm, this->maxLookupGapMm);
      force = this->LookupForceN(gapMm, contactFraction);
    }
    else
    {
      if (this->adhesionModel == "gap_decay")
        force *= this->GapDecayScale(_gapM);
      force *= contactFraction;
    }
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
        << ",\"adhesion_model\":\"" << this->adhesionModel << "\""
        << ",\"force_nominal_n\":" << this->forceN
        << ",\"min_force_n\":" << this->minForceN
        << ",\"max_gap_mm\":" << (this->maxGapM * 1000.0)
        << ",\"gap_reference_mm\":" << this->gapReferenceMm
        << ",\"pipe_radius_m\":" << this->pipeRadiusM
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
          << ",\"effective_gap_mm\":" << (status.gapM * 1000.0)
          << ",\"radial_gap_mm\":" << (status.radialGapM * 1000.0)
          << ",\"curvature_gap_mm\":" << (status.curvatureGapM * 1000.0)
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
  private: std::string forceTableCsv{""};
  private: double wallThicknessMm{10.0};
  private: double maxLookupGapMm{-1.0};
  private: double gapReferenceMm{3.0};
  private: double gapBiasMm{0.0};
  private: double wheelRadiusM{0.05};
  private: double wheelWidthM{0.05};
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
  private: std::map<ForceKey, double> forceTable;
  private: std::vector<double> gapAxisMm;
  private: std::vector<double> thicknessAxisMm;
  private: std::vector<double> pipeRadiusAxisM;
  private: std::vector<double> contactFractionAxis;
  private: bool hasPipeRadiusAxis{false};
  private: bool hasContactFractionAxis{false};

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
