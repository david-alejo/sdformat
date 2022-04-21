/*
 * Copyright 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "sdf/Radar.hh"
#include "sdf/parser.hh"

using namespace sdf;
using namespace ignition;

/// \brief Private radar data.
class sdf::Radar::Implementation
{
  /// \brief Number of rays horizontally per laser sweep
  public: unsigned int horizontalScanSamples{640};

  /// \brief Resolution for horizontal scan
  public: double horizontalScanResolution{1.0};

  /// \brief Minimum angle for horizontal scan
  public: math::Angle horizontalScanMinAngle{0.0};

  /// \brief Maximum angle for horizontal scan
  public: math::Angle horizontalScanMaxAngle{0.0};

  /// \brief Number of rays vertically per laser sweep
  public: unsigned int verticalScanSamples{1};

  /// \brief Resolution for vertical scan
  public: double verticalScanResolution{1.0};

  /// \brief Minimum angle for vertical scan
  public: math::Angle verticalScanMinAngle{0.0};

  /// \brief Maximum angle for vertical scan
  public: math::Angle verticalScanMaxAngle{0.0};

  /// \brief Minimum distance for each ray
  public: double minRange{0.0};

  /// \brief Maximum distance for each ray
  public: double maxRange{0.0};

  /// \brief Linear resolution for each ray
  public: double rangeResolution{0.0};

  /// \brief Noise values for the radar sensor
  public: Noise radarNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};

  /// \brief Number of horizontal measures
  public: int h_measures{6};

  /// \brief Number of vertical measures
  public: int v_measures{4};
};

//////////////////////////////////////////////////
Radar::Radar()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
/// \brief Load the radar based on an element pointer. This is *not*
/// the usual entry point. Typical usage of the SDF DOM is through the Root
/// object.
/// \param[in] _sdf The SDF Element pointer
/// \return Errors, which is a vector of Error objects. Each Error includes
/// an error code and message. An empty vector indicates no error.
Errors Radar::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Radar, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <radar> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "radar" && _sdf->GetName() != "gpu_radar")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Radar, but the provided SDF element is "
        "not a <radar>."});
    return errors;
  }

  // Load radar sensor properties
  if (_sdf->HasElement("scan"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("scan");
    if (elem->HasElement("horizontal"))
    {
      sdf::ElementPtr subElem = elem->GetElement("horizontal");
      if (subElem->HasElement("samples"))
        this->dataPtr->horizontalScanSamples = subElem->Get<unsigned int>(
          "samples");
      if (subElem->HasElement("resolution"))
        this->dataPtr->horizontalScanResolution = subElem->Get<double>(
          "resolution");
      if (subElem->HasElement("min_angle"))
        this->dataPtr->horizontalScanMinAngle = math::Angle(
          subElem->Get<double>("min_angle"));
      if (subElem->HasElement("max_angle"))
        this->dataPtr->horizontalScanMaxAngle = math::Angle(
          subElem->Get<double>("max_angle"));
    }
    else
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
        "A radar scan horizontal element is required, but it is not set."});
      return errors;
    }

    if (elem->HasElement("vertical"))
    {
      sdf::ElementPtr subElem = elem->GetElement("vertical");
      if (subElem->HasElement("samples"))
        this->dataPtr->verticalScanSamples = subElem->Get<unsigned int>(
          "samples");
      if (subElem->HasElement("resolution"))
        this->dataPtr->verticalScanResolution = subElem->Get<double>(
          "resolution");
      if (subElem->HasElement("min_angle"))
        this->dataPtr->verticalScanMinAngle = math::Angle(subElem->Get<double>(
          "min_angle"));
      if (subElem->HasElement("max_angle"))
        this->dataPtr->verticalScanMaxAngle = math::Angle(subElem->Get<double>(
          "max_angle"));
    }
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
      "A radar scan element is required, but the scan is not set."});
    return errors;
  }

  if (_sdf->HasElement("range"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("range");
    if (elem->HasElement("min"))
      this->dataPtr->minRange = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->dataPtr->maxRange = elem->Get<double>("max");
    if (elem->HasElement("resolution"))
      this->dataPtr->rangeResolution = elem->Get<double>("resolution");
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
      "A radar range element is required, but the range is not set."});
    return errors;
  }

  if (_sdf->HasElement("noise"))
    this->dataPtr->radarNoise.Load(_sdf->GetElement("noise"));

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr Radar::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
unsigned int Radar::HorizontalScanSamples() const
{
  return this->dataPtr->horizontalScanSamples;
}

//////////////////////////////////////////////////
void Radar::SetHorizontalScanSamples(unsigned int _samples)
{
  this->dataPtr->horizontalScanSamples = _samples;
}

//////////////////////////////////////////////////
double Radar::HorizontalScanResolution() const
{
  return this->dataPtr->horizontalScanResolution;
}

//////////////////////////////////////////////////
void Radar::SetHorizontalScanResolution(double _res)
{
  this->dataPtr->horizontalScanResolution = _res;
}

//////////////////////////////////////////////////
math::Angle Radar::HorizontalScanMinAngle() const
{
  return this->dataPtr->horizontalScanMinAngle;
}

//////////////////////////////////////////////////
void Radar::SetHorizontalScanMinAngle(const math::Angle &_min)
{
  this->dataPtr->horizontalScanMinAngle = _min;
}

//////////////////////////////////////////////////
math::Angle Radar::HorizontalScanMaxAngle() const
{
  return this->dataPtr->horizontalScanMaxAngle;
}

//////////////////////////////////////////////////
void Radar::SetHorizontalScanMaxAngle(const math::Angle &_max)
{
  this->dataPtr->horizontalScanMaxAngle = _max;
}

//////////////////////////////////////////////////
unsigned int Radar::VerticalScanSamples() const
{
  return this->dataPtr->verticalScanSamples;
}

//////////////////////////////////////////////////
void Radar::SetVerticalScanSamples(unsigned int _samples)
{
  this->dataPtr->verticalScanSamples = _samples;
}

//////////////////////////////////////////////////
double Radar::VerticalScanResolution() const
{
  return this->dataPtr->verticalScanResolution;
}

//////////////////////////////////////////////////
void Radar::SetVerticalScanResolution(double _res)
{
  this->dataPtr->verticalScanResolution = _res;
}

//////////////////////////////////////////////////
math::Angle Radar::VerticalScanMinAngle() const
{
  return this->dataPtr->verticalScanMinAngle;
}

//////////////////////////////////////////////////
void Radar::SetVerticalScanMinAngle(const math::Angle &_min)
{
  this->dataPtr->verticalScanMinAngle = _min;
}

//////////////////////////////////////////////////
math::Angle Radar::VerticalScanMaxAngle() const
{
  return this->dataPtr->verticalScanMaxAngle;
}

//////////////////////////////////////////////////
void Radar::SetVerticalScanMaxAngle(const math::Angle &_max)
{
  this->dataPtr->verticalScanMaxAngle = _max;
}

//////////////////////////////////////////////////
double Radar::RangeMin() const
{
  return this->dataPtr->minRange;
}

//////////////////////////////////////////////////
void Radar::SetRangeMin(double _min)
{
  this->dataPtr->minRange = _min;
}

//////////////////////////////////////////////////
double Radar::RangeMax() const
{
  return this->dataPtr->maxRange;
}

//////////////////////////////////////////////////
void Radar::SetRangeMax(double _max)
{
  this->dataPtr->maxRange = _max;
}

//////////////////////////////////////////////////
double Radar::RangeResolution() const
{
  return this->dataPtr->rangeResolution;
}

//////////////////////////////////////////////////
void Radar::SetRangeResolution(double _range)
{
  this->dataPtr->rangeResolution = _range;
}

//////////////////////////////////////////////////
const Noise &Radar::RadarNoise() const
{
  return this->dataPtr->radarNoise;
}

//////////////////////////////////////////////////
void Radar::SetRadarNoise(const Noise &_noise)
{
  this->dataPtr->radarNoise = _noise;
}

int Radar::HMeasures() const 
{
  return this->dataPtr->h_measures;
}
    
void Radar::SetHMeasures(int h_measures) 
{
  this->dataPtr->h_measures = h_measures;
}

int Radar::VMeasures() const 
{
  return this->dataPtr->v_measures;
}
    
void Radar::SetVMeasures(int v_measures) 
{
  this->dataPtr->v_measures = v_measures;
}
//////////////////////////////////////////////////
bool Radar::operator==(const Radar &_radar) const
{
  if (this->dataPtr->horizontalScanSamples != _radar.HorizontalScanSamples())
    return false;
  if (std::abs(this->dataPtr->horizontalScanResolution -
    _radar.HorizontalScanResolution()) > 1e-6)
    return false;
  if (this->dataPtr->horizontalScanMinAngle != _radar.HorizontalScanMinAngle())
    return false;
  if (this->dataPtr->horizontalScanMaxAngle != _radar.HorizontalScanMaxAngle())
    return false;
  if (this->dataPtr->verticalScanSamples != _radar.VerticalScanSamples())
    return false;
  if (std::abs(this->dataPtr->verticalScanResolution -
    _radar.VerticalScanResolution()) > 1e-6)
    return false;
  if (this->dataPtr->verticalScanMinAngle != _radar.VerticalScanMinAngle())
    return false;
  if (this->dataPtr->verticalScanMaxAngle != _radar.VerticalScanMaxAngle())
    return false;
  if (std::abs(this->dataPtr->minRange - _radar.RangeMin()) > 1e-6)
    return false;
  if (std::abs(this->dataPtr->maxRange - _radar.RangeMax()) > 1e-6)
    return false;
  if (std::abs(this->dataPtr->rangeResolution -
        _radar.RangeResolution()) > 1e-6)
  {
    return false;
  }
  if (this->dataPtr->radarNoise != _radar.RadarNoise())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool Radar::operator!=(const Radar &_radar) const
{
  return !(*this == _radar);
}

/////////////////////////////////////////////////
sdf::ElementPtr Radar::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("radar.sdf", elem);

  sdf::ElementPtr scanElem = elem->GetElement("scan");
  sdf::ElementPtr horElem = scanElem->GetElement("horizontal");
  horElem->GetElement("samples")->Set<double>(this->HorizontalScanSamples());
  horElem->GetElement("resolution")->Set<double>(
      this->HorizontalScanResolution());
  horElem->GetElement("min_angle")->Set<double>(
      this->HorizontalScanMinAngle().Radian());
  horElem->GetElement("max_angle")->Set<double>(
      this->HorizontalScanMaxAngle().Radian());

  sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
  vertElem->GetElement("samples")->Set<double>(this->VerticalScanSamples());
  vertElem->GetElement("resolution")->Set<double>(
      this->VerticalScanResolution());
  vertElem->GetElement("min_angle")->Set<double>(
      this->VerticalScanMinAngle().Radian());
  vertElem->GetElement("max_angle")->Set<double>(
      this->VerticalScanMaxAngle().Radian());

  sdf::ElementPtr rangeElem = elem->GetElement("range");
  rangeElem->GetElement("min")->Set<double>(this->RangeMin());
  rangeElem->GetElement("max")->Set<double>(this->RangeMax());
  rangeElem->GetElement("resolution")->Set<double>(
      this->RangeResolution());

  sdf::ElementPtr noiseElem = elem->GetElement("noise");
  std::string noiseType;
  switch (this->dataPtr->radarNoise.Type())
  {
    case sdf::NoiseType::NONE:
      noiseType = "none";
      break;
    case sdf::NoiseType::GAUSSIAN:
      noiseType = "gaussian";
      break;
    case sdf::NoiseType::GAUSSIAN_QUANTIZED:
      noiseType = "gaussian_quantized";
      break;
    default:
      noiseType = "none";
  }

  // radar does not use noise.sdf description
  noiseElem->GetElement("type")->Set<std::string>(noiseType);
  noiseElem->GetElement("mean")->Set<double>(this->dataPtr->radarNoise.Mean());
  noiseElem->GetElement("stddev")->Set<double>(
      this->dataPtr->radarNoise.StdDev());

  return elem;
}
