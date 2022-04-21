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
#ifndef SDF_RADAR_HH_
#define SDF_RADAR_HH_

#include <ignition/math/Angle.hh>
#include <ignition/utils/ImplPtr.hh>

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>


namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// TODO: DAT Update documentation below

  /// \brief Radar contains information about a Radar sensor.
  /// This sensor can be attached to a link. The Radar sensor can be defined
  /// SDF XML using either the "ray" or "Radar" types. The "radar" type is
  /// preffered as "ray" is considered legacy.
  ///
  /// # Example SDF XML using radar type:
  ///
  /// ~~~{.xml}
  /// <sensor name="radar_sensor" type="radar">
  ///     <pose>1 2 3 0 0 0</pose>
  ///     <radar>
  ///         <scan>
  ///             <horizontal>
  ///                 <samples>320</samples>
  ///                 <resolution>0.9</resolution>
  ///                 <min_angle>1.75</min_angle>
  ///                 <max_angle>2.94</max_angle>
  ///             </horizontal>
  ///             <vertical>
  ///                 <samples>240</samples>
  ///                 <resolution>0.8</resolution>
  ///                 <min_angle>2.75</min_angle>
  ///                 <max_angle>3.94</max_angle>
  ///             </vertical>
  ///         </scan>
  ///         <range>
  ///             <min>1.23</min>
  ///             <max>4.56</max>
  ///             <resolution>7.89</resolution>
  ///         </range>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///         <h_measures>
  ///           6
  ///         </h_measures>
  ///         <v_measures>
  ///           4
  ///         </v_measures>
  ///     </radar>
  /// </sensor>
  ///
  /// ~~~
  class SDFORMAT_VISIBLE Radar
  {
    /// \brief Default constructor
    public: Radar();

    /// \brief Load the radar based on an element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the number of radar rays horizontally to generate per laser
    /// sweep.
    /// \return Number of radar rays horizontally per laser sweep.
    public: unsigned int HorizontalScanSamples() const;

    /// \brief Set the number of radar rays horizontally to generate per laser
    /// sweep.
    /// \param[in] Number of radar rays horizontally per laser sweep.
    public: void SetHorizontalScanSamples(unsigned int _samples);

    /// \brief Get the resolution for horizontal scan.
    /// \return Resolution for horizontal scan.
    public: double HorizontalScanResolution() const;

    /// \brief Set the resolution for horizontal scan.
    /// \param[in] Resolution for horizontal scan.
    public: void SetHorizontalScanResolution(double _res);

    /// \brief Get the minimum angle for horizontal scan.
    /// \return Minimum angle for horizontal scan.
    public: ignition::math::Angle HorizontalScanMinAngle() const;

    /// \brief Set the minimum angle for horizontal scan.
    /// \param[in] Minimum angle for horizontal scan.
    public: void SetHorizontalScanMinAngle(const ignition::math::Angle &_min);

    /// \brief Get the maximum angle for horizontal scan.
    /// \return Maximum angle for horizontal scan.
    public: ignition::math::Angle HorizontalScanMaxAngle() const;

    /// \brief Set the maximum angle for horizontal scan.
    /// \param[in] Maximum angle for horizontal scan.
    public: void SetHorizontalScanMaxAngle(const ignition::math::Angle &_max);

    /// \brief Get the number of radar rays vertically to generate per laser
    /// sweep.
    /// \return Number of radar rays vertically per laser sweep.
    public: unsigned int VerticalScanSamples() const;

    /// \brief Set the number of radar rays vertically to generate per laser
    /// sweep.
    /// \param[in] Number of radar rays vertically per laser sweep.
    public: void SetVerticalScanSamples(unsigned int _samples);

    /// \brief Get the resolution for vertical scan.
    /// \return Resolution for vertical scan.
    public: double VerticalScanResolution() const;

    /// \brief Set the resolution for vertical scan.
    /// \param[in] Resolution for vertical scan.
    public: void SetVerticalScanResolution(double _res);

    /// \brief Get the minimum angle for vertical scan.
    /// \return Minimum angle for vertical scan.
    public: ignition::math::Angle VerticalScanMinAngle() const;

    /// \brief Set the minimum angle for vertical scan.
    /// \param[in] Minimum angle for vertical scan.
    public: void SetVerticalScanMinAngle(const ignition::math::Angle &_min);

    /// \brief Get the maximum angle for vertical scan.
    /// \return Maximum angle for vertical scan.
    public: ignition::math::Angle VerticalScanMaxAngle() const;

    /// \brief Set the maximum angle for vertical scan.
    /// \param[in] Maximum angle for vertical scan.
    public: void SetVerticalScanMaxAngle(const ignition::math::Angle &_max);

    /// \brief Get minimum distance for each radar ray.
    /// \return Minimum distance for each radar ray.
    public: double RangeMin() const;

    /// \brief Set minimum distance for each radar ray.
    /// \param[in] Minimum distance for each radar ray.
    public: void SetRangeMin(double _min);

    /// \brief Get maximum distance for each radar ray.
    /// \return Maximum distance for each radar ray.
    public: double RangeMax() const;

    /// \brief Set maximum distance for each radar ray.
    /// \param[in] Maximum distance for each radar ray.
    public: void SetRangeMax(double _max);

    /// \brief Get linear resolution of each radar ray.
    /// \return Linear resolution for each radar ray.
    public: double RangeResolution() const;

    /// \brief Set linear resolution of each radar ray.
    /// \param[in] Linear resolution for each radar ray.
    public: void SetRangeResolution(double _range);

    /// \brief Get the noise values for the radar sensor.
    /// \return Noise values for the radar sensor.
    public: const Noise &RadarNoise() const;

    /// \biref Set the noise values for the radar sensor.
    /// \param[in] _noise Noise values for the radar sensor.
    public: void SetRadarNoise(const Noise &_noise);

    public: int HMeasures() const;
    public: void SetHMeasures(int h_measures);

    public: int VMeasures() const;
    public: void SetVMeasures(int v_measures);

    /// \brief Return true if both Radar objects contain the same values.
    /// \param[_in] _radar Radar value to compare.
    /// \return True if 'this' == _radar.
    public: bool operator==(const Radar &_radar) const;

    /// \brief Return true this Radar object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _radar Radar value to compare.
    /// \return True if 'this' != _radar.
    public: bool operator!=(const Radar &_radar) const;

    /// \brief Create and return an SDF element filled with data from this
    /// radar.
    /// \return SDF element pointer with updated sensor values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
