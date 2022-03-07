/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <ignition/math/Angle.hh>
#include "sdf/Radar.hh"

/////////////////////////////////////////////////
TEST(DOMRadar, Construction)
{
  sdf::Radar radar;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, radar.RadarNoise());
}

/////////////////////////////////////////////////
TEST(DOMRadar, Set)
{
  sdf::Radar radar;

  radar.SetHorizontalScanSamples(123);
  EXPECT_EQ(radar.HorizontalScanSamples(), 123u);
  radar.SetHorizontalScanResolution(0.45);
  EXPECT_DOUBLE_EQ(radar.HorizontalScanResolution(), 0.45);
  radar.SetHorizontalScanMinAngle(ignition::math::Angle(0.67));
  EXPECT_DOUBLE_EQ(*(radar.HorizontalScanMinAngle()), 0.67);
  radar.SetHorizontalScanMaxAngle(ignition::math::Angle(0.89));
  EXPECT_DOUBLE_EQ(*(radar.HorizontalScanMaxAngle()), 0.89);

  radar.SetVerticalScanSamples(98);
  EXPECT_EQ(radar.VerticalScanSamples(), 98u);
  radar.SetVerticalScanResolution(0.76);
  EXPECT_DOUBLE_EQ(radar.VerticalScanResolution(), 0.76);
  radar.SetVerticalScanMinAngle(ignition::math::Angle(0.54));
  EXPECT_DOUBLE_EQ(*(radar.VerticalScanMinAngle()), 0.54);
  radar.SetVerticalScanMaxAngle(ignition::math::Angle(0.321));
  EXPECT_DOUBLE_EQ(*(radar.VerticalScanMaxAngle()), 0.321);

  radar.SetRangeMin(1.2);
  EXPECT_DOUBLE_EQ(radar.RangeMin(), 1.2);
  radar.SetRangeMax(3.4);
  EXPECT_DOUBLE_EQ(radar.RangeMax(), 3.4);
  radar.SetRangeResolution(5.6);
  EXPECT_DOUBLE_EQ(radar.RangeResolution(), 5.6);

  sdf::Noise noise;
  noise.SetMean(6.5);
  noise.SetStdDev(3.79);
  radar.SetRadarNoise(noise);
  EXPECT_EQ(noise, radar.RadarNoise());

  radar.SetHorizontalScanSamples(111);
  radar.SetHorizontalScanResolution(2.2);

  // Inequality operator
  sdf::Radar radar2;
  EXPECT_NE(radar2, radar);

  // Copy constructor
  sdf::Radar radarCopied(radar);
  EXPECT_EQ(radarCopied, radar);

  // Assignment operator
  sdf::Radar radarAssigned;
  radarAssigned = radar;
  EXPECT_EQ(radarAssigned, radar);

  // Move constructor
  sdf::Radar radarMoved = std::move(radar);
  EXPECT_EQ(radarCopied, radarMoved);
  // Test nullptr private class
  radar = radarMoved;
  EXPECT_EQ(radarCopied, radar);

  // Move assignment operator
  sdf::Radar radarMoveAssigned;
  radarMoveAssigned = std::move(radarCopied);
  EXPECT_EQ(radarAssigned, radarMoveAssigned);
  // Test nullptr private class
  radarCopied = radarMoveAssigned;
  EXPECT_EQ(radarAssigned, radarCopied);
}

/////////////////////////////////////////////////
TEST(DOMRadar, Load)
{
  sdf::Radar radar;
  sdf::Errors errors;

  // Null element
  errors = radar.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, radar.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = radar.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, radar.Element());

  // The Radar::Load function is tested more thouroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMRadar, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Radar radar;
  radar.SetHorizontalScanSamples(123);
  radar.SetHorizontalScanResolution(0.45);
  radar.SetHorizontalScanMinAngle(ignition::math::Angle(0.67));
  radar.SetHorizontalScanMaxAngle(ignition::math::Angle(0.89));
  radar.SetVerticalScanSamples(98);
  radar.SetVerticalScanResolution(0.76);
  radar.SetVerticalScanMinAngle(ignition::math::Angle(0.54));
  radar.SetVerticalScanMaxAngle(ignition::math::Angle(0.321));
  radar.SetRangeMin(1.2);
  radar.SetRangeMax(3.4);
  radar.SetRangeResolution(5.6);

  sdf::Noise noise;
  noise.SetMean(6.5);
  noise.SetStdDev(3.79);
  radar.SetRadarNoise(noise);

  sdf::ElementPtr radarElem = radar.ToElement();
  EXPECT_NE(nullptr, radarElem);
  EXPECT_EQ(nullptr, radar.Element());

  // verify values after loading the element back
  sdf::Radar radar2;
  radar2.Load(radarElem);

  EXPECT_EQ(123u, radar2.HorizontalScanSamples());
  EXPECT_DOUBLE_EQ(0.45, radar2.HorizontalScanResolution());
  EXPECT_DOUBLE_EQ(0.67, *(radar2.HorizontalScanMinAngle()));
  EXPECT_DOUBLE_EQ(0.89, *(radar2.HorizontalScanMaxAngle()));
  EXPECT_EQ(98u, radar2.VerticalScanSamples());
  EXPECT_DOUBLE_EQ(0.76, radar2.VerticalScanResolution());
  EXPECT_DOUBLE_EQ(0.54, *(radar2.VerticalScanMinAngle()));
  EXPECT_DOUBLE_EQ(0.321, *(radar2.VerticalScanMaxAngle()));
  EXPECT_DOUBLE_EQ(1.2, radar2.RangeMin());
  EXPECT_DOUBLE_EQ(3.4, radar2.RangeMax());
  EXPECT_DOUBLE_EQ(5.6, radar2.RangeResolution());
  EXPECT_EQ(noise, radar2.RadarNoise());

  // make changes to DOM and verify ToElement produces updated values
  radar2.SetHorizontalScanSamples(111u);
  sdf::ElementPtr radar2Elem = radar2.ToElement();
  EXPECT_NE(nullptr, radar2Elem);
  sdf::Radar radar3;
  radar3.Load(radar2Elem);
  EXPECT_EQ(111u, radar3.HorizontalScanSamples());
}
