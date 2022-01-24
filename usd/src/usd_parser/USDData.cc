/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include "sdf/usd/usd_parser/USDData.hh"

#include <string>
#include <set>
#include <unordered_map>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/primCompositionQuery.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#include "sdf/usd/usd_parser/utils.hh"
#include "sdf/Material.hh"

namespace sdf {
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd {
  /// \brief USDStage private data.
  class USDData::Implementation
  {
    public:
      /// File name of the main stage
      std::string filename;

      /// Directory where the main stage is located
      std::string directoryPath;

      /// map with the filename and its own Stage data
      std::unordered_map<std::string, std::shared_ptr<USDStage>> references;

      /// Models
      std::set<std::string> models;

      /// Materials availables in the stage and substages
      std::unordered_map<std::string, sdf::Material> materials;

      /// Add all subdirectories that are inside the stage folder
      /// This will help us to find other stages and/or textures.
      /// \param[in] _path Path of the subdirectory to add
      void AddSubdirectories(const std::string &_path)
      {
        for (ignition::common::DirIter file(_path);
          file != ignition::common::DirIter(); ++file)
        {
          std::string current(*file);

          if (ignition::common::isDirectory(current))
          {
            auto systemPaths = ignition::common::systemPaths();
            systemPaths->AddFilePaths(current);
            this->AddSubdirectories(current);
          }
        }
      }
  };

  /////////////////////////////////////////////////
  USDData::USDData(const std::string &_filename)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
  {
    this->dataPtr->filename = _filename;

    // Add the base stage to the structure
    this->dataPtr->references.insert(
      {_filename,
       std::make_shared<USDStage>(_filename)
      });
  }

  /////////////////////////////////////////////////
  const std::unordered_map<std::string, sdf::Material> &
    USDData::GetMaterials() const
  {
    return this->dataPtr->materials;
  }

  /////////////////////////////////////////////////
  const std::set<std::string> &USDData::GetModels() const
  {
    return this->dataPtr->models;
  }

  /////////////////////////////////////////////////
  const std::unordered_map<std::string, std::shared_ptr<USDStage>> &
    USDData::GetAllReferences() const
  {
    return this->dataPtr->references;
  }

  /////////////////////////////////////////////////
  bool USDData::Init()
  {
    // it's the stage available
    auto referencee = pxr::UsdStage::Open(this->dataPtr->filename);
    if (!referencee)
    {
      return false;
    }

    this->dataPtr->directoryPath = ignition::common::absPath(
      this->dataPtr->filename);

    std::string basename = ignition::common::basename(
      this->dataPtr->directoryPath);
    this->dataPtr->directoryPath = removeSubStr(
      this->dataPtr->directoryPath, basename);

    this->dataPtr->AddSubdirectories(this->dataPtr->directoryPath);

    auto range = pxr::UsdPrimRange::Stage(referencee);

    for (auto const &_prim : range)
    {
      if (_prim.IsA<pxr::UsdShadeMaterial>())
      {
        continue;
      }

      if (_prim.IsA<pxr::UsdPhysicsScene>())
      {
        continue;
      }

      std::string primName = pxr::TfStringify(_prim.GetPath());
      std::vector<std::string> tokens = ignition::common::split(primName, "/");
      if (tokens.size() == 1)
      {
        this->dataPtr->models.insert(tokens[0]);
      }

      pxr::UsdPrimCompositionQuery query =
        pxr::UsdPrimCompositionQuery::GetDirectReferences(_prim);

      std::vector<pxr::UsdPrimCompositionQueryArc> arcs =
        query.GetCompositionArcs();
      for (auto & a : arcs )
      {
        pxr::SdfLayerHandle handler = a.GetIntroducingLayer();

        for (auto & ref : handler->GetCompositionAssetDependencies())
        {
          this->AddStage(ref);
        }
      }
    }

    return true;
  }

  /////////////////////////////////////////////////
  int USDData::ParseMaterials()
  {
    auto referencee = pxr::UsdStage::Open(this->dataPtr->filename);
    if (!referencee)
    {
      return false;
    }

    auto range = pxr::UsdPrimRange::Stage(referencee);

    for (auto const &prim : range)
    {
      std::string primName = pxr::TfStringify(prim.GetPath());
      if (prim.IsA<pxr::UsdShadeMaterial>())
      {
        std::string materialName = prim.GetName();

        if (this->dataPtr->materials.find(materialName)
            != this->dataPtr->materials.end())
        {
          continue;
        }

        sdf::Material material = ParseMaterial(prim);
        this->dataPtr->materials.insert(std::pair<std::string, sdf::Material>(
          materialName, material));
      }
    }
    return this->dataPtr->materials.size();
  }

  /////////////////////////////////////////////////
  const std::pair<std::string, std::shared_ptr<USDStage>>
    USDData::findStage(const std::string &_name)
  {
    for (auto &ref : this->dataPtr->references)
    {
      if (ref.second != nullptr)
      {
        for (auto &path : ref.second->GetUSDPaths())
        {
          if (path == _name)
          {
            return ref;
          }
        }
      }
    }
    return std::make_pair<std::string, std::shared_ptr<USDStage>>("", nullptr);
  }

  /////////////////////////////////////////////////
  bool USDData::AddStage(const std::string &_ref)
  {
    std::string key = _ref;

    auto search = this->dataPtr->references.find(_ref);
    if (search == this->dataPtr->references.end())
    {
      std::string basename = ignition::common::basename(key);
      std::string subDirectory = removeSubStr(key, basename);

      auto systemPaths = ignition::common::systemPaths();
      systemPaths->AddFilePaths(ignition::common::joinPaths(
        this->dataPtr->directoryPath, subDirectory));

      this->dataPtr->AddSubdirectories(ignition::common::joinPaths(
        this->dataPtr->directoryPath, subDirectory));

      std::string fileNameRef = ignition::common::findFile(basename);
      if (fileNameRef.empty())
      {
        std::cerr << "Not able to find asset [" << _ref << "]" << '\n';
        return false;
      }

      this->dataPtr->references.insert(
        {key,
         std::make_shared<USDStage>(fileNameRef)
        });
      return true;
    }
    return false;
  }
}
}
}
