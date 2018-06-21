/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef VRBROWSER_QUAD_DOT_H
#define VRBROWSER_QUAD_DOT_H

#include "vrb/Forward.h"
#include "vrb/MacroUtils.h"

#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace crow {

class Quad;
typedef std::shared_ptr<Quad> QuadPtr;

class Quad {
public:
  enum class ScaleMode {
    Fill,
    AspectFit,
    AspectFill
  };
  static QuadPtr Create(vrb::ContextWeak aContext, const vrb::Vector& aMin, const vrb::Vector& aMax);
  static vrb::GeometryPtr CreateGeometry(vrb::ContextWeak aContext, const vrb::Vector& aMin, const vrb::Vector& aMax);
  void SetTexture(const vrb::TextureSurfacePtr& aTexture, int32_t aWidth, int32_t aHeight);
  void SetMaterial(const vrb::Color& aAmbient, const vrb::Color& aDiffuse, const vrb::Color& aSpecular, const float aSpecularExponent);
  void GetTextureSize(int32_t& aWidth, int32_t& aHeight) const;
  void GetWorldMinAndMax(vrb::Vector& aMin, vrb::Vector& aMax) const;
  const vrb::Vector& GetWorldMin() const;
  const vrb::Vector& GetWorldMax() const;
  void GetWorldSize(float& aWidth, float& aHeight) const;
  vrb::Vector GetNormal() const;
  vrb::NodePtr GetRoot() const;
  vrb::TransformPtr GetTransformNode() const;
  bool TestIntersection(const vrb::Vector& aStartPoint, const vrb::Vector& aDirection, vrb::Vector& aResult, bool aClamp, bool& aIsInside, float& aDistance) const;
  void ConvertToQuadCoordinates(const vrb::Vector& point, float& aX, float& aY) const;

  struct State;
  Quad(State& aState, vrb::ContextWeak& aContext);
  ~Quad();
private:
  State& m;
  Quad() = delete;
  VRB_NO_DEFAULTS(Quad)
};

} // namespace crow

#endif // VRBROWSER_QUAD_DOT_H
