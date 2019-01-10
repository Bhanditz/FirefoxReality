/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Cylinder.h"
#include "VRLayer.h"
#include "VRLayerNode.h"
#include "vrb/ConcreteClass.h"

#include "vrb/Color.h"
#include "vrb/CreationContext.h"
#include "vrb/Matrix.h"
#include "vrb/Geometry.h"
#include "vrb/RenderState.h"
#include "vrb/SurfaceTextureFactory.h"
#include "vrb/TextureSurface.h"
#include "vrb/Toggle.h"
#include "vrb/Transform.h"
#include "vrb/Vector.h"
#include "vrb/VertexArray.h"

namespace crow {

struct Cylinder::State {
  vrb::CreationContextWeak context;
  VRLayerCylinderPtr layer;
  VRLayerNodePtr layerNode;
  int32_t textureWidth;
  int32_t textureHeight;
  vrb::TogglePtr root;
  vrb::TransformPtr transform;
  vrb::GeometryPtr geometry;
  vrb::Vector worldMin;
  vrb::Vector worldMax;

  State()
      : textureWidth(0)
      , textureHeight(0)
      , worldMin(0.0f, 0.0f, 0.0f)
      , worldMax(0.0f, 0.0f, 0.0f)
  {}

  void Initialize() {
    vrb::CreationContextPtr create = context.lock();
    transform = vrb::Transform::Create(create);
    if (layer) {
      textureWidth = layer->GetWidth();
      textureHeight = layer->GetHeight();
      layer->SetWorldSize(GetWorldWidth(), GetWorldHeight());
      layerNode = VRLayerNode::Create(create, layer);
      transform->AddNode(layerNode);
    } else {
      // TODO: Create cylinder geometry
      VRB_LOG("Cylinder geometry not implemented yet");
    }
    root = vrb::Toggle::Create(create);
    root->AddNode(transform);
  }

  float GetWorldWidth() const {
    return worldMax.x() - worldMin.x();
  }

  float GetWorldHeight() const {
    return worldMax.y() - worldMin.y();
  }
};

CylinderPtr
Cylinder::Create(vrb::CreationContextPtr aContext, const vrb::Vector& aMin, const vrb::Vector& aMax, const VRLayerCylinderPtr& aLayer) {
  CylinderPtr result = std::make_shared<vrb::ConcreteClass<Cylinder, Cylinder::State> >(aContext);
  result->m.worldMin = aMin;
  result->m.worldMax = aMax;
  result->m.layer = aLayer;
  result->m.Initialize();
  return result;
}

CylinderPtr
Cylinder::Create(vrb::CreationContextPtr aContext, const float aWorldWidth, const float aWorldHeight, const VRLayerCylinderPtr& aLayer) {
  CylinderPtr result = std::make_shared<vrb::ConcreteClass<Cylinder, Cylinder::State> >(aContext);
  result->m.worldMin = vrb::Vector(-aWorldWidth * 0.5f, -aWorldHeight * 0.5f, 0.0f);
  result->m.worldMax = vrb::Vector(aWorldWidth * 0.5f, aWorldHeight * 0.5f, 0.0f);
  result->m.layer = aLayer;
  result->m.Initialize();
  return result;
}

void
Cylinder::GetTextureSize(int32_t& aWidth, int32_t& aHeight) const {
  aWidth = m.textureWidth;
  aHeight = m.textureHeight;
}

void
Cylinder::SetTextureSize(int32_t aWidth, int32_t aHeight) {
  m.textureWidth = aWidth;
  m.textureHeight = aHeight;
  if (m.layer) {
    m.layer->Resize(aWidth, aHeight);
  }
}

void
Cylinder::GetWorldMinAndMax(vrb::Vector& aMin, vrb::Vector& aMax) const {
  aMin = m.worldMin;
  aMax = m.worldMax;
}

const vrb::Vector&
Cylinder::GetWorldMin() const {
  return m.worldMin;
}

const vrb::Vector&
Cylinder::GetWorldMax() const {
  return m.worldMax;
}

float
Cylinder::GetWorldWidth() const {
  return  m.GetWorldWidth();
}

float
Cylinder::GetWorldHeight() const {
  return m.GetWorldHeight();
}

void
Cylinder::GetWorldSize(float& aWidth, float& aHeight) const {
  aWidth = m.worldMax.x() - m.worldMin.x();
  aHeight = m.worldMax.y() - m.worldMin.y();
}

void
Cylinder::SetWorldSize(const float aWidth, const float aHeight) const {
  vrb::Vector min = vrb::Vector(-aWidth * 0.5f, -aHeight * 0.5f, 0.0f);
  vrb::Vector max = vrb::Vector(aWidth * 0.5f, aHeight * 0.5f, 0.0f);
  SetWorldSize(min, max);
}

void
Cylinder::SetWorldSize(const vrb::Vector& aMin, const vrb::Vector& aMax) const {
  if (m.worldMin == aMin && m.worldMax == aMax) {
    return;
  }
  m.worldMin = aMin;
  m.worldMax = aMax;

  if (m.layer) {
    m.layer->SetWorldSize(GetWorldWidth(), GetWorldHeight());
  }
}

void
Cylinder::SetTintColor(const vrb::Color& aColor) {
  if (m.layer) {
    m.layer->SetTintColor(aColor);
  } else if (m.geometry && m.geometry->GetRenderState()) {
    m.geometry->GetRenderState()->SetTintColor(aColor);
  }
}

vrb::Vector
Cylinder::GetCenterNormal() const {
  const vrb::Vector bottomRight(m.worldMax.x(), m.worldMin.y(), m.worldMin.z());
  return (bottomRight - m.worldMin).Cross(m.worldMax - m.worldMin).Normalize();
}

vrb::NodePtr
Cylinder::GetRoot() const {
  return m.root;
}

VRLayerCylinderPtr
Cylinder::GetLayer() const {
  return m.layer;
}

vrb::TransformPtr
Cylinder::GetTransformNode() const {
  return m.transform;
}
static const float kEpsilon = 0.00000001f;

bool
Cylinder::TestIntersection(const vrb::Vector& aStartPoint, const vrb::Vector& aDirection, vrb::Vector& aResult, bool aClamp, bool& aIsInside, float& aDistance) const {
  aDistance = -1.0f;
  if (!m.root->IsEnabled(*m.transform)) {
    return false;
  }
  vrb::Matrix modelView = m.transform->GetWorldTransform().AfineInverse();
  vrb::Vector point = modelView.MultiplyPosition(aStartPoint);
  vrb::Vector direction = modelView.MultiplyDirection(aDirection);
  vrb::Vector normal = GetCenterNormal();
  const float dotNormals = direction.Dot(normal);
  if (dotNormals > -kEpsilon) {
    // Not pointed at the plane
    return false;
  }

  const float dotV = (m.worldMin - point).Dot(normal);

  if ((dotV < kEpsilon) && (dotV > -kEpsilon)) {
    return false;
  }

  const float length = dotV / dotNormals;
  vrb::Vector result = point + (direction * length);

  if ((result.x() >= m.worldMin.x()) && (result.y() >= m.worldMin.y()) &&(result.z() >= (m.worldMin.z() - 0.1f)) &&
      (result.x() <= m.worldMax.x()) && (result.y() <= m.worldMax.y()) &&(result.z() <= (m.worldMax.z() + 0.1f))) {

    aIsInside = true;
  }

  aResult = result;

  aDistance = (aResult - point).Magnitude();

  // Clamp to keep pointer in Cylinder.
  if (aClamp) {
    if (result.x() > m.worldMax.x()) { result.x() = m.worldMax.x(); }
    else if (result.x() < m.worldMin.x()) { result.x() = m.worldMin.x(); }

    if (result.y() > m.worldMax.y()) { result.y() = m.worldMax.y(); }
    else if (result.y() < m.worldMin.y()) { result.y() = m.worldMin.y(); }
  }

  return true;
}

void
Cylinder::ConvertToQuadCoordinates(const vrb::Vector& point, float& aX, float& aY, bool aClamp) const {
  vrb::Vector value = point;
  // Clamp value to Cylinder bounds.
  if (aClamp) {
    if (value.x() > m.worldMax.x()) { value.x() = m.worldMax.x(); }
    else if (value.x() < m.worldMin.x()) { value.x() = m.worldMin.x(); }
    if (value.y() > m.worldMax.y()) { value.y() = m.worldMax.y(); }
    else if (value.y() < m.worldMin.y()) { value.y() = m.worldMin.y(); }
  }

  // Convert to Cylinder coordinates.
  aX = (((value.x() - m.worldMin.x()) / (m.worldMax.x() - m.worldMin.x())) * (float)m.textureWidth);
  aY = (((m.worldMax.y() - value.y()) / (m.worldMax.y() - m.worldMin.y())) * (float)m.textureHeight);
}

Cylinder::Cylinder(State& aState, vrb::CreationContextPtr& aContext) : m(aState) {
  m.context = aContext;
}

Cylinder::~Cylinder() {}

} // namespace crow
