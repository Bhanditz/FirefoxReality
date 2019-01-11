/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Widget.h"
#include "Cylinder.h"
#include "Quad.h"
#include "VRLayer.h"
#include "VRBrowser.h"
#include "WidgetPlacement.h"
#include "WidgetResizer.h"
#include "vrb/ConcreteClass.h"

#include "vrb/Color.h"
#include "vrb/RenderContext.h"
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

struct Widget::State {
  vrb::RenderContextWeak context;
  std::string name;
  uint32_t handle;
  QuadPtr quad;
  CylinderPtr cylinder;
  vrb::TogglePtr root;
  vrb::TransformPtr transform;
  vrb::TextureSurfacePtr surface;
  WidgetPlacementPtr placement;
  WidgetResizerPtr resizer;
  bool resizing;
  bool toggleState;

  State()
      : handle(0)
      , resizing(false)
      , toggleState(false)
  {}

  void Initialize(const int aHandle, const int32_t aTextureWidth, const int32_t aTextureHeight,
                  const QuadPtr& aQuad, const CylinderPtr& aCylinder) {
    handle = aHandle;
    name = "crow::Widget-" + std::to_string(handle);
    vrb::RenderContextPtr render = context.lock();
    if (!render) {
      return;
    }

    quad = aQuad;
    cylinder = aCylinder;

    VRLayerPtr layer = GetLayer();

    if (layer) {
      layer->SetSurfaceChangedDelegate([=](const VRLayer& aLayer, VRLayer::SurfaceChange aChange, const std::function<void()>& aCallback) {
        const VRLayerQuad& layerQuad = static_cast<const VRLayerQuad&>(aLayer);
        VRBrowser::DispatchCreateWidgetLayer((jint)aHandle, layerQuad.GetSurface(), layerQuad.GetWidth(), layerQuad.GetHeight(), aCallback);
      });
    } else {
      surface = vrb::TextureSurface::Create(render, name);
      if (quad) {
        quad->SetTexture(surface, aTextureWidth, aTextureHeight);
        quad->SetMaterial(vrb::Color(0.4f, 0.4f, 0.4f), vrb::Color(1.0f, 1.0f, 1.0f), vrb::Color(0.0f, 0.0f, 0.0f), 0.0f);
      }
    }

    vrb::CreationContextPtr create = render->GetRenderThreadCreationContext();
    transform = vrb::Transform::Create(create);
    if (quad) {
      transform->AddNode(quad->GetRoot());
    } else {
      transform->AddNode(cylinder->GetRoot());
    }
    root = vrb::Toggle::Create(create);
    root->AddNode(transform);

    if (layer) {
      toggleState = true;
      root->ToggleAll(true);
    } else {
      root->ToggleAll(false);
    }
  }

  bool FirstDraw() {
    if (!placement) {
      return false;
    }
    return placement->firstDraw;
  }

  const VRLayerSurfacePtr GetLayer() {
    return quad ? (VRLayerSurfacePtr) quad->GetLayer() : cylinder->GetLayer();
  }
};

WidgetPtr
Widget::Create(vrb::RenderContextPtr& aContext, const int aHandle,
               const int32_t aTextureWidth, const int32_t aTextureHeight,const QuadPtr& aQuad) {
  WidgetPtr result = std::make_shared<vrb::ConcreteClass<Widget, Widget::State> >(aContext);
  result->m.Initialize(aHandle, aTextureWidth, aTextureHeight, aQuad, nullptr);
  return result;
}

WidgetPtr
Widget::Create(vrb::RenderContextPtr& aContext, const int aHandle,
               const int32_t aTextureWidth, const int32_t aTextureHeight, const CylinderPtr& aCylinder) {
  WidgetPtr result = std::make_shared<vrb::ConcreteClass<Widget, Widget::State> >(aContext);
  result->m.Initialize(aHandle, aTextureWidth, aTextureHeight, nullptr, aCylinder);
  return result;
}

uint32_t
Widget::GetHandle() const {
  return m.handle;
}

void
Widget::ResetFirstDraw() {
  if (m.placement) {
    m.placement->firstDraw = false;
  }
  if (m.root) {
    m.root->ToggleAll(false);
  }
}

const std::string&
Widget::GetSurfaceTextureName() const {
  return m.name;
}

const vrb::TextureSurfacePtr
Widget::GetSurfaceTexture() const {
  return m.surface;
}

void
Widget::GetSurfaceTextureSize(int32_t& aWidth, int32_t& aHeight) const {
  if (m.quad) {
    m.quad->GetTextureSize(aWidth, aHeight);
  } else {
    m.cylinder->GetTextureSize(aWidth, aHeight);
  }
}

void
Widget::SetSurfaceTextureSize(int32_t aWidth, int32_t aHeight) {
  if (m.quad) {
    m.quad->SetTextureSize(aWidth, aHeight);
  } else {
    m.cylinder->SetTextureSize(aWidth, aHeight);
  }
}

void
Widget::GetWidgetMinAndMax(vrb::Vector& aMin, vrb::Vector& aMax) const {
  if (m.quad) {
    m.quad->GetWorldMinAndMax(aMin, aMax);
  } else {
    m.cylinder->GetWorldMinAndMax(aMin, aMax);
  }
}

void
Widget::SetWorldWidth(float aWorldWidth) const {
  int32_t width, height;
  this->GetSurfaceTextureSize(width, height);
  const float aspect = (float)width / (float) height;
  const float worldHeight = aWorldWidth / aspect;
  if (m.quad) {
    m.quad->SetWorldSize(aWorldWidth, worldHeight);
  } else {
    m.cylinder->SetWorldSize(aWorldWidth, worldHeight);
  }
  if (m.resizing && m.resizer) {
    vrb::Vector min(-aWorldWidth * 0.5f, -worldHeight * 0.5f, 0.0f);
    vrb::Vector max(aWorldWidth *0.5f, worldHeight * 0.5f, 0.0f);
    m.resizer->SetSize(min, max);
  }
}

void
Widget::GetWorldSize(float& aWidth, float& aHeight) const {
  if (m.quad) {
    m.quad->GetWorldSize(aWidth, aHeight);
  } else {
    m.cylinder->GetWorldSize(aWidth, aHeight);
  }
}

bool
Widget::TestControllerIntersection(const vrb::Vector& aStartPoint, const vrb::Vector& aDirection, vrb::Vector& aResult, bool& aIsInWidget, float& aDistance) const {
  aDistance = -1.0f;
  if (!m.root->IsEnabled(*m.transform)) {
    return false;
  }

  bool clamp = !m.resizing;
  bool result = false;
  if (m.quad) {
    result = m.quad->TestIntersection(aStartPoint, aDirection, aResult, clamp, aIsInWidget, aDistance);
  } else {
    result = m.cylinder->TestIntersection(aStartPoint, aDirection, aResult, clamp, aIsInWidget, aDistance);
  }
  if (result && m.resizing && !aIsInWidget) {
    // Handle extra intersections while resizing
    aIsInWidget = m.resizer->TestIntersection(aResult);
  }

  return result;
}

void
Widget::ConvertToWidgetCoordinates(const vrb::Vector& point, float& aX, float& aY) const {
  bool clamp = !m.resizing;
  if (m.quad) {
    m.quad->ConvertToQuadCoordinates(point, aX, aY, clamp);
  } else {
    m.cylinder->ConvertToQuadCoordinates(point, aX, aY, clamp);
  }
}

void
Widget::ConvertToWorldCoordinates(const vrb::Vector& aPoint, vrb::Vector& aResult) const {
  aResult = m.transform->GetTransform().MultiplyPosition(aPoint);
}

const vrb::Matrix
Widget::GetTransform() const {
  return m.transform->GetTransform();
}

void
Widget::SetTransform(const vrb::Matrix& aTransform) {
  m.transform->SetTransform(aTransform);
}

void
Widget::ToggleWidget(const bool aEnabled) {
  m.toggleState = aEnabled;
  m.root->ToggleAll(aEnabled && m.FirstDraw());
}

bool
Widget::IsVisible() const {
  return m.toggleState;
}


vrb::NodePtr
Widget::GetRoot() const {
  return m.root;
}

QuadPtr
Widget::GetQuad() const {
  return m.quad;
}

CylinderPtr
Widget::GetCylinder() const {
  return m.cylinder;
}

VRLayerSurfacePtr
Widget::GetLayer() const {
  return m.GetLayer();
}

vrb::TransformPtr
Widget::GetTransformNode() const {
  return m.transform;
}

const WidgetPlacementPtr&
Widget::GetPlacement() const {
  return m.placement;
}

void
Widget::SetPlacement(const WidgetPlacementPtr& aPlacement) {
  if (!m.FirstDraw() && aPlacement && aPlacement->firstDraw && m.root) {
      m.root->ToggleAll(m.toggleState);
  }
  m.placement = aPlacement;
  VRLayerSurfacePtr layer = GetLayer();
  if (layer) {
    layer->SetPixelDensity(aPlacement->density);
  }
}

void
Widget::StartResize() {
  vrb::Vector worldMin, worldMax;
  GetWidgetMinAndMax(worldMin, worldMax);
  if (m.resizer) {
    m.resizer->SetSize(worldMin, worldMax);
  } else {
    vrb::RenderContextPtr render = m.context.lock();
    if (!render) {
      return;
    }
    vrb::CreationContextPtr create = render->GetRenderThreadCreationContext();
    m.resizer = WidgetResizer::Create(create, worldMin, worldMax);
    m.transform->InsertNode(m.resizer->GetRoot(), 0);
  }
  m.resizing = true;
  m.resizer->ToggleVisible(true);
  if (m.quad) {
    m.quad->SetScaleMode(Quad::ScaleMode::AspectFit);
    m.quad->SetBackgroundColor(vrb::Color(1.0f, 1.0f, 1.0f, 1.0f));
  }
}

void
Widget::FinishResize() {
  if (!m.resizing) {
    return;
  }
  m.resizing = false;
  m.resizer->ToggleVisible(false);
  if (m.quad) {
    m.quad->SetScaleMode(Quad::ScaleMode::Fill);
    m.quad->SetBackgroundColor(vrb::Color(0.0f, 0.0f, 0.0f, 0.0f));
  }
}

bool
Widget::IsResizing() const {
  return m.resizing;
}

void
Widget::HandleResize(const vrb::Vector& aPoint, bool aPressed, bool& aResized, bool &aResizeEnded) {
  m.resizer->HandleResizeGestures(aPoint, aPressed, aResized, aResizeEnded);
  if (aResized || aResizeEnded) {
    if (m.quad) {
      m.quad->SetWorldSize(m.resizer->GetCurrentMin(), m.resizer->GetCurrentMax());
    } else {
      m.cylinder->SetWorldSize(m.resizer->GetCurrentMin(), m.resizer->GetCurrentMax());
    }
  }
}

void
Widget::HoverExitResize() {
  if (m.resizing) {
    m.resizer->HoverExitResize();
  }
}

Widget::Widget(State& aState, vrb::RenderContextPtr& aContext) : m(aState) {
  m.context = aContext;
}

Widget::~Widget() {}

} // namespace crow
