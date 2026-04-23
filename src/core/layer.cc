
#include "core/layer.h"

namespace geditor {

Layer::Layer(LayerType id)
    : layer_id_(id), m_bEnableEdit(true), m_bVisible(true) {}

Layer::~Layer() {}

void Layer::SetVisible(bool bVisible) { m_bVisible = bVisible; }

bool Layer::IsVisible() const { return m_bVisible; }

bool Layer::IsEnableEdit() const { return m_bEnableEdit; }

}  // namespace geditor
