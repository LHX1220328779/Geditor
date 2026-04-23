
#pragma once

namespace geditor {

class VertexBuffer;

class IndexBuffer;

class VertexFormat;

class Program;

class RenderLayout {
 public:
  enum TopologyType {
    TT_PointList,
    TT_LineList,
    TT_LineStrip,
    TT_TriangleList,
    TT_TriangleStrip,
  };

 public:
  RenderLayout();

  ~RenderLayout();

  void SetTopologyType(TopologyType type);

  TopologyType GetTopologyType() const;

  void BindVertexStream(VertexBuffer *vBuffer, VertexFormat *vFormat);

  void BindIndexStream(IndexBuffer *iBuffer);

  bool UseIndices() const;

  void Active(Program const &so) const;

 protected:
  TopologyType topo_type_;
  VertexBuffer *buffer_ = nullptr;
  VertexFormat *format_ = nullptr;
  IndexBuffer *ibuffer_ = nullptr;
};

}  // namespace geditor
