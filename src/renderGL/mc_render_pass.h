
#pragma once

namespace geditor {

class RenderStateObject;

class Program;

class RenderPass {
 public:
  RenderPass();

  ~RenderPass();

 public:
  RenderStateObject *const &GetRenderStateObject() const;

  void SetRenderStateObject(RenderStateObject *const renderStateObj);

  void Bind() const;

  void Unbind() const;

  void SetShaderProgram(Program *shader);

  Program *GetShaderProgram() const;

 private:
  RenderStateObject *render_state_obj_ = nullptr;
  Program *shader_program_ = nullptr;
};

}  // namespace geditor
