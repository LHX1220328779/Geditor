
#pragma once
namespace geditor {

class Shader {
 public:
  Shader();

  virtual ~Shader();

 public:
  bool LoadShaderSource(const char *shaderSoure);

  bool LoadShaderFile(const char *fileName);

  void DeleteShader();

  bool IsValidate() { return (handle_ != 0); }

  unsigned int GetHandle() { return handle_; }

 protected:
  virtual unsigned int CreateShader() = 0;

 private:
  bool CheckCompileStatus(unsigned int shader);

 private:
  unsigned int handle_;
};

//-----------------------------------------------

class VertexShader : public Shader {
 public:
  VertexShader();

  virtual ~VertexShader();

 protected:
  virtual unsigned int CreateShader();
};

//-----------------------------------------------

class FragmentShader : public Shader {
 public:
  FragmentShader();

  virtual ~FragmentShader();

 protected:
  virtual unsigned int CreateShader();
};
}  // namespace geditor
