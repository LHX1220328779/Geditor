
#pragma once

#include "core/object.h"
#include "core/state.h"

#include <list>

namespace geditor {

class GraphicsThread;

class Camera;

class GraphicsContext : public Object {
 public:
  GraphicsContext();

  virtual ~GraphicsContext();

 public:
  bool CreateContext(HDC hDC);

  void DestoyContext();

  bool makeCurrent();

  bool releaseContext();

  void swapBuffers();

  void resized(int x, int y, int width, int height);

  void setState(State *state) { m_state = state; }

  State *getState() { return m_state; }

  virtual void resizedImplementation(int x, int y, int width, int height);

  virtual void runOperations();

 public:
  bool SetPixelFormat();

 public:
  void CreateGraphicsThread();

  // GraphicsThread*   GetGraphicsThread() { return m_graphicsThread; }

 private:
  // GraphicsThread*       m_graphicsThread;
  State *m_state;
  std::list<Camera *> m_cameras;

  HDC m_hDC = NULL;
  HGLRC m_hRC = NULL;
  HWND m_hWnd = NULL;
};
}  // namespace geditor
