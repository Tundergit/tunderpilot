#pragma once

#include "visionbuf.h"

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GLES3/gl3.h>
#endif

#ifdef QCOM
#include <EGL/egl.h>
#define EGL_EGLEXT_PROTOTYPES
#include <EGL/eglext.h>
#undef Status
#endif

class EGLImageTexture {
 public:
  EGLImageTexture(const VisionBuf *buf);
  ~EGLImageTexture();
  GLuint frame_tex = 0;
#ifdef QCOM
  void *private_handle = nullptr;
  EGLImageKHR img_khr = 0;
#endif
};
