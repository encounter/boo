#ifndef BOO_HPP
#define BOO_HPP

#if defined(_WIN32)
#error "No support for WGL"

#elif defined(__APPLE__)
#include "mac/CCGLContext.hpp"
typedef CCGLContext CGraphicsContext;


#elif defined(__GNUC__) || defined(__clang__)
#include "x11/CGLXContext.hpp"
typedef CGLXContext CGraphicsContext;

#endif

#include "IGraphicsContext.hpp"
#include "inputdev/CDeviceFinder.hpp"

#endif // BOO_HPP
