
// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once	// same with #ifndef #define .... #endif but is non-standard and supported by a limited number of platforms.

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // Exclude rarely-used stuff from Windows headers
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // some CString constructors will be explicit

// turns off MFC's hiding of some common and often safely ignored warning messages
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions


#include <afxdisp.h>        // MFC Automation classes



#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC support for Internet Explorer 4 Common Controls
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC support for ribbons and control bars


#if !defined __OPENCV_ALL_HPP__
#include <opencv2\opencv.hpp>
using namespace cv;
#endif






#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif


#ifndef MIN
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

#ifndef ABS
#  define ABS(x)	((x) > 0) ? (x) : (-(x))
#endif

#define SQUARE(a)			((a)*(a))
#define MAX3(a,b,c)			MAX(MAX(a, b), c)
#define MIN3(a,b,c)			MIN(MIN(a, b), c)
#define CLAMP_U8(x)			MIN(MAX(x, 0), 255)
#define SQRT3_OVER6			0.288675f
#define SQRT3_OVER2			0.866025f
//#define SVM_VEC_SZ			161

typedef unsigned char uint8_t;
typedef struct point_t point_t;
typedef struct point3d_t point3d_t;
typedef struct rect_t rect_t;
typedef struct rect2_t rect2_t;
typedef struct image_t image_t;

struct image_t {
	unsigned char *data;
	int width, height;
	int channels;
	rect_t *roi;
	long misc_info;
};
typedef struct bgr{
	uint8_t b;
	uint8_t g;
	uint8_t r;
} bgr_t;

typedef struct hsv{
	uint8_t h;
	uint8_t s;
	uint8_t v;
} hsv_t;
struct point_t {
	long x, y;
};
struct point3d_t {
	double x, y, z;
};
struct rect_t {
	long x, y, width, height;
};
struct rect2_t {
	long x1, y1, x2, y2;
};


static rect_t *rect_set(rect_t *rect, int x, int y, int width, int height)
{
	rect->x = x;
	rect->y = y;
	rect->width = width;
	rect->height = height;
	return rect;
}