//--------------------------------------------------------------------------------------
// File: SimpleBezier11.cpp
//--------------------------------------------------------------------------------------

#include "DXUT.h"
#include "DXUTcamera.h"
#include "DXUTgui.h"
#include "DXUTsettingsDlg.h"
#include "SDKmisc.h"
#include "resource.h"
#include <math.h>
#include <sstream>
#include <iostream>

#pragma warning( disable : 4100 )
using namespace DirectX;

struct SimpleVertex {
   float px, py, pz;
   float nx, ny, nz;
   float r, g, b;
};
SimpleVertex *g_extraGeometry = nullptr;
int g_extraGeometry_Size = 0;

struct BEZIER_CONTROL_POINT {
   float m_vPosition[3];
   float weight;
};
BEZIER_CONTROL_POINT *g_MobiusStrip = nullptr;
int g_MobiusStrip_Size = 0;


// ===================================================
// 3D vector utilities
// ===================================================

struct Vector {
   float x, y, z;

   Vector(float x_, float y_, float z_) { x = x_;   y = y_; z = z_; }
   Vector(const Vector& v) { x = v.x; y = v.y; z = v.z; }
   Vector& operator+=(const Vector &v) { x += v.x; y += v.y; z += v.z; return *this; }
   Vector& operator-=(const Vector &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
   Vector& operator*=(float f) { x *= f;  y *= f;  z *= f;  return *this; }
   Vector& operator/=(float f) { x /= f;  y /= f;  z /= f;  return *this; }
   Vector operator*(float f) { return Vector(*this) *= f; }
   Vector operator/(float f) { return Vector(*this) /= f; }
   float length_sq();
   float length();
   Vector normalized() { return Vector(*this) /= length(); }
};

Vector operator+(Vector v1, const Vector& v2) { return v1 += v2; }
Vector operator-(Vector v1, const Vector& v2) { return v1 -= v2; }
Vector operator*(float f, Vector v2) { return v2 *= f; }
float dot(const Vector& v1, const Vector& v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }
float Vector::length_sq() { return dot(*this, *this); }
float Vector::length() { return sqrt(length_sq()); }

Vector cross(const Vector& v1, const Vector& v2) {
   return Vector(v1.y*v2.z - v1.z*v2.y, -v1.x*v2.z + v1.z*v2.x, v1.x*v2.y - v1.y*v2.x);
}
Vector latlon(float lat, float lon) {
   float cosLat = (float)cos(lat);
   float sinLat = (float)sin(lat);
   float cosLon = (float)cos(lon);
   float sinLon = (float)sin(lon);
   return Vector(cosLat * cosLon, cosLat * sinLon, sinLat);
}

// ==========================================================
// Fill the arrays that are shipped to the GPU
// ==========================================================

struct ControlPoint {
   Vector point;    // 3 numbers: x,y,z
   float  weight;   // 1 number

   ControlPoint() : point(0.0f, 0.0f, 0.0f), weight(1.0f) {}
   ControlPoint(const Vector& point_, float weight_) : point(point_), weight(weight_) {}
};
struct Patch {
   ControlPoint p[4][4];
};

struct GeometryAssembler {
   Patch patch0,patch1,patch2;

   GeometryAssembler(Patch patch0_, Patch patch1_, Patch patch2_): 
      patch0(patch0_), patch1(patch1_), patch2(patch2_) {}
   ~GeometryAssembler();
   void updateDirectXArrays(float shape);

  private:
   std::vector<SimpleVertex> extra_geometry;

   void addTriangle(const Vector& v0, 
                   const Vector& v1, 
                   const Vector& v2, 
                   const Vector& color);
   void addQuad(const Vector& tl,
               const Vector& bl,
                const Vector& br,
               const Vector& tr,
               const Vector& color);

   void addBall(const Vector& center,
              float radius,
              const Vector& color);

   void addRod(const Vector& v0, const Vector& v1, float radius, const Vector& color);

   void addIcoTriangle(float lat0, float lon0,
                        float lat1, float lon1,
                      float lat2, float lon2,
                      const Vector& center,
                      float radius,
                      const Vector& color);

   void addFloor(float z);
   void clearDirectXArrays();
};

GeometryAssembler::~GeometryAssembler() {
   clearDirectXArrays();
}

// This adds the coordinates of a "regular triangle" to the
// "extra_geometry" array
void GeometryAssembler::addTriangle(const Vector& v0,
                                    const Vector& v1,
                                   const Vector& v2,
                                   const Vector& color) {

   Vector n = cross((v2-v1), (v0-v1)).normalized();
   extra_geometry.push_back(
      SimpleVertex{ v0.x,v0.y,v0.z, n.x,n.y,n.z, color.x,color.y,color.z });
   extra_geometry.push_back(
      SimpleVertex{ v1.x,v1.y,v1.z, n.x,n.y,n.z, color.x,color.y,color.z });
   extra_geometry.push_back(
      SimpleVertex{ v2.x,v2.y,v2.z, n.x,n.y,n.z, color.x,color.y,color.z });
}

// This adds a quad
void GeometryAssembler::addQuad(
   const Vector& tl,
   const Vector& bl,
   const Vector& br,
   const Vector& tr,
   const Vector& color) {

   addTriangle(tl, bl, br, color);
   addTriangle(tl, br, tr, color);
}

// This adds a checkerboard floor
void GeometryAssembler::addFloor(float z) {
   Vector color1(0.05f, 0.05f, 0.1f);
   Vector color2(0.10f, 0.10f, 0.15f);
   for (int x = -4; x <= 3; x++) {
      for (int y = -4; y <= 3; y++) {
         Vector tl = Vector(x, y, z) / 4;
         Vector bl = Vector(x, y + 1, z) / 4;
         Vector tr = Vector(x + 1, y, z) / 4;
         Vector br = Vector(x + 1, y + 1, z) / 4;
         addQuad(tl, bl, br, tr, (((x+4) % 2) == ((y+4) % 2)) ? color1 : color2);
      }
   }
}

// This adds an icosahedron
void GeometryAssembler::addBall(const Vector& center,
                                  float radius,
                                const Vector& color) {

   float pi = (float)3.1415926536;
   float lat0 = pi / 2;
   float lat1 = (float)atan(0.5);
   float lat2 = -lat1;
   float lat3 = -lat0;

   for (int i = 0; i < 5; ++i) {
      float lon0 = 2 * i*(pi / 5);
      float lon1 = lon0 + (pi / 5);
      float lon2 = lon1 + (pi / 5);
      float lon3 = lon2 + (pi / 5);

      // Top
      addIcoTriangle( lat0, lon1,
                     lat1, lon0,
                     lat1, lon2, center,radius,color);
      // "interior"
      addIcoTriangle(lat1, lon0,
         lat2, lon1,
         lat1, lon2, center, radius, color);

      addIcoTriangle(lat1, lon2,
         lat2, lon1,
         lat2, lon3, center, radius, color);

      // Bottom
      addIcoTriangle(lat2, lon1,
         lat3, lon2,
         lat2, lon3, center, radius, color);
   }
}
void GeometryAssembler::addIcoTriangle(
   float lat0, float lon0,
   float lat1, float lon1,
   float lat2, float lon2,
   const Vector& center,
   float radius,
   const Vector& color) {

   Vector v0 = center + latlon(lat0, lon0) * radius;
   Vector v1 = center + latlon(lat1, lon1) * radius;
   Vector v2 = center + latlon(lat2, lon2) * radius;
   addTriangle(v0, v1, v2, color);
}

// This adds a cylinder
void GeometryAssembler::addRod(const Vector& v0, const Vector& v1, float radius, const Vector& color) {
   Vector axis = v1 - v0;
   Vector unit_vector_not_along_axis =
      ((fabs(axis.y) <= fabs(axis.x)) && (fabs(axis.y) <= fabs(axis.z))) ? Vector(0.0, 1.0, 0.0) :
      ((fabs(axis.x) <= fabs(axis.y)) && (fabs(axis.x) <= fabs(axis.z))) ? Vector(1.0, 0.0, 0.0) :
                                                                           Vector(0.0, 0.0, 1.0);

   Vector lx = cross(unit_vector_not_along_axis, axis).normalized();
   Vector ly = cross(lx, axis).normalized();

   int n_div = 7;
   float pi = (float)3.1415926536;
   for (int div = 0; div < n_div; div++) {
      float angle1 = (float) (2 * pi * div / n_div);
      float angle2 = (float) (2 * pi * (div+1) / n_div);

      Vector d1 = (lx * cos(angle1) + ly * sin(angle1)) * radius;
      Vector d2 = (lx * cos(angle2) + ly * sin(angle2)) * radius;
      addQuad(v0 + d2, v0 + d1, v1 + d1, v1 + d2, color);
   }
}

// Clear out the arrays
void GeometryAssembler::clearDirectXArrays() {
   if (g_MobiusStrip != nullptr) {
      delete[] g_MobiusStrip;
      g_MobiusStrip = nullptr;
   }
   g_MobiusStrip_Size = 0;

   if (g_extraGeometry != nullptr) {
      delete[] g_extraGeometry;
      g_extraGeometry = nullptr;
   }
   g_extraGeometry_Size = 0;
   extra_geometry.clear();
}

// -----------------------------------------------------------------
// This functions assembles all the data we're going to send to the GPU.
// -----------------------------------------------------------------
//
// This includes the 16-control-points for the PATCH, 
// (which will be rendered with tessellation),
// but also a large number of "regular" triangles that make up the checkerboard
// floor, the icosahedron for each control point, and cylinders representing
// thick solid lines connecting the control points.

void GeometryAssembler::updateDirectXArrays(float shape) {

   const Patch& p0 = (shape < 0.5f) ? patch0 : patch1;
   const Patch& p1 = (shape < 0.5f) ? patch1 : patch2;
   float f = (shape < 0.5f) ? (2 * shape) : (2 * (shape - 0.5f));

   // What are the 16 control points?
   // We have a SLIDER called "shape" that's actually interpolating
   // between three hardcoded patches called patch0/patch1/patch2.
   Patch patch;

   // This code fills in "patch":
   for (int cpX = 0; cpX < 4; cpX++) {
      for (int cpY = 0; cpY < 4; cpY++) {
         Vector v0 = p0.p[cpX][cpY].point;
         Vector v1 = p1.p[cpX][cpY].point;
         float w0 = p0.p[cpX][cpY].weight;
         float w1 = p1.p[cpX][cpY].weight;

         patch.p[cpX][cpY].point = v0 * (1.0f - f) + v1 * f;
         patch.p[cpX][cpY].weight = w0 * (1.0f - f) + w1 * f;
      }
   }

   // Now "patch" contains the 16 control points we're going to render.

   // Clear the arrays so we can fill them:
   clearDirectXArrays();

   // This copies the 16 control points
   // into the array that will be sent to the GPU:
   g_MobiusStrip_Size = 16;
   g_MobiusStrip = new BEZIER_CONTROL_POINT[g_MobiusStrip_Size];
   int i = 0;
   for (int cpX = 0; cpX < 4; cpX++) {
      for (int cpY = 0; cpY < 4; cpY++) {
         const ControlPoint& cp = patch.p[cpX][cpY];
         BEZIER_CONTROL_POINT* out_point = g_MobiusStrip + (i++);
         out_point->m_vPosition[0] = cp.point.x;
         out_point->m_vPosition[1] = cp.point.y;
         out_point->m_vPosition[2] = cp.point.z;
         out_point->weight = cp.weight;
      }
   }

   // This adds "regular triangles" representing a checkerboard floor
   addFloor(-0.01f);

   Vector ballColor = Vector(0.2f, 0.4f, 1.0f);
   Vector rodColor = Vector(0.8f, 0.3f, 0.35f);
   float ballRadius = 0.035f;
   float rodRadius  = 0.009f;

   // We're going to add "regular triangles" in the shape of an icosahedron
   // surrounding each control point.  This will make the control points
   // nice and visible:
   for (int cpX = 0; cpX < 4; cpX++) {
      for (int cpY = 0; cpY < 4; cpY++) {
         const ControlPoint& cp = patch.p[cpX][cpY];
         addBall(cp.point, ballRadius, ballColor);
      }
   }

   // We're going to add more "regular triangles" in the shape of
   // cylinders CONNECTING adjacent control points:
   for (int cpX = 0; cpX < 4; cpX++) {
      const Vector& v0 = patch.p[cpX][0].point;
      const Vector& v1 = patch.p[cpX][1].point;
      const Vector& v2 = patch.p[cpX][2].point;
      const Vector& v3 = patch.p[cpX][3].point;
      addRod(v0, v1, rodRadius, rodColor);
      addRod(v1, v2, rodRadius, rodColor);
      addRod(v2, v3, rodRadius, rodColor);
   }
   for (int cpY = 0; cpY < 4; cpY++) {
      const Vector& v0 = patch.p[0][cpY].point;
      const Vector& v1 = patch.p[1][cpY].point;
      const Vector& v2 = patch.p[2][cpY].point;
      const Vector& v3 = patch.p[3][cpY].point;
      addRod(v0, v1, rodRadius, rodColor);
      addRod(v1, v2, rodRadius, rodColor);
      addRod(v2, v3, rodRadius, rodColor);
   }

   g_extraGeometry_Size = extra_geometry.size();;
   g_extraGeometry = new SimpleVertex[g_extraGeometry_Size];
   for (i = 0; i < g_extraGeometry_Size; ++i) {
      g_extraGeometry[i] = extra_geometry[i];
   }
}


// This object manages our geometry. 
// In particular, it uses a SLIDER VALUE to interpolate between
// three different control point configurations, so we can
// move the control points by sliding the slider back and forth:

GeometryAssembler geometryAssembler(

   // First Patch
   Patch{
        ControlPoint(Vector(1.0f, -0.5f, 0.0f),         1.0f),
      ControlPoint(Vector(1.0f, -0.5f, 0.5f),         1.0f),
      ControlPoint(Vector(-0.5f, -0.3536f, 1.354f),   1.0f),
      ControlPoint(Vector(-1.0f, -0.8f, 0.0f),        1.0f),

      ControlPoint(Vector(1.0f, -0.1667f, 0.0f),      1.0f),
      ControlPoint(Vector(1.0f, -0.1667f, 0.5f),      1.0f),
      ControlPoint(Vector(-0.5f, -0.1179f, 1.118f),   1.0f),
      ControlPoint(Vector(-1.0f, -0.4f, 0.5f),        1.0f),

      ControlPoint(Vector(1.0f, 0.1667f, 0.0f),       1.0f),
      ControlPoint(Vector(1.0f, 0.1667f, 0.5f),       1.0f),
      ControlPoint(Vector(-0.5f, 0.1179f, 0.8821f),   1.0f),
      ControlPoint(Vector(-1.0f, 0.4f, 0.5f),         1.0f),

      ControlPoint(Vector(1.0f, 0.5f, 0.0f),          1.0f),
      ControlPoint(Vector(1.0f, 0.5f, 0.5f),          1.0f),
      ControlPoint(Vector(-0.5f, 0.3536f, 0.6464f),   1.0f),
      ControlPoint(Vector(-1.0f, 0.8f, 0.0f),         1.0f)
   },

   // If the SLIDER VALUE is between 0 and 0.5,
   // the patch that's actually displayed is an interpolation
   // between these First and Second configurations

   // Second Patch
   Patch{
      ControlPoint(Vector(1.0f, -0.5f, 0.0f),         1.0f),
      ControlPoint(Vector(1.0f, -0.5f, 0.5f),         1.0f),
      ControlPoint(Vector(-0.5f, -0.3536f, 1.354f),   1.0f),
      ControlPoint(Vector(-1.0f, -0.8f, 0.0f),        1.0f),

      ControlPoint(Vector(1.0f, -0.1667f, 0.0f),      1.0f),
      ControlPoint(Vector(1.0f, -0.1667f, 0.5f),      1.0f),
      ControlPoint(Vector(-0.5f, -0.1179f, 1.118f),   1.0f),
      ControlPoint(Vector(-1.0f, -0.4f, 0.0f),        1.0f),

      ControlPoint(Vector(1.0f, 0.1667f, 0.0f),       1.0f),
      ControlPoint(Vector(1.0f, 0.1667f, 0.5f),       1.0f),
      ControlPoint(Vector(-0.5f, 0.1179f, 0.8821f),   1.0f),
      ControlPoint(Vector(-1.0f, 0.4f, 0.0f),         1.0f),

      ControlPoint(Vector(1.0f, 0.5f, 0.0f),          1.0f),
      ControlPoint(Vector(1.0f, 0.5f, 0.5f),          1.0f),
      ControlPoint(Vector(-0.5f, 0.3536f, 0.6464f),   1.0f),
      ControlPoint(Vector(-1.0f, 0.8f, 0.0f),         1.0f)
   },

   // If the SLIDER VALUE is between 0.5 and 1.0,
   // the patch that's actually displayed is an interpolation
   // between these Second and Third configurations

   // Below are the control points & weights
   // for a hemispherical NURBS surface
   Patch{
      ControlPoint(Vector(0.0f,  -0.5f,  0.0f),   1.0f),
      ControlPoint(Vector(0.0f,  -0.5f,  0.0f),   1.0f/3),
      ControlPoint(Vector(0.0f,  -0.5f,  0.0f),   1.0f/3),
      ControlPoint(Vector(0.0f,  -0.5f,  0.0f),   1.0f),

      ControlPoint(Vector(1.0f,   -0.5f,  0.0f),   1.0f/3),
      ControlPoint(Vector(1.0f,   -0.5f,  2.0f),   1.0f/9),
      ControlPoint(Vector(-1.0f,  -0.5f,  2.0f),   1.0f/9),
      ControlPoint(Vector(-1.0f,  -0.5f,  0.0f),   1.0f/3),

      ControlPoint(Vector(1.0f,    0.5f,  0.0f),   1.0f/3),
      ControlPoint(Vector(1.0f,    0.5f,  2.0f),   1.0f/9),
      ControlPoint(Vector(-1.0f,   0.5f,  2.0f),   1.0f/9),
      ControlPoint(Vector(-1.0f,   0.5f,  0.0f),   1.0f/3),

      ControlPoint(Vector(0.0f,    0.5f,  0.0f),   1.0f),
      ControlPoint(Vector(0.0f,    0.5f,  0.0f),   1.0f/3),
      ControlPoint(Vector(0.0f,    0.5f,  0.0f),   1.0f/3),
      ControlPoint(Vector(0.0f,    0.5f,  0.0f),   1.0f)
    }
);


// Min and Max divisions of the patch per side for the slider control
const DWORD MIN_DIVS = 2;
const DWORD MAX_DIVS = 40; 

//--------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------
CDXUTDialogResourceManager          g_DialogResourceManager; // manager for shared resources of dialogs
CModelViewerCamera                  g_Camera;                // A model viewing camera
CD3DSettingsDlg                     g_D3DSettingsDlg;        // Device settings dialog
CDXUTDialog                         g_HUD;                   // manages the 3D   
CDXUTDialog                         g_SampleUI;              // dialog for sample specific controls

// Resources
CDXUTTextHelper*                    g_pTxtHelper = nullptr;

ID3D11InputLayout*                  g_pPatchLayout = nullptr;
ID3D11VertexShader*                 g_pVertexShader = nullptr;
ID3D11HullShader*                   g_pHullShader = nullptr;
ID3D11DomainShader*                 g_pDomainShader = nullptr;
ID3D11PixelShader*                  g_pPixelShader = nullptr;
ID3D11PixelShader*                  g_pSolidColorPS = nullptr;
ID3D11Buffer*                       g_pControlPointVB;           // Control points for mesh

ID3D11InputLayout*                  g_pExtraGeometryPatchLayout = nullptr;
ID3D11VertexShader*                 g_pExtraGeometryVS = nullptr;
ID3D11PixelShader*                  g_pExtraGeometryPS = nullptr;
ID3D11Buffer*                       g_pExtraGeometryVB;         // Vertices for extra geometry

struct CB_PER_FRAME_CONSTANTS
{
   XMFLOAT4X4  mView;
   XMFLOAT4X4  mProj;

   XMFLOAT3    vCameraPosWorld;
   float       fTessellationFactor;
   float       bNurbs;
   float       mNearPlane;
   float       mFarPlane;
   float       padding[1];
};

ID3D11Buffer*                       g_pcbPerFrame = nullptr;
UINT                                g_iBindPerFrame = 0;

ID3D11RasterizerState*              g_pRasterizerStateSolid = nullptr;
ID3D11RasterizerState*              g_pRasterizerStateWireframe = nullptr;

// Control variables
float                               g_fSubdivs = 0.5f;       // Startup subdivisions per side
bool                                g_bDrawWires = false;    // Draw the mesh with wireframe overlay
float                               g_fShape = 0; 
bool                                g_shapeChanged = false;
bool                                g_bNurbs = false;

//--------------------------------------------------------------------------------------
// UI control IDs
//--------------------------------------------------------------------------------------

#define IDC_TOGGLEFULLSCREEN      1
#define IDC_TOGGLEREF             3
#define IDC_CHANGEDEVICE          4
#define IDC_PATCH_SUBDIVS         5
#define IDC_PATCH_SUBDIVS_STATIC  6
#define IDC_TOGGLE_LINES          7
#define IDC_PATCH_SHAPE           8
#define IDC_PATCH_SHAPE_STATIC    9
#define IDC_TOGGLE_NURBS          10

//--------------------------------------------------------------------------------------
// Forward declarations 
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings( DXUTDeviceSettings* pDeviceSettings, void* pUserContext );
void CALLBACK OnFrameMove( double fTime, float fElapsedTime, void* pUserContext );
LRESULT CALLBACK MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing,
                          void* pUserContext );
void CALLBACK OnGUIEvent( UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext );

bool CALLBACK IsD3D11DeviceAcceptable( const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo,
                                       DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext );
HRESULT CALLBACK OnD3D11CreateDevice( ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
                                      void* pUserContext );
HRESULT CALLBACK OnD3D11ResizedSwapChain( ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain,
                                          const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext );
void CALLBACK OnD3D11ReleasingSwapChain( void* pUserContext );
void CALLBACK OnD3D11DestroyDevice( void* pUserContext );
void CALLBACK OnD3D11FrameRender( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime,
                                  float fElapsedTime, void* pUserContext );


void InitApp();
void RenderText();

//--------------------------------------------------------------------------------------
// Entry point to the program. Initializes everything and goes into a message processing 
// loop. Idle time is used to render the scene.
//--------------------------------------------------------------------------------------
int WINAPI wWinMain( _In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow )
{
    // Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif

   geometryAssembler.updateDirectXArrays(g_fShape);

    // Set DXUT callbacks
    DXUTSetCallbackDeviceChanging( ModifyDeviceSettings );
    DXUTSetCallbackMsgProc( MsgProc );
    DXUTSetCallbackFrameMove( OnFrameMove );

    DXUTSetCallbackD3D11DeviceAcceptable( IsD3D11DeviceAcceptable );
    DXUTSetCallbackD3D11DeviceCreated( OnD3D11CreateDevice );
    DXUTSetCallbackD3D11SwapChainResized( OnD3D11ResizedSwapChain );
    DXUTSetCallbackD3D11FrameRender( OnD3D11FrameRender );
    DXUTSetCallbackD3D11SwapChainReleasing( OnD3D11ReleasingSwapChain );
    DXUTSetCallbackD3D11DeviceDestroyed( OnD3D11DestroyDevice );

    InitApp();
    DXUTInit( true, true ); // Parse the command line, show msgboxes on error, and an extra cmd line param to force REF for now
    DXUTSetCursorSettings( true, true ); // Show the cursor and clip it when in full screen
    DXUTCreateWindow( L"Sfarti Patent Demo" );
    DXUTCreateDevice( D3D_FEATURE_LEVEL_11_0,  true, 1000, 1000 );
    DXUTMainLoop(); // Enter into the DXUT render loop

    return DXUTGetExitCode();
}


//--------------------------------------------------------------------------------------
// Initialize the app 
//--------------------------------------------------------------------------------------
void InitApp()
{
    // Initialize dialogs
    g_D3DSettingsDlg.Init( &g_DialogResourceManager );
    g_HUD.Init( &g_DialogResourceManager );
    g_SampleUI.Init( &g_DialogResourceManager );

    g_HUD.SetCallback( OnGUIEvent ); int iY = 20;
    g_HUD.AddButton( IDC_TOGGLEFULLSCREEN, L"Toggle full screen", 0, iY, 170, 22 );
    g_HUD.AddButton( IDC_TOGGLEREF, L"Toggle REF (F3)", 0, iY += 26, 170, 22, VK_F3 );
    g_HUD.AddButton( IDC_CHANGEDEVICE, L"Change device (F2)", 0, iY += 26, 170, 22, VK_F2 );

    g_SampleUI.SetCallback( OnGUIEvent ); iY = 10;

    WCHAR sz[100];
    iY += 24;
    swprintf_s( sz, L"Threshold: %2.1f", g_fSubdivs );
    g_SampleUI.AddStatic( IDC_PATCH_SUBDIVS_STATIC, sz, 10, iY += 26, 150, 22 );
    g_SampleUI.AddSlider( IDC_PATCH_SUBDIVS, 10, iY += 24, 150, 22, 0, 100, (int)(g_fSubdivs * 100));

    iY += 24;
    g_SampleUI.AddCheckBox( IDC_TOGGLE_LINES, L"Wireframe?", 20, iY += 26, 150, 22, g_bDrawWires );

   WCHAR sz2[100];
   iY += 24;
   swprintf_s(sz2, L"Shape: %2d%%", ((int)(100 * g_fShape)));
   g_SampleUI.AddStatic(IDC_PATCH_SHAPE_STATIC, sz2, 10, iY += 26, 150, 22);
   g_SampleUI.AddSlider(IDC_PATCH_SHAPE, 10, iY += 24, 150, 22, 0, 100, (int)(g_fShape * 100));

   iY += 24;
   g_SampleUI.AddCheckBox(IDC_TOGGLE_NURBS, L"NURBS?", 20, iY += 26, 150, 22, g_bNurbs);

   // Setup the camera's view parameters
   static const XMVECTORF32 s_vecEye = { 0.0f, -3.0f, 3.0f, 0.f };
   static const XMVECTORF32 s_vecAt = { 0.0f, 0.0f, 0.45f, 0.f };
   g_Camera.SetViewParams( s_vecEye, s_vecAt );
}


//--------------------------------------------------------------------------------------
// Called right before creating a D3D device, allowing the app to modify the device settings as needed
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings( DXUTDeviceSettings* pDeviceSettings, void* pUserContext )
{
    return true;
}


//--------------------------------------------------------------------------------------
// Handle updates to the scene
//--------------------------------------------------------------------------------------
void CALLBACK OnFrameMove( double fTime, float fElapsedTime, void* pUserContext )
{
    // Update the camera's position based on user input 
    g_Camera.FrameMove( fElapsedTime );
}


//--------------------------------------------------------------------------------------
// Render the help and statistics text
//--------------------------------------------------------------------------------------
void RenderText()
{
    g_pTxtHelper->Begin();
    g_pTxtHelper->SetInsertionPos( 2, 0 );
    g_pTxtHelper->SetForegroundColor( Colors::Yellow );
    g_pTxtHelper->DrawTextLine( DXUTGetFrameStats( DXUTIsVsyncEnabled() ) );
    g_pTxtHelper->DrawTextLine( DXUTGetDeviceStats() );
    g_pTxtHelper->DrawTextLine( L"Hello World!" );

    g_pTxtHelper->End();
}


//--------------------------------------------------------------------------------------
// Handle messages to the application
//--------------------------------------------------------------------------------------
LRESULT CALLBACK MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing,
                          void* pUserContext )
{
    // Pass messages to dialog resource manager calls so GUI state is updated correctly
    *pbNoFurtherProcessing = g_DialogResourceManager.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;

    // Pass messages to settings dialog if its active
    if( g_D3DSettingsDlg.IsActive() )
    {
        g_D3DSettingsDlg.MsgProc( hWnd, uMsg, wParam, lParam );
        return 0;
    }

    // Give the dialogs a chance to handle the message first
    *pbNoFurtherProcessing = g_HUD.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;
    *pbNoFurtherProcessing = g_SampleUI.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;

    // Pass all remaining windows messages to camera so it can respond to user input
    g_Camera.HandleMessages( hWnd, uMsg, wParam, lParam );

    return 0;
}


//--------------------------------------------------------------------------------------
// Handles the GUI events
//--------------------------------------------------------------------------------------
void CALLBACK OnGUIEvent( UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext )
{
    switch( nControlID )
    {
            // Standard DXUT controls
        case IDC_TOGGLEFULLSCREEN:
            DXUTToggleFullScreen(); break;
        case IDC_TOGGLEREF:
            DXUTToggleREF(); break;
        case IDC_CHANGEDEVICE:
            g_D3DSettingsDlg.SetActive( !g_D3DSettingsDlg.IsActive() ); break;

            // Custom app controls
        case IDC_PATCH_SUBDIVS:
        {
            g_fSubdivs = g_SampleUI.GetSlider( IDC_PATCH_SUBDIVS )->GetValue() / 100.0f;

            WCHAR sz[100];
            swprintf_s( sz, L"Threshold: %2.1f", g_fSubdivs );
            g_SampleUI.GetStatic( IDC_PATCH_SUBDIVS_STATIC )->SetText( sz );
        }
        break;

      case IDC_PATCH_SHAPE:
      {
         g_fShape = g_SampleUI.GetSlider(IDC_PATCH_SHAPE)->GetValue() / 100.0f;
         g_shapeChanged = true;

         WCHAR sz[100];
         swprintf_s(sz, L"Shape: %2d%%", ((int)(100*g_fShape)));
         g_SampleUI.GetStatic(IDC_PATCH_SHAPE_STATIC)->SetText(sz);
      }
      break;

      case IDC_TOGGLE_LINES:
            g_bDrawWires = g_SampleUI.GetCheckBox( IDC_TOGGLE_LINES )->GetChecked();
            break;

      case IDC_TOGGLE_NURBS:
         g_bNurbs = g_SampleUI.GetCheckBox(IDC_TOGGLE_NURBS)->GetChecked();
         break;

    }
}


//--------------------------------------------------------------------------------------
// Reject any D3D11 devices that aren't acceptable by returning false
//--------------------------------------------------------------------------------------
bool CALLBACK IsD3D11DeviceAcceptable( const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo,
                                       DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext )
{
    return true;
}


//--------------------------------------------------------------------------------------
// Create any D3D11 resources that aren't dependant on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11CreateDevice( ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
                                      void* pUserContext )
{
    HRESULT hr;

    auto pd3dImmediateContext = DXUTGetD3D11DeviceContext();
    V_RETURN( g_DialogResourceManager.OnD3D11CreateDevice( pd3dDevice, pd3dImmediateContext ) );
    V_RETURN( g_D3DSettingsDlg.OnD3D11CreateDevice( pd3dDevice ) );
    g_pTxtHelper = new CDXUTTextHelper( pd3dDevice, pd3dImmediateContext, &g_DialogResourceManager, 15 );

    // Compile shaders
    ID3DBlob* pBlobVS = nullptr;
    ID3DBlob* pBlobHS = nullptr;
    ID3DBlob* pBlobDS = nullptr;
    ID3DBlob* pBlobPS = nullptr;
    ID3DBlob* pBlobPSSolid = nullptr;
    ID3DBlob* pBlobExtraGeometryPS = nullptr;
    ID3DBlob* pBlobExtraGeometryVS = nullptr;

    V_RETURN( DXUTCompileFromFile( L"SimpleBezier11.hlsl", nullptr, "BezierVS", "vs_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobVS ) );
    V_RETURN( DXUTCompileFromFile( L"SimpleBezier11.hlsl", nullptr, "BezierHS", "hs_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobHS ) );
    V_RETURN( DXUTCompileFromFile( L"SimpleBezier11.hlsl", nullptr, "BezierDS", "ds_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobDS ) );
    V_RETURN( DXUTCompileFromFile( L"SimpleBezier11.hlsl", nullptr, "BezierPS", "ps_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobPS ) );
    V_RETURN( DXUTCompileFromFile( L"SimpleBezier11.hlsl", nullptr, "SolidColorPS", "ps_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobPSSolid ) );

    V_RETURN(DXUTCompileFromFile(L"SimpleBezier11.hlsl", nullptr, "ExtraGeometryVS", "vs_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobExtraGeometryVS));
    V_RETURN(DXUTCompileFromFile(L"SimpleBezier11.hlsl", nullptr, "ExtraGeometryPS", "ps_5_0",
                                   D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobExtraGeometryPS));

    // Create shaders
    V_RETURN( pd3dDevice->CreateVertexShader( pBlobVS->GetBufferPointer(), pBlobVS->GetBufferSize(), nullptr, &g_pVertexShader ) );
    DXUT_SetDebugName( g_pVertexShader, "BezierVS" );
   
    V_RETURN( pd3dDevice->CreateHullShader( pBlobHS->GetBufferPointer(), pBlobHS->GetBufferSize(), nullptr, &g_pHullShader ) );
    DXUT_SetDebugName( g_pHullShader, "BezierHS" );

    V_RETURN( pd3dDevice->CreateDomainShader( pBlobDS->GetBufferPointer(), pBlobDS->GetBufferSize(), nullptr, &g_pDomainShader ) );
    DXUT_SetDebugName( g_pDomainShader, "BezierDS" );

    V_RETURN( pd3dDevice->CreatePixelShader( pBlobPS->GetBufferPointer(), pBlobPS->GetBufferSize(), nullptr, &g_pPixelShader ) );
    DXUT_SetDebugName( g_pPixelShader, "BezierPS" );

    V_RETURN( pd3dDevice->CreatePixelShader( pBlobPSSolid->GetBufferPointer(), pBlobPSSolid->GetBufferSize(), nullptr, &g_pSolidColorPS ) );
    DXUT_SetDebugName( g_pSolidColorPS, "SolidColorPS" );

    V_RETURN(pd3dDevice->CreateVertexShader(pBlobExtraGeometryVS->GetBufferPointer(), pBlobExtraGeometryVS->GetBufferSize(), nullptr, &g_pExtraGeometryVS));
    DXUT_SetDebugName(g_pExtraGeometryVS, "ExtraGeometryVS");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobExtraGeometryPS->GetBufferPointer(), pBlobExtraGeometryPS->GetBufferSize(), nullptr, &g_pExtraGeometryPS));
    DXUT_SetDebugName(g_pExtraGeometryPS, "ExtraGeometryPS");

    // Create our vertex input layout - this matches the BEZIER_CONTROL_POINT structure
    const D3D11_INPUT_ELEMENT_DESC patchlayout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0,  D3D11_INPUT_PER_VERTEX_DATA, 0 },
    };

    V_RETURN( pd3dDevice->CreateInputLayout( patchlayout, ARRAYSIZE( patchlayout ), pBlobVS->GetBufferPointer(),
                                             pBlobVS->GetBufferSize(), &g_pPatchLayout ) );
    DXUT_SetDebugName( g_pPatchLayout, "Primary" );


    // Create our vertex input layout - this matches the BEZIER_CONTROL_POINT structure
    const D3D11_INPUT_ELEMENT_DESC extraGeometryPatchlayout[] =
    {
      { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,  D3D11_INPUT_PER_VERTEX_DATA, 0 },
      { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
      { "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
    };

    V_RETURN(pd3dDevice->CreateInputLayout(extraGeometryPatchlayout, ARRAYSIZE(extraGeometryPatchlayout),
       pBlobExtraGeometryVS->GetBufferPointer(),
       pBlobExtraGeometryVS->GetBufferSize(), &g_pExtraGeometryPatchLayout));
    DXUT_SetDebugName(g_pExtraGeometryPatchLayout, "ExtraGeometryPrimary");


    SAFE_RELEASE( pBlobVS );
    SAFE_RELEASE( pBlobHS );
    SAFE_RELEASE( pBlobDS );
    SAFE_RELEASE( pBlobPS );
    SAFE_RELEASE( pBlobPSSolid );
    SAFE_RELEASE( pBlobExtraGeometryPS );
    SAFE_RELEASE( pBlobExtraGeometryVS );
   
    // Create constant buffers
    D3D11_BUFFER_DESC Desc;
    Desc.Usage =  D3D11_USAGE_DYNAMIC;
    Desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    Desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    Desc.MiscFlags = 0;

    Desc.ByteWidth = sizeof( CB_PER_FRAME_CONSTANTS );
    V_RETURN( pd3dDevice->CreateBuffer( &Desc, nullptr, &g_pcbPerFrame ) );
    DXUT_SetDebugName( g_pcbPerFrame, "CB_PER_FRAME_CONSTANTS" );

    // Create solid and wireframe rasterizer state objects
    D3D11_RASTERIZER_DESC RasterDesc;
    ZeroMemory( &RasterDesc, sizeof(D3D11_RASTERIZER_DESC) );
    RasterDesc.FillMode = D3D11_FILL_SOLID;
    RasterDesc.CullMode = D3D11_CULL_NONE;
    RasterDesc.DepthClipEnable = TRUE;
    V_RETURN( pd3dDevice->CreateRasterizerState( &RasterDesc, &g_pRasterizerStateSolid ) );
    DXUT_SetDebugName( g_pRasterizerStateSolid, "Solid" );

    RasterDesc.FillMode = D3D11_FILL_WIREFRAME;
    V_RETURN( pd3dDevice->CreateRasterizerState( &RasterDesc, &g_pRasterizerStateWireframe ) );
    DXUT_SetDebugName( g_pRasterizerStateWireframe, "Wireframe" );

    // ------------------------------------
    // The code below TRANSFERS point data to the GPU
    // ------------------------------------

    // ... stack-allocating a vbDesc to set ...
    D3D11_BUFFER_DESC vbDesc;
    ZeroMemory( &vbDesc, sizeof(D3D11_BUFFER_DESC) );
    vbDesc.ByteWidth = sizeof(BEZIER_CONTROL_POINT) * g_MobiusStrip_Size;
    vbDesc.Usage = D3D11_USAGE_DEFAULT;
    vbDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;

    D3D11_SUBRESOURCE_DATA vbInitData;
    ZeroMemory( &vbInitData, sizeof(vbInitData) );
    vbInitData.pSysMem = g_MobiusStrip;
    V_RETURN( pd3dDevice->CreateBuffer( &vbDesc, &vbInitData, &g_pControlPointVB ) );
    DXUT_SetDebugName( g_pControlPointVB, "Control Points" );

    // ... stack-allocating a vbDesc to set ...
    D3D11_BUFFER_DESC vbDesc2;
    ZeroMemory(&vbDesc2, sizeof(D3D11_BUFFER_DESC));
    vbDesc2.ByteWidth = sizeof(SimpleVertex) * g_extraGeometry_Size;
    vbDesc2.Usage = D3D11_USAGE_DEFAULT;
    vbDesc2.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    vbDesc2.CPUAccessFlags = 0; 

    D3D11_SUBRESOURCE_DATA vbInitData2;
    ZeroMemory(&vbInitData2, sizeof(vbInitData2));
    vbInitData2.pSysMem = g_extraGeometry;
    V_RETURN(pd3dDevice->CreateBuffer(&vbDesc2, &vbInitData2, &g_pExtraGeometryVB));
    DXUT_SetDebugName(g_pExtraGeometryVB, "Extra Geometry");

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Create any D3D11 resources that depend on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11ResizedSwapChain( ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain,
                                          const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext )
{
    HRESULT hr;

    V_RETURN( g_DialogResourceManager.OnD3D11ResizedSwapChain( pd3dDevice, pBackBufferSurfaceDesc ) );
    V_RETURN( g_D3DSettingsDlg.OnD3D11ResizedSwapChain( pd3dDevice, pBackBufferSurfaceDesc ) );

    // Setup the camera's projection parameters
    float fAspectRatio = pBackBufferSurfaceDesc->Width / ( FLOAT )pBackBufferSurfaceDesc->Height;
    g_Camera.SetProjParams( XM_PI / 4, fAspectRatio, 0.1f, 200.0f );
    g_Camera.SetWindow( pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height );
    g_Camera.SetButtonMasks( MOUSE_MIDDLE_BUTTON, MOUSE_WHEEL, MOUSE_LEFT_BUTTON );

    g_HUD.SetLocation( pBackBufferSurfaceDesc->Width - 170, 0 );
    g_HUD.SetSize( 170, 170 );
    g_SampleUI.SetLocation( pBackBufferSurfaceDesc->Width - 170, pBackBufferSurfaceDesc->Height - 300 );
    g_SampleUI.SetSize( 170, 300 );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Render the scene using the D3D11 device
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11FrameRender( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime,
                                  float fElapsedTime, void* pUserContext )
{
    // If the settings dialog is being shown, then render it instead of rendering the app's scene
    if( g_D3DSettingsDlg.IsActive() ) {
        g_D3DSettingsDlg.OnRender( fElapsedTime );
        return;
    }
    if (g_shapeChanged) {
       g_shapeChanged = false;
       geometryAssembler.updateDirectXArrays(g_fShape);
       pd3dImmediateContext->UpdateSubresource(g_pExtraGeometryVB, 0, nullptr, g_extraGeometry, 0, 0);
       pd3dImmediateContext->UpdateSubresource(g_pControlPointVB, 0, nullptr, g_MobiusStrip, 0, 0);
    }

    // WVP
    XMMATRIX mProj = g_Camera.GetProjMatrix();
    XMMATRIX mView = g_Camera.GetViewMatrix();

    XMMATRIX mViewProjection = mView * mProj;

    // Update per-frame variables
    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map( g_pcbPerFrame, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
    auto pData = reinterpret_cast<CB_PER_FRAME_CONSTANTS*>( MappedResource.pData );
    XMStoreFloat4x4( &pData->mView, XMMatrixTranspose( mView ));
    XMStoreFloat4x4( &pData->mProj, XMMatrixTranspose( mProj ) );
    XMStoreFloat3( &pData->vCameraPosWorld, g_Camera.GetEyePt() );
    pData->fTessellationFactor = .00015f / (float)(0.01 + 0.99 * g_fSubdivs);
    pData->mNearPlane = (float)g_Camera.GetNearClip();
    pData->mFarPlane = (float)g_Camera.GetFarClip();
    pData->bNurbs = g_bNurbs;

    pd3dImmediateContext->Unmap( g_pcbPerFrame, 0 );

    // Clear the render target and depth stencil
    auto pRTV = DXUTGetD3D11RenderTargetView();
    pd3dImmediateContext->ClearRenderTargetView( pRTV, Colors::Black );
    auto pDSV = DXUTGetD3D11DepthStencilView();
    pd3dImmediateContext->ClearDepthStencilView( pDSV, D3D11_CLEAR_DEPTH, 1.0, 0 );

    // Set state for solid rendering
    pd3dImmediateContext->RSSetState( g_pRasterizerStateSolid );

    // Render the meshes
    // Bind all of the CBs
    pd3dImmediateContext->VSSetConstantBuffers( g_iBindPerFrame, 1, &g_pcbPerFrame );
    pd3dImmediateContext->HSSetConstantBuffers( g_iBindPerFrame, 1, &g_pcbPerFrame );
    pd3dImmediateContext->DSSetConstantBuffers( g_iBindPerFrame, 1, &g_pcbPerFrame );
    pd3dImmediateContext->PSSetConstantBuffers( g_iBindPerFrame, 1, &g_pcbPerFrame );

    // Set the shaders
    pd3dImmediateContext->VSSetShader( g_pVertexShader, nullptr, 0 );
    pd3dImmediateContext->HSSetShader( g_pHullShader,   nullptr, 0);
    pd3dImmediateContext->DSSetShader( g_pDomainShader, nullptr, 0 );
    pd3dImmediateContext->GSSetShader( nullptr, nullptr, 0 );
    pd3dImmediateContext->PSSetShader( g_pPixelShader, nullptr, 0 );

    // Optionally draw the wireframe
    if( g_bDrawWires )
    {
        pd3dImmediateContext->PSSetShader( g_pSolidColorPS, nullptr, 0 );
        pd3dImmediateContext->RSSetState( g_pRasterizerStateWireframe ); 
    }

    // Set the input assembler
    // This sample uses patches with 16 control points each
    // Although the Mobius strip only needs to use a vertex buffer,
    // you can use an index buffer as well by calling IASetIndexBuffer().
    pd3dImmediateContext->IASetInputLayout( g_pPatchLayout );
    UINT Stride = sizeof( BEZIER_CONTROL_POINT );
    UINT Offset = 0;
    pd3dImmediateContext->IASetVertexBuffers( 0, 1, &g_pControlPointVB, &Stride, &Offset );
    pd3dImmediateContext->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_16_CONTROL_POINT_PATCHLIST );

    // Draw the mesh
    pd3dImmediateContext->Draw(g_MobiusStrip_Size, 0 );
    pd3dImmediateContext->RSSetState( g_pRasterizerStateSolid );

    // ----------------------------------
    // attempt to draw extra geometry
    // ----------------------------------

    pd3dImmediateContext->IASetInputLayout(g_pExtraGeometryPatchLayout);
    UINT extraGeometryStride = sizeof(SimpleVertex);
    UINT extraGeometryOffset = 0;
    pd3dImmediateContext->IASetVertexBuffers(0, 1, &g_pExtraGeometryVB, 
       &extraGeometryStride, &extraGeometryOffset);
    pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST); 
   
    pd3dImmediateContext->VSSetShader(g_pExtraGeometryVS, nullptr, 0);
    pd3dImmediateContext->HSSetShader(nullptr, nullptr, 0);
    pd3dImmediateContext->DSSetShader(nullptr, nullptr, 0);
    pd3dImmediateContext->PSSetShader(g_pExtraGeometryPS, nullptr, 0);

    pd3dImmediateContext->Draw(g_extraGeometry_Size, 0);

    // Render the HUD
    DXUT_BeginPerfEvent( DXUT_PERFEVENTCOLOR, L"HUD / Stats" );
    g_HUD.OnRender( fElapsedTime );
    g_SampleUI.OnRender( fElapsedTime );
    RenderText();
    DXUT_EndPerfEvent();
}


//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11ResizedSwapChain 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11ReleasingSwapChain( void* pUserContext )
{
    g_DialogResourceManager.OnD3D11ReleasingSwapChain();
}


//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11CreateDevice 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11DestroyDevice( void* pUserContext )
{
    g_DialogResourceManager.OnD3D11DestroyDevice();
    g_D3DSettingsDlg.OnD3D11DestroyDevice();
    DXUTGetGlobalResourceCache().OnDestroyDevice();
    SAFE_DELETE( g_pTxtHelper );

    SAFE_RELEASE( g_pPatchLayout );
    SAFE_RELEASE( g_pcbPerFrame );
    SAFE_RELEASE( g_pVertexShader );
    SAFE_RELEASE( g_pHullShader );
    SAFE_RELEASE( g_pDomainShader );
    SAFE_RELEASE( g_pPixelShader );
    SAFE_RELEASE( g_pSolidColorPS );
    SAFE_RELEASE( g_pRasterizerStateSolid );
    SAFE_RELEASE( g_pRasterizerStateWireframe );
    SAFE_RELEASE( g_pControlPointVB );

    SAFE_RELEASE(g_pExtraGeometryPatchLayout);
    SAFE_RELEASE(g_pExtraGeometryVS);
    SAFE_RELEASE(g_pExtraGeometryPS);
    SAFE_RELEASE( g_pExtraGeometryVB );
}
