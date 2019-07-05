/*****************************************************************************

                                   main.cpp

						Copyright 2001, John J. Bolton
	----------------------------------------------------------------------

	$Header: //depot/Libraries/PathFinder/Test/main.cpp#2 $

	$NoKeywords: $

*****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <cmath>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>

#include <gl/gl.h>
#include <gl/glu.h>
#include <gl/glaux.h>

#include "Glx/Glx.h"
#include "Wglx/Wglx.h"
#include "Wx/Wx.h"

#include "Misc/Etc.h"
#include "Misc/Random.h"
#include "Misc/Trace.h"
#include "Misc/Max.h"
#include "Math/Constants.h"
#include "Math/Vector3f.h"
#include "Math/Vector2f.h"

#include "TgaFile/TgaFile.h"
#include "TerrainCamera/TerrainCamera.h"
#include "HeightField/HeightField.h"
#include "Water/Water.h"

#include "../PathFinder.h"

int const	WATER_TO_LAND_RATIO	= 4;
int const	PATH_TO_LAND_RATIO	= 4;
float const	XY_SCALE			= 1.f;
float const	Z_SCALE				= 32.f;

static LRESULT CALLBACK WindowProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );
static void InitializeRendering();
static void Display();
static void Reshape( int w, int h );

static void Update( HWND hWnd );
static void ReportGlErrors( GLenum error );
static void UpdateWater( DWORD newTime );

static void DrawWater();
static void DrawTerrain();

static Vector3f ComputeNormal( HeightField const & hf, int x, int y );
static void ComputeNormals( HeightField const & hf, Vector3f * paNormals );

static float ComputeTerrainCost( int fx, int fy, int tx, int ty );
static void ComputePath( int fx, int fy, int tx, int ty );
static void InitializeEdge( PathFinder::Edge * pEdge, int x0, int y0, int x1, int y1, int sx, int sy );
static void Drawpath();

static char						s_aAppName[]	= "Path Finder";
static char						s_aTitleBar[]	= "Path Finder";

static Glx::Lighting *			s_pLighting;
static Glx::DirectionalLight *	s_pDirectionalLight;

static HeightField *			s_pTerrain;
static Vector3f *				s_paTerrainNormals;
static Glx::Mesh *				s_pTerrainMesh;
static Glx::MipMappedTexture *	s_pTerrainTexture;
static Glx::Material *			s_pTerrainMaterial;

static Water *					s_pWater;
static Glx::Material *			s_pWaterMaterial;

static Glx::Mesh *				s_pPathMesh;
static Glx::Material *			s_pPathMaterial;

static TerrainCamera *			s_pCamera;

static float					s_CameraSpeed				= 2.f;
static double					s_SeaLevel					= Z_SCALE * .25;
static float					s_DownhillCost				= .5f;
static float					s_UphillCost				= 1.f;

static Random					s_Random( timeGetTime() );
static PathFinder::Path			s_Path;

static inline int PSizeX()
{
	return ( s_pTerrain->GetSizeX() - 1 ) / PATH_TO_LAND_RATIO + 1 - 2;
}

static inline int PSizeY()
{
	return ( s_pTerrain->GetSizeY() - 1 ) / PATH_TO_LAND_RATIO + 1 - 2;
}

static inline int P2T( int a )
{
	return ( a + 1 ) * PATH_TO_LAND_RATIO;
}

static inline int T2P( int a )
{
	return a / PATH_TO_LAND_RATIO - 1;
}

static inline int WSizeX()
{
	return ( s_pTerrain->GetSizeX() - 1 ) / WATER_TO_LAND_RATIO + 1;
}

static inline int WSizeY()
{
	return ( s_pTerrain->GetSizeY() - 1 ) / WATER_TO_LAND_RATIO + 1;
}

static inline int W2T( int a )
{
	return a * WATER_TO_LAND_RATIO;
}

static inline int T2W( int a )
{
	return a / WATER_TO_LAND_RATIO;
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

int WINAPI WinMain( HINSTANCE hInstance, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow )
{
	if ( Wx::RegisterWindowClass(	CS_OWNDC,
									( WNDPROC )WindowProc,
									0,
									0,
									hInstance,
									NULL,
									LoadCursor( NULL, IDC_ARROW ),
									NULL,
									NULL,
									s_aAppName ) == NULL )
	{
		MessageBox( NULL, "Wx::RegisterWindowClass() failed.", "Error", MB_OK );
		exit( 1 );
	}

	HWND hWnd = CreateWindowEx( 0,
								s_aAppName,
								s_aTitleBar,
								WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
								0, 0, 500, 500,
								NULL,
								NULL,
								hInstance,
								NULL );

	if ( hWnd == NULL )
	{
		MessageBox( NULL, "CreateWindowEx() failed.", "Error", MB_OK );
		exit( 1 );
	}

	// Create the terrain height field

	try
	{
		s_pTerrain = new HeightField( TgaFile( "hf.tga" ), XY_SCALE, Z_SCALE );
		if ( !s_pTerrain ) throw std::bad_alloc();
	
		std::ostringstream	buffer;

		buffer << "Heightfield size - x: " << s_pTerrain->GetSizeX() << ", y: " << s_pTerrain->GetSizeY() << std::endl << std::ends;
		OutputDebugString( buffer.str().c_str() );
	}
	catch( ... )
	{
			MessageBox(NULL, "Unable to load terrain height field.", "Water", MB_OK );
			throw;
	}

	// Compute the normals

	s_paTerrainNormals = new Vector3f[ s_pTerrain->GetSizeX() * s_pTerrain->GetSizeY() ];
	if ( !s_pTerrain ) throw std::bad_alloc();

	ComputeNormals( *s_pTerrain, s_paTerrainNormals);

	// Create the water

	s_pWater = new Water( WSizeX(), WSizeY(), WATER_TO_LAND_RATIO * XY_SCALE, 20.f, .99f );
	if ( !s_pWater ) throw std::bad_alloc();

	HDC const	hDC	= GetDC( hWnd );
	int			rv;

	WGlx::SetPixelFormat( hDC, 0, true );

	{
		WGlx::CurrentRenderingContext	rc( hDC );	// Current rendering context

		InitializeRendering();

		s_pCamera			= new TerrainCamera( 60.f, 1.f, 1000.f,
												 Vector3f( 0.f, -s_pTerrain->GetSizeY() * XY_SCALE * .5f, Z_SCALE ),
												 -90.f, 90.f, 90.f );
		if ( !s_pCamera ) throw std::bad_alloc();

		s_pLighting			= new Glx::Lighting( Glx::Rgba( .4f, .4f, .4f ) );
		if ( !s_pLighting ) throw std::bad_alloc();
		s_pDirectionalLight	= new Glx::DirectionalLight( GL_LIGHT0, Vector3f( 1.f, -1.f, 1.f ), Glx::Rgba( .6f, .6f, .6f ) );
		if ( !s_pDirectionalLight ) throw std::bad_alloc();

		int const	terrainTextureSize	= 256;

		s_pTerrainTexture	= new Glx::MipMappedTexture( terrainTextureSize, terrainTextureSize, GL_BGR_EXT, GL_UNSIGNED_BYTE );
		if ( !s_pTerrainTexture ) throw std::bad_alloc();

		try
		{
//			TgaFile	file( "256.tga" );
			TgaFile	file( "drock011.tga" );
//			TgaFile	file( "drock032.tga" );
			bool	ok;
			
			assert( file.m_ImageType == TgaFile::IMAGE_TRUECOLOR );
			assert( file.m_Height == terrainTextureSize && file.m_Width == terrainTextureSize );
			
			unsigned char *	s_pTextureData	= new unsigned char[ terrainTextureSize * terrainTextureSize * 4 ];
			if ( !s_pTextureData ) throw std::bad_alloc();
			
			ok = file.Read( s_pTextureData );
			assert( ok );
			
			s_pTerrainTexture->BuildAllMipMaps( s_pTextureData );
			
			delete[] s_pTextureData;
		}
		catch ( ... )
		{
			MessageBox(NULL, "Unable to load terrain texture.", "Water", MB_OK );
			throw;
		}

		s_pTerrainMaterial	= new Glx::Material( s_pTerrainTexture/*,
												 GL_MODULATE,
												 Glx::Lighting::WHITE,
												 Glx::Lighting::BLACK, 0.f,
												 Glx::Lighting::BLACK,
												 GL_SMOOTH,
												 GL_FRONT_AND_BACK*/ );
		if ( !s_pTerrainMaterial ) throw std::bad_alloc();

		s_pTerrainMesh			= new Glx::Mesh( s_pTerrainMaterial );
		if ( !s_pTerrainMesh ) throw std::bad_alloc();

		s_pTerrainMesh->Begin();
		DrawTerrain();
		s_pTerrainMesh->End();

		s_pWaterMaterial	= new Glx::Material( 0,
												 GL_MODULATE,
												 Glx::Rgba( .35f, .5f, 7.f, .75f )/*,
												 Glx::Rgba( 1.f, 1.f, 1.f, 1.f ), 128.f,
												 Glx::Lighting::BLACK,
												 GL_SMOOTH,
												 GL_FRONT_AND_BACK*/ );
		if ( !s_pWaterMaterial ) throw std::bad_alloc();

		s_pPathMaterial	= new Glx::Material( Glx::Lighting::WHITE/*,
												 Glx::Lighting::BLACK, 0.f,
												 Glx::Lighting::GRAY,
												 GL_SMOOTH,
												 GL_FRONT_AND_BACK*/ );
		if ( !s_pPathMaterial ) throw std::bad_alloc();

		SetTimer( hWnd, 0, 1000, NULL );

		ShowWindow( hWnd, nCmdShow );

		rv = Wx::MessageLoop( hWnd, Update );

		delete s_pPathMaterial;
		delete s_pWaterMaterial;
		delete s_pTerrainMesh;
		delete s_pTerrainMaterial;
		delete s_pTerrainTexture;
		delete s_pDirectionalLight;
		delete s_pLighting;
		delete s_pCamera;
		delete s_pWater;
		delete[] s_paTerrainNormals;
		delete s_pTerrain;

	}

	ReleaseDC( hWnd, hDC );
	DestroyWindow( hWnd );

	return rv;
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void Update( HWND hWnd )
{
	UpdateWater( timeGetTime() );
	InvalidateRect( hWnd, NULL, FALSE );
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static LRESULT CALLBACK WindowProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{ 
	switch( uMsg )
	{
	case WM_PAINT:
	{
		static PAINTSTRUCT ps;

		Display();
		BeginPaint( hWnd, &ps );
		EndPaint( hWnd, &ps );
		return 0;
	}

	case WM_SIZE:
		Reshape( LOWORD( lParam ), HIWORD( lParam ) );
		PostMessage( hWnd, WM_PAINT, 0, 0 );
		return 0;

	case WM_CHAR:
	{
		switch ( wParam )
		{
		case VK_ESCAPE:	// Quit
			PostQuitMessage( 0 );
			break;

		case ' ':	// Forward
			s_pCamera->Move( s_pCamera->GetDirection() * s_CameraSpeed );
			break;

		case 's':	// Backwards
			s_pCamera->Move( -s_pCamera->GetDirection() * s_CameraSpeed );
			break;

		case 'd':	// Strafe right
			s_pCamera->Move( s_pCamera->GetRight() * s_CameraSpeed );
			break;

		case 'a':	// Strafe left
			s_pCamera->Move( -s_pCamera->GetRight() * s_CameraSpeed );
			break;

		case 'w':	// Strafe up
			s_pCamera->Move( s_pCamera->GetUp() * s_CameraSpeed );
			break;

		case 'x':	// Strafe down
			s_pCamera->Move( -s_pCamera->GetUp() * s_CameraSpeed );
			break;

		case '1':
			s_SeaLevel -= Z_SCALE * .01;
			trace( "Sea level = %f\n", s_SeaLevel );
			break;

		case '2':
			s_SeaLevel += Z_SCALE * .01;
			trace( "Sea level = %f\n", s_SeaLevel );
			break;

		case '7':
			s_DownhillCost -= 1.f;
			trace( "Downhill Cost = %f\n", s_DownhillCost );
			break;

		case '8':
			s_DownhillCost += 1.f;
			trace( "Downhill cost = %f\n", s_DownhillCost );
			break;

		case '9':
			s_UphillCost -= 1.f;
			trace( "Uphill Cost = %f\n", s_UphillCost );
			break;

		case '0':
			s_UphillCost += 1.f;
			trace( "Uphill cost = %f\n", s_UphillCost );
			break;

		case 'p':
			ComputePath( P2T( 0 ), P2T( 0 ), P2T( PSizeX() - 1 ), P2T( PSizeY() - 1 ) );
			break;
		}

		return 0;
	}

	case WM_KEYDOWN:
		switch ( wParam )
		{
		case VK_UP:
			s_pCamera->Pitch( 5.f );
			break;

		case VK_DOWN:
			s_pCamera->Pitch( -5.f );
			break;

		case VK_LEFT:
			s_pCamera->Yaw( 5.f );
			break;

		case VK_RIGHT:
			s_pCamera->Yaw( -5.f );
			break;

		case VK_PRIOR:
			s_pCamera->Roll( 5.f );
			break;

		case VK_INSERT:
			s_pCamera->Roll( -5.f );
			break;
		}
		return 0;

	case WM_TIMER:
		{
			int const		RADIUS	=	3;
			float const		H		= Z_SCALE *.125f;
			float const		L		= 8.f;
			int	const		x0		= s_Random.Next() % ( s_pWater->GetSizeX() - 2*RADIUS ) + RADIUS;
			int const		y0		= s_Random.Next() % ( s_pWater->GetSizeY() - 2*RADIUS ) + RADIUS;

			for ( int i = -(RADIUS-1); i < RADIUS; i++ )
			{
				for ( int j = -(RADIUS-1); j < RADIUS; j++ )
				{
					s_pWater->GetData( x0+j, y0+i )->m_Z = H * cos( Math::TWO_PI * sqrt( i*i + j*j ) / L );
				}
			}
		}
		return 0;

	case WM_CLOSE:
		PostQuitMessage( 0 );
		return 0;
	}

	return DefWindowProc( hWnd, uMsg, wParam, lParam ); 
} 


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void InitializeRendering()
{
	glClearColor( .65f, .80f, .90f, 1.f );
	glEnable( GL_DEPTH_TEST );
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void UpdateWater( DWORD newTime )
{
	static DWORD	oldTime	= timeGetTime();

	// Compute the new heights

	s_pWater->Update( ( newTime - oldTime ) * .001f );

	// Apply a damping factor due to land

	int const					sx		= s_pWater->GetSizeX();
	int const					sy		= s_pWater->GetSizeY();

	for ( int y = 1; y < sy - 1; y++ )
	{
		for ( int x = 1; x < sx - 1; x++ )
		{
			double const	depth	= ( s_SeaLevel - s_pTerrain->GetZ( W2T( x ), W2T( y ) ) );

			s_pWater->GetData( x, y )->m_Z	*= limit( 0., depth / ( s_SeaLevel * .25 ), 1. );
		}
	}

	// Save the new time

	oldTime = newTime;
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void DrawWater()
{
	glPushMatrix();

	int const					sx		= s_pWater->GetSizeX();
	int const					sy		= s_pWater->GetSizeY();
	float const					d		= s_pWater->GetXYScale();
	HeightField::Vertex const *	pData	= s_pWater->GetData();
	float const					x0		= -( sx - 1 ) * .5f * d;
	float const					y0		= -( sy - 1 ) * .5f * d;

	// Compute water normals

	Vector3f * const	paWaterNormals	= new Vector3f[ sx * sy ];
	ComputeNormals( s_pWater->GetHeightField(), paWaterNormals );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	s_pWaterMaterial->Apply();

	glEnableClientState( GL_NORMAL_ARRAY );
	glNormalPointer( GL_FLOAT, sizeof( *paWaterNormals ), paWaterNormals );

	// Raise the water to sealevel

	glTranslatef( 0.f, 0.f, s_SeaLevel );

	for ( int i = 0; i < sy-1; i += 1 )
	{
		glBegin( GL_TRIANGLE_STRIP );

		for ( int j = 0; j < sx; j += 1 )
		{
			float const	x	= x0 + j * d;
			float const	tx	= float( j ) * .125f;//float( j ) / float( sx - 1 );

			HeightField::Vertex const * const	pa	= &pData[ i * sx + j ];
			HeightField::Vertex const * const	pb	= pa + sx;

			float const	yb	= y0 + ( i + 1 ) * d;
			float const	tyb	= float( i + 1 ) * .125f;//float( i + 1 ) / float( sy - 1 );
			float const	zb	= pb->m_Z;

			glArrayElement( ( i + 1 ) * sx + j );
//			glNormal3fv( pb->m_Normal.m_V );
			glVertex3f( x, yb, zb );

			float const	ya	= y0 + i * d;
			float const	tya	= float( i ) * .125f;//float( i ) / float( sy - 1 );
			float const	za	= pa->m_Z;

			glArrayElement( i * sx + j );
//			glNormal3fv( pa->m_Normal.m_V );
			glVertex3f( x, ya, za );
		}

		glEnd();
	}

	glDisableClientState( GL_NORMAL_ARRAY );
	glDisable( GL_BLEND );

	delete[] paWaterNormals;

	glPopMatrix();
}



/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void DrawTerrain()
{
	int const					sx		= s_pTerrain->GetSizeX();
	int const					sy		= s_pTerrain->GetSizeY();
	HeightField::Vertex const *	pData	= s_pTerrain->GetData();
	float const					d		= s_pTerrain->GetXYScale();
	float const					x0		= -( sx - 1 ) * .5f * XY_SCALE;
	float const					y0		= -( sy - 1 ) * .5f * XY_SCALE;

	s_pTerrainMaterial->Apply();

	glEnableClientState( GL_NORMAL_ARRAY );
	glNormalPointer( GL_FLOAT, sizeof( *s_paTerrainNormals ), s_paTerrainNormals );

	for ( int i = 0; i < sy-1; i += 1 )
	{
		glBegin( GL_TRIANGLE_STRIP );

		for ( int j = 0; j < sx; j += 1 )
		{
			float const	x	= x0 + j * d;
			float const	tx	= float( j ) * .125f;//float( j ) / float( sx - 1 );

			HeightField::Vertex const * const	pa	= &pData[ i * sx + j ];
			HeightField::Vertex const * const	pb	= pa + sx;

			float const	yb	= y0 + ( i + 1 ) * d;
			float const	tyb	= float( i + 1 ) * .125f;//float( i + 1 ) / float( sy - 1 );
			float const	zb	= pb->m_Z;

			glTexCoord2f( tx, tyb );
			glArrayElement( ( i + 1 ) * sx + j );
//			glNormal3fv( pb->m_Normal.m_V );
			glVertex3f( x, yb, zb );

			float const	ya	= y0 + i * d;
			float const	tya	= float( i ) * .125f;//float( i ) / float( sy - 1 );
			float const	za	= pa->m_Z;

			glTexCoord2f( tx, tya );
			glArrayElement( i * sx + j );
//			glNormal3fv( pa->m_Normal.m_V );
			glVertex3f( x, ya, za );
		}

		glEnd();
	}

	glDisableClientState( GL_NORMAL_ARRAY );
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void ComputeNormals( HeightField const & hf, Vector3f * paNormals )
{
	for ( int y = 0; y < hf.GetSizeY(); y++ )
	{
		for ( int x = 0; x < hf.GetSizeX(); x++ )
		{

			*paNormals = ComputeNormal( hf, x, y );
			++paNormals;
		}
	}
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static Vector3f ComputeNormal( HeightField const & hf, int x, int y )
{
	HeightField::Vertex const * const	pV	= hf.GetData( x, y );
	float const							s	= hf.GetXYScale();
	int const							sx	= hf.GetSizeX();
	int const							sy	= hf.GetSizeY();

	float const							x0	= x * s;
	float const							y0	= y * s;
	Vector3f const						v0	( x0, y0, pV[ 0 ].m_Z );

	Vector3f	normal	= Vector3f::ORIGIN;

	if ( x + 1 < sx && y + 1 < sy )
	{
		normal += Glx::ComputeFaceNormal( v0,
										  Vector3f( x0 + s, y0,     pV[  1      ].m_Z ),
										  Vector3f( x0 + s, y0 + s, pV[  sx + 1 ].m_Z ) );

		normal += Glx::ComputeFaceNormal( v0,
										  Vector3f( x0 + s, y0 + s, pV[  sx + 1 ].m_Z ),
										  Vector3f( x0,     y0 + s, pV[  sx     ].m_Z ) );
	}

	if ( x - 1 >= 0 && y + 1 < sy )
	{
		normal += Glx::ComputeFaceNormal( v0,
										  Vector3f( x0,     y0 + s, pV[  sx     ].m_Z ),
										  Vector3f( x0 - s, y0,     pV[ -1      ].m_Z ) );

//		normal += Glx::ComputeFaceNormal( v0,
//										  Vector3f( x0,     y0 + s, pV[  sx     ].m_Z ),
//										  Vector3f( x0 - s, y0 + s, pV[  sx - 1 ].m_Z ) );
//
//		normal += Glx::ComputeFaceNormal( v0,
//										  Vector3f( x0 - s, y0 + s, pV[  sx - 1 ].m_Z ),
//										  Vector3f( x0 - s, y0,     pV[ -1      ].m_Z ) );
	}

	if ( x - 1 >= 0 && y - 1 >= 0 )
	{
		normal += Glx::ComputeFaceNormal( v0,
										  Vector3f( x0 - s, y0,     pV[ -1      ].m_Z ),
										  Vector3f( x0 - s, y0 - s, pV[ -sx - 1 ].m_Z ) );

		normal += Glx::ComputeFaceNormal( v0,
										  Vector3f( x0 - s, y0 - s, pV[ -sx - 1 ].m_Z ),
										  Vector3f( x0,     y0 - s, pV[ -sx     ].m_Z ) );
	}

	if ( x + 1 < sx && y - 1 >= 0 )
	{
		normal += Glx::ComputeFaceNormal( v0,
										  Vector3f( x0,     y0 - s, pV[ -sx     ].m_Z ),
										  Vector3f( x0 + s, y0,     pV[  1      ].m_Z ) );

//		normal += Glx::ComputeFaceNormal( v0,
//										  Vector3f( x0,     y0 - s, pV[ -sx     ].m_Z ),
//										  Vector3f( x0 + s, y0 - s, pV[ -sx + 1 ].m_Z ) );
//
//		normal += Glx::ComputeFaceNormal( v0,
//										  Vector3f( x0 + s, y0 - s, pV[ -sx + 1 ].m_Z ),
//										  Vector3f( x0 + s, y0,     pV[  1      ].m_Z ) );
	}

	return normal.Normalize();
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

class TerrainPathFindingNode : public PathFinder::Node
{
public:
		// Return the estimated cost from this node to the goal. This value
		// is assumed to be constant.
	virtual float	h( PathFinder::Node const & goal ) const { return m_DistanceToGoal; }
	
	//	// Update pathfinding values for the node
	//	void Update( float predCost, float edgeCost, Node * pPred );
	//
	//	// Anticipate being added to the open queue
	//	void Open( float predCost, float edgeCost, Node * pPred, Node const & goal );
	//
	//	// Returns true if the node is open.
	//	bool IsOpen() const		{ return m_Status == NS_OPEN; }
	//
	//	// Anticipate being closed.
	//	void Close();
	//
	//	// Returns true if the node is closed
	//	bool IsClosed() const	{ return m_Status == NS_CLOSED; }
	//
	//	// Reset the node's status to not visited
	//	void ResetNodeStatus();

	float m_DistanceToGoal;
};


static TerrainPathFindingNode *	s_pNodeData	= 0;
static PathFinder::Edge *		s_pEdgeData	= 0;


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void ComputePath( int fx, int fy, int tx, int ty )
{
	int const	sx				= PSizeX();
	int const	sy				= PSizeY();
	int const	nNodes			= sx * sy;

	PathFinder	pf( nNodes );
	delete[] s_pNodeData;
	delete[] s_pEdgeData;

	s_pNodeData = new TerrainPathFindingNode[ nNodes ];
	s_pEdgeData = new PathFinder::Edge[ nNodes * 8 ];

	int			x, y;

	// Build the pathfinder graph edge data

	for ( y = 0; y < sy; y++ )
	{
		for ( x = 0; x < sx; x++ )
		{
			PathFinder::Edge * const	pEL	= &s_pEdgeData[ ( y * sx + x ) * 8 ];

			InitializeEdge( pEL + 0, x, y, x-1, y+1, sx, sy );
			InitializeEdge( pEL + 1, x, y, x  , y+1, sx, sy );
			InitializeEdge( pEL + 2, x, y, x+1, y+1, sx, sy );
			InitializeEdge( pEL + 3, x, y, x-1, y  , sx, sy );
			InitializeEdge( pEL + 4, x, y, x+1, y  , sx, sy );
			InitializeEdge( pEL + 5, x, y, x-1, y-1, sx, sy );
			InitializeEdge( pEL + 6, x, y, x  , y-1, sx, sy );
			InitializeEdge( pEL + 7, x, y, x+1, y-1, sx, sy );

		}
	}

	// Build the pathfinding graph node data

	for ( y = 0; y < sy; y++ )
	{
		for ( x = 0; x < sx; x++ )
		{
			// Add the edges to the node

			PathFinder::Edge * const		pEL	= &s_pEdgeData[ ( y * sx + x ) * 8 ];
			TerrainPathFindingNode * const	pN	= &s_pNodeData[ y * sx + x ];

			pN->m_DistanceToGoal = ( Vector2f( tx, ty ) - Vector2f( x, y ) ).Length();
			pN->m_AdjacencyList.reserve( 8 );

			for ( int i = 0; i < 8; i++ )
			{
				if ( pEL[ i ].m_pTo != 0 )
				{
					pN->m_AdjacencyList.push_back( &pEL[ i ] );
				}
			}

			// Add the node to the graph

			pf.m_ConnectivityGraph[ y * sx + x ] = pN;
		}
	}

	pf.FindPath( &s_pNodeData[ T2P( fy ) * sx + T2P( fx ) ],
				 &s_pNodeData[ T2P( ty ) * sx + T2P( tx ) ],
				 &s_Path );
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

void InitializeEdge( PathFinder::Edge * pEdge, int x0, int y0, int x1, int y1, int sx, int sy )
{
	int const	wx		= P2T( x1 );
	int const	wy		= P2T( y1 );

	if ( x1 >= 0 && x1 < sx &&
		 y1 >= 0 && y1 < sy &&
		 s_pTerrain->GetZ( wx, wy ) >= s_SeaLevel )
	{
		pEdge->m_pTo = &s_pNodeData[ y1 * sx + x1 ];
		pEdge->m_Cost = ComputeTerrainCost( x0, y0, x1 , y1 );
	}
	else
	{
		pEdge->m_pTo = 0;
	}
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static float ComputeTerrainCost( int fx, int fy, int tx, int ty )
{
	float const	fz	= s_pTerrain->GetZ( P2T( fx ), P2T( fy ) );
	float const	tz	= s_pTerrain->GetZ( P2T( tx ), P2T( ty ) );

	float const	dxy	= ( Vector2f( tx, ty ) - Vector2f( fx, fy ) ).Length();

	if ( fz < tz )
	{
		return dxy * ( 1.f + ( tz - fz ) * s_UphillCost );
	}
	else
	{
		return dxy * ( 1.f + ( fz - tz ) * s_DownhillCost );
	}
}




/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void DrawPath()
{
	s_pPathMaterial->Apply();

	glPushMatrix();

	glTranslatef( -( s_pTerrain->GetSizeX() - 1 ) * XY_SCALE * .5f, -( s_pTerrain->GetSizeY() - 1 ) * XY_SCALE * .5f, 0.f );

	for ( PathFinder::Path::reverse_iterator p = s_Path.rbegin(); p != s_Path.rend(); ++p )
	{
		glPushMatrix();

		TerrainPathFindingNode * const pN	= static_cast< TerrainPathFindingNode * >( *p );
		int const	y	= P2T( ( pN - s_pNodeData ) / PSizeX() ); 
		int const	x	= P2T( ( pN - s_pNodeData ) % PSizeX() ); 
		
		glTranslatef( x * XY_SCALE, y * XY_SCALE, s_pTerrain->GetZ( x, y ) );
		auxSolidSphere( XY_SCALE );

		glPopMatrix();
	}

	glPopMatrix();
}



/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void Display()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glMatrixMode( GL_MODELVIEW );

	glLoadIdentity();

	// Place the camera

	s_pCamera->Look();

	// Lights

	s_pDirectionalLight->Apply();

	// Draw the path

	DrawPath();

//	s_pPathMesh->Apply();

	// Draw the terrain

	s_pTerrainMesh->Apply();

	// Draw the water

	DrawWater();

	// Display the scene

	glFlush();
	SwapBuffers( wglGetCurrentDC() );

	GLenum const	error	= glGetError();
	if ( error != GL_NO_ERROR )
	{
		ReportGlErrors( error );
	}
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void ReportGlErrors( GLenum error )
{
	std::ostringstream buffer;

	buffer << "*** glError returned errors: " << std::endl;
		
	do
	{
		buffer << gluErrorString( error ) << std::endl;

	} while ( ( error = glGetError() ) != GL_NO_ERROR );

	buffer << std::ends;

	OutputDebugString( buffer.str().c_str() );
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

static void Reshape( int w, int h )
{
	s_pCamera->Reshape( w, h );
}


