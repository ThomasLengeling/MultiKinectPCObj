#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/TriMesh.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Arcball.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "cinder/ObjLoader.h"

#include "Kinect.h"


#define		MAX_KINECTS 2
#define		MAXFRAME	90

static ci::Vec2f mDepthDim(640, 480);

class MultiKinectPCObjApp : public ci::app::AppBasic{
public:
	void	setup();

	void	keyDown(ci::app::KeyEvent event);
	void	mouseDown(ci::app::MouseEvent event);
	void    mouseDrag(ci::app::MouseEvent event);
	void	prepareSettings(ci::app::AppBasic::Settings *settings);

	void	update();
	void	draw();
	void	shutdown();

private:

	//THREAD WRITE
	void									writeObj();
	std::shared_ptr<std::thread>			mThread;

	// Device info
	struct Device
	{
		int32_t								mDepthCallbackId;
		KinectSdk::KinectRef				mKinect;
		ci::gl::Texture						mTexture;
		int32_t								mTilt;
		int32_t								mTiltPrev;

		std::vector<ci::Vec3f>				mPoints;

		float								mDepthThreaholdBack;
		float								mDepthThreaholdFront;

		ci::TriMesh							mMeshObj;
		ci::gl::VboMesh						mVboMeshObj;

		bool								mToggleDepth;
		bool								mTogglePointCloud;

		// Camera
		ci::Arcball							mArcball;
		ci::CameraPersp						mCamera;

	};

	//OBS
	bool									mStartWritingObjs;
	bool									mStartWriting;

	volatile bool							mShouldQuit;
	int										mFrameCounter;
	int										mCurrentFrame;

	std::map<int, std::pair<int, ci::TriMesh> >				mMeshWriter;


	int										mCurrentDeviceIdx;

	std::vector<Device>						mDevices;

	int32_t									mNumDevices;

	// Callback
	void									onDepthData(ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions);

	//PARAMS
	ci::params::InterfaceGlRef				mParams;
	float									mAvgFps;
};

