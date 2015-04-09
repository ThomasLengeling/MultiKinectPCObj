#include "MultiKinectPCObjApp.h"

using namespace ci;
using namespace ci::app;
using namespace KinectSdk;
using namespace std;


void MultiKinectPCObjApp::prepareSettings(Settings * settings)
{
	settings->setWindowSize(1024, 768);
	settings->setFrameRate(60.0f);
}

void MultiKinectPCObjApp::setup()
{
	// Set up OpenGL
	//gl::enable(GL_DEPTH_TEST);
	gl::enable(GL_DEPTH_TEST);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_POINT_SMOOTH);
	glPointSize(0.25f);
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();

	DeviceOptions deviceOptions;
	deviceOptions.enableSkeletonTracking(false);
	deviceOptions.enableDepth(true);
	deviceOptions.setDepthResolution(KinectSdk::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );

	// Start all available devices
	int32_t count = Kinect::getDeviceCount();
	console() << "Devices Count " << count << std::endl;


	mStartWritingObjs = false;
	mStartWriting = false;

	mShouldQuit = false;




	for (int32_t i = 0; i < count; i++) {
		deviceOptions.setDeviceIndex(i);
		deviceOptions.setDeviceId(std::to_string(i));

		Device device;
		device.mKinect = Kinect::create();
		device.mKinect->start(deviceOptions);

		//device.mKinect->removeBackground();
		//device.mKinect->setTransform(Kinect::TRANSFORM_SMOOTH);

		device.mDepthCallbackId = device.mKinect->addDepthCallback(&MultiKinectPCObjApp::onDepthData, this);
		device.mTexture = gl::Texture(mDepthDim.x, mDepthDim.y);
		device.mDepthThreaholdBack  = 0.0;
		device.mDepthThreaholdFront = 0.48;
		device.mToggleDepth = false;
		device.mTogglePointCloud = true;

		//CAMERA
		device.mArcball = Arcball(getWindowSize());
		device.mArcball.setRadius((float)getWindowHeight());
		device.mCamera.lookAt(Vec3f(0.0f, 0.0f, 670.0f), Vec3f::zero());
		device.mCamera.setPerspective(60.0f, getWindowAspectRatio(), 0.01f, 5000.0f);

		mDevices.push_back(device);

		console() << device.mKinect->getDeviceOptions().getDeviceIndex() << ": " << device.mKinect->getDeviceOptions().getDeviceId() << endl;
	}


	mCurrentDeviceIdx = 0;
	mAvgFps = 0;
	mFrameCounter = 0;
	mCurrentFrame = 0;

	mParams = params::InterfaceGl::create(getWindow(), "Parameters", toPixels(Vec2i(245, 500)));
	mParams->addText("DEVICE");
	mNumDevices = mDevices.size();
	mParams->addParam("Fps", &mAvgFps, "", true);
	mParams->addParam("Device count", &mNumDevices, "", true);
	mParams->addParam("Current Device", &mCurrentDeviceIdx, "min=0 max=1 step =1");
	mParams->addSeparator();
	for (int32_t i = 0; i < count; i++) {
		int32_t mLocalTilt = mDevices.at(i).mKinect->getTilt();
		mDevices.at(i).mTilt = mLocalTilt;
		mDevices.at(i).mTiltPrev = mLocalTilt;

		std::string result = "Device " + std::to_string(i);
		mParams->addText(result);
		mParams->addParam("Toggle Depth" + std::to_string(i), &mDevices.at(i).mToggleDepth, "");
		mParams->addParam("Toggle PC " + std::to_string(i), &mDevices.at(i).mTogglePointCloud, "");
		mParams->addParam("Tilt " + std::to_string(i), &mDevices.at(i).mTilt, "min=-" + toString(Kinect::MAXIMUM_TILT_ANGLE) + " max=" + toString(Kinect::MAXIMUM_TILT_ANGLE) + " step=1");
		mParams->addParam("depth front " + std::to_string(i), &mDevices.at(i).mDepthThreaholdBack, "min=0 max=1 step=0.01");
		mParams->addParam("depth back " + std::to_string(i), &mDevices.at(i).mDepthThreaholdFront, "min=0 max=1 step=0.01");
		//mParams->addParam("Camera " + std::to_string(i), &mDevices.at(i).mArcball, "");
		mParams->addSeparator();
	}
	mParams->addSeparator();


	mThread = shared_ptr<thread>(new thread(bind(&MultiKinectPCObjApp::writeObj, this)));
}

void MultiKinectPCObjApp::shutdown()
{
	for (uint32_t i = 0; i < mDevices.size(); i++) {
		Device& device = mDevices.at(i);
		device.mKinect->removeCallback(device.mDepthCallbackId);
		device.mKinect->stop();
	}
	mDevices.clear();
	mShouldQuit = true;

	mThread->join();
}

void MultiKinectPCObjApp::mouseDown(ci::app::MouseEvent event)
{
	if (mDevices.size() == MAX_KINECTS){
		mDevices.at(mCurrentDeviceIdx).mArcball.mouseDown(event.getPos());
	}
}

void MultiKinectPCObjApp::mouseDrag(ci::app::MouseEvent event)
{
	if (mDevices.size() == MAX_KINECTS){
		mDevices.at(mCurrentDeviceIdx).mArcball.mouseDrag(event.getPos());
	}
}

// Handles key press
void MultiKinectPCObjApp::keyDown(KeyEvent event)
{
	switch (event.getCode()) {
	case KeyEvent::KEY_q:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen(!isFullScreen());
		break;
	case KeyEvent::KEY_s:

		break;
	case KeyEvent::KEY_1:
		mCurrentDeviceIdx = 0;
		break;
	case KeyEvent::KEY_2:
		mCurrentDeviceIdx = 1;
		break;
	case KeyEvent::KEY_o:
		console() << " start allocating Meshes" << std::endl;
		mStartWriting = true;
	case KeyEvent::KEY_p:
		mStartWritingObjs = true;
		break;

	}
}

void MultiKinectPCObjApp::onDepthData(ci::Surface16u surface, const KinectSdk::DeviceOptions &deviceOptions)
{
	int32_t index = deviceOptions.getDeviceIndex();
	for (size_t i = 0; i < mDevices.size(); ++i) {
		if (index == mDevices.at(i).mKinect->getDeviceOptions().getDeviceIndex()) {
			mDevices.at(i).mTexture = gl::Texture(surface);
			break;
		}
	}
}

void MultiKinectPCObjApp::update()
{
	// Update Kinect
	mAvgFps = getAverageFps();

	for (uint32_t i = 0; i < mDevices.size(); i++) {
		Device & device = mDevices.at(i);
		if (device.mKinect->isCapturing()) {
			device.mKinect->update();

			if (mDevices.size() > 0){




			}

			// Adjust Kinect camera angle, as needed
			if (device.mTilt != device.mTiltPrev) {
				device.mKinect->setTilt(device.mTilt);
				device.mTiltPrev = device.mTilt;
			}
		}
	}
}

void	MultiKinectPCObjApp::writeObj()
{
	ci::ThreadSetup threadSetup;

	while (!mShouldQuit){
		if (mStartWritingObjs){

			console() << "SUCCEFUL WRITING OBJS " << mMeshWriter.size() << " = " << MAXFRAME*mDevices.size()<< std::endl;
			console() << "START WRITING OBJS "<<std::endl;

			int i = 0;
			for (auto it = mMeshWriter.begin(); it != mMeshWriter.end(); ++it){
				int frame = it->first; //local frame
				std::pair<int, ci::TriMesh> objMap = it->second; // frame index, trimesh

				ci::TriMesh triMesh = objMap.second;
				std::ostringstream oss;

				boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

				//oss << "../Objs/model_" << frame <<"_"<< now.date().day_number() << "_" << now.time_of_day().seconds() << ".obj";

			
				if (i % 2 == 0)
					oss << "../Objs/modelLeft_" << objMap.first << ".obj";
				else
					oss << "../Objs/modelRight_" << objMap.first << ".obj";

				console() << "writting  " << oss.str() << " ... " << i << " " << frame << " " << objMap.first << std::endl;
				try{
					ObjLoader::write(writeFile(oss.str()), triMesh);
				}
				catch (exception & e){
					console() << e.what() << std::endl;
				}

				if (i %2 != 0)
					console() << "Done writing Frame: " << objMap.first << std::endl;

				i++;
			}
			console() << "Finalizing" << std::endl;
			mStartWritingObjs = false;
		}
	}
}

void MultiKinectPCObjApp::draw()
{
	// Clear window
	gl::setViewport(getWindowBounds());
	gl::clear(Colorf::black());



	// Draw images

	gl::color(ColorAf::white());
	if (mDevices.size() > 0){
		for (uint32_t i = 0; i < mDevices.size(); ++i) {


			float transx = mDepthDim.x*i + 10 * i;
			float transy = 0;

			if (mDevices.at(i).mKinect->isCapturing()) {

				//GLTEXTURE

				if (mDevices.at(i).mTexture) {
					if (mDevices.at(i).mToggleDepth){
						gl::pushMatrices();
						gl::translate(ci::Vec2f(transx, transy));
						gl::color(1.0, 1.0, 1.0);
						gl::draw(mDevices.at(i).mTexture);
						gl::popMatrices();
					}
				}
			}
		}
	}


	//POINT CLOUD

	if (mDevices.size() > 0){
		for (uint32_t i = 0; i < mDevices.size(); ++i) {
			Device & device = mDevices.at(i);

			gl::pushMatrices();
			gl::setMatrices(device.mCamera);
			gl::rotate(device.mArcball.getQuat());

			if (device.mKinect->isCapturing()) {

				device.mMeshObj.clear();
				if (device.mTogglePointCloud){

					Vec3f offset(Vec2f(mDepthDim) * Vec2f(-0.5f, 0.5f));
					offset.z = device.mCamera.getEyePoint().z;
					Vec3f position = Vec3f::zero();
					// Iterate image rows
					gl::begin(GL_POINTS);
					for (int32_t y = 0; y < mDepthDim.y; y++) {
						for (int32_t x = 0; x < mDepthDim.x; x++) {

							float depth = device.mKinect->getDepthAt(Vec2i(x, y));

							if (depth >  device.mDepthThreaholdBack && depth < device.mDepthThreaholdFront){

								position.z = depth * device.mCamera.getEyePoint().z * -3.0f;
								ci::Vec3f mPoint = position * Vec3f(1.1f, -1.1f, 1.0f) + offset;

								depth = 1.0f - mPoint.z / device.mCamera.getEyePoint().z * -1.5f;
								// Add position to point list
								ci::ColorA colDepth = ColorAf(1.0f*i, depth, 1.0f - depth, depth);
								gl::color(colDepth);
								gl::vertex(mPoint);

								//MESH
								device.mMeshObj.appendVertex(mPoint);
								device.mMeshObj.appendColorRgb(colDepth);
							}

							position.x++;
						}
						position.x = 0;
						position.y++;
					}
					gl::end();

				}


				if (mStartWriting){
					if (mFrameCounter < MAXFRAME*2){
						mCurrentFrame = getElapsedFrames();

						std::pair<int, ci::TriMesh> pairObj(mCurrentFrame, device.mMeshObj);
						mMeshWriter.insert(std::pair<int, std::pair<int, ci::TriMesh> >(mFrameCounter, pairObj));

						console() <<"Device: "<<i <<" - added to mesh, frame " << mCurrentFrame << " " << mFrameCounter << std::endl;
						mFrameCounter++;
					}
					else{
						console() << "DONE SAVING TO OBJS " << std::endl;
						mStartWriting = false;
						mStartWritingObjs = true;

					}
				}
			}

			gl::popMatrices();
		}
	}


	//DRAW PARAMS
	mParams->draw();
}



CINDER_APP_BASIC(MultiKinectPCObjApp, RendererGl)
