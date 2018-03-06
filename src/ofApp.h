#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"
#include "Kinect.h"
#include "ComPtr.h"

#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
}

#define SERVER_PORT
#define SERVER_IP_ADDR

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		bool isValidDepthRange(int index);
		bool isValidColorRange(int x, int y);
		bool isInColorCircle(int x, int y);

		IKinectSensor* kinect = nullptr;
		IColorFrameReader* colorFrameReader = nullptr;
		std::vector<BYTE> colorBuffer;

		IDepthFrameReader* depthFrameReader = nullptr;
		std::vector<UINT16> depthBuffer;

		int colorWidth;
		int colorHeight;
		const int ColorBytesPerPixel = 4;

		UINT16 minDepthReliableDistance;
		UINT16 maxDepthReliableDistance;

		int depthWidth;
		int depthHeight;

		int depthPointX;
		int depthPointY;

		ofxPanel gui;
		ofxIntSlider minDepth;
		ofxIntSlider baseDepth;
		ofxIntSlider circleResolution;
		ofxIntSlider circlePointX;
		ofxIntSlider circlePointY;
		ofxToggle  depthOrcolor;
		bool showMenu = true;
};
