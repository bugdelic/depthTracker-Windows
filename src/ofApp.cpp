#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofBackground(0);
	ofSetFrameRate(30);
	ofSetBackgroundAuto(true);

	// デフォルトのKinectを取得する
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));

	// Kinectを開く
	ERROR_CHECK(kinect->Open());

	BOOLEAN isOpen = false;
	ERROR_CHECK(kinect->get_IsOpen(&isOpen));
	if (!isOpen) {
		throw std::runtime_error("Kinectが開けません");
	}

	// カラーリーダーを取得する
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	// カラー画像のサイズを取得する
	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->get_FrameDescription(&colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));

	// Depthリーダーを取得する
	ComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
	ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

	// Depth画像のサイズを取得する
	ComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
	ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
	ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));

	depthPointX = depthWidth / 2;
	depthPointY = depthHeight / 2;

	// Depthの最大値、最小値を取得する
	ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minDepthReliableDistance));
	ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxDepthReliableDistance));

	std::cout << "Depthデータの幅   : " << depthWidth << std::endl;
	std::cout << "Depthデータの高さ : " << depthHeight << std::endl;

	std::cout << "Depth最小値       : " << minDepthReliableDistance << std::endl;
	std::cout << "Depth最大値       : " << maxDepthReliableDistance << std::endl;

	// バッファーを作成する
	colorBuffer.resize(colorWidth * colorHeight * ColorBytesPerPixel);
	depthBuffer.resize(depthWidth * depthHeight);

	// GUIを作成
	gui.setup();
	gui.add(minDepth.setup("min depth", minDepthReliableDistance, minDepthReliableDistance, maxDepthReliableDistance));
	gui.add(baseDepth.setup("base depth", maxDepthReliableDistance, minDepthReliableDistance, maxDepthReliableDistance));
	gui.add(circleResolution.setup("circle resolution", 100, 10, colorHeight));
	gui.add(circlePointX.setup("circle center x", colorWidth/2, 0, colorWidth));
	gui.add(circlePointY.setup("circle center y", colorHeight/2, 0, colorHeight));
	gui.add(depthOrcolor.setup("color or depth", true));
	gui.add(detectThreshold.setup("cv threshold", 200, 1, 400));
	gui.add(cvMinArea.setup("cv min area", 10, 1, 200));
	gui.add(cvMaxArea.setup("cv max area", 2000, 1, 100000));
	gui.add(cvMaxTrackRect.setup("cv max track", 10, 1, MAX_CV_TRACK_RECT));
	gui.add(oscSendFrameCounter.setup("osc send frame%counter", 30, 10, 300));

	// OSC用
	oscSender.setup(SERVER_IP_ADDR, SERVER_PORT);
}

//--------------------------------------------------------------
void ofApp::update(){

	ComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (ret == S_OK) {
		// BGRAの形式でデータを取得する
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
			colorBuffer.size(), &colorBuffer[0], ColorImageFormat_Bgra));
	}

	ComPtr<IDepthFrame> depthFrame;
	ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
	if (ret == S_OK) {
		ERROR_CHECK(depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]));
	}

	ComPtr<ICoordinateMapper> mapper;
	ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

	std::vector<DepthSpacePoint> depthSpacePoints(colorWidth * colorHeight);
	ERROR_CHECK(mapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0],
		depthSpacePoints.size(), &depthSpacePoints[0]));

	// 重心データのクリア
	trackRectCenters.clear();

	// 各種メモリ確保
	if (!colorImage.isAllocated())
		colorImage.allocate(colorWidth, colorHeight, OF_IMAGE_COLOR_ALPHA);

	if (!depthImage.isAllocated())
		depthImage.allocate(colorWidth, colorHeight, OF_IMAGE_GRAYSCALE);

	// アクセス用ポインタ取得
	unsigned char *dataColor = colorImage.getPixels().getData();
	unsigned char *dataDepth = depthImage.getPixels().getData();

	// depth/colorデータ取得
	for (int j = 0; j < colorImage.getWidth(); ++j) {
		for (int i = 0; i < colorImage.getHeight(); ++i) {
			int idx = i * colorImage.getWidth() + j;
			int x = (int)depthSpacePoints[idx].X;
			int y = (int)depthSpacePoints[idx].Y;

			int depthIndex = (y * depthWidth) + x;
			int colorIndex = idx * ColorBytesPerPixel;

			if (isInColorCircle(j, i) && isValidColorRange(x, y) && isValidDepthRange(depthIndex)) {
				if (depthOrcolor) {
					dataColor[colorIndex + 0] = colorBuffer[colorIndex + 0];
					dataColor[colorIndex + 1] = colorBuffer[colorIndex + 1];
					dataColor[colorIndex + 2] = colorBuffer[colorIndex + 2];
					dataColor[colorIndex + 3] = 255;
				} else {
					// ここで完全に二値可しても良い
					dataDepth[idx] = ~((depthBuffer[depthIndex] * 255) / maxDepthReliableDistance);
				}
			} else {
				if (depthOrcolor) {
					dataColor[colorIndex + 0] = 0;
					dataColor[colorIndex + 1] = 0;
					dataColor[colorIndex + 2] = 0;
					dataColor[colorIndex + 3] = 255;
				} else {
					dataDepth[idx] = 0;
				}
			}
		}
	}

	if (depthOrcolor) {
		colorImage.update();
	} else {
		// colorImage に対してOpenCVで矩形探索
		depthImage.update();
		dataDepth = depthImage.getPixels().getData();

		ofxCvContourFinder cvCountourFinder;
		ofxCvGrayscaleImage cvGrayScaleImage;
		cvGrayScaleImage.setFromPixels(dataDepth, depthImage.getWidth(), depthImage.getHeight());
		cvGrayScaleImage.threshold(detectThreshold);
		cvCountourFinder.findContours(cvGrayScaleImage, cvMinArea, cvMaxArea, cvMaxTrackRect, false);

		// 特定した矩形毎に重心特定
		for (int i = 0; i < cvCountourFinder.nBlobs; i++) {
			int x = 0, y = 0;
			for (int j = 0; j < cvCountourFinder.blobs[i].pts.size(); j++) {
				x += cvCountourFinder.blobs[i].pts[j].x;
				y += cvCountourFinder.blobs[i].pts[j].y;
			}
			x /= cvCountourFinder.blobs[i].pts.size();
			y /= cvCountourFinder.blobs[i].pts.size();

			trackRectCenters.push_back(ofPoint(x, y));
		}
	}

	// 重心データをOSCで送信(oscSendFrameCounterフレームカウンター毎に)
	if (trackRectCenters.size() > 0 && ofGetFrameNum() % oscSendFrameCounter == 0) {
		for (int i = 0; i < trackRectCenters.size(); i++) {
			// circlePointX, circlePointY を使って座標変換した方がよさげ
			// 重心座標(x, y)と、矩形の輪郭点のカメラからの距離平均(z=大体の高さ)を送信
			
			// 距離を特定(存在しない事はありえない)
			int idx = trackRectCenters[i].y * colorImage.getWidth() + trackRectCenters[i].x;
			int depthX = (int)depthSpacePoints[idx].X;
			int depthY = (int)depthSpacePoints[idx].Y;
			int depthIndex = (depthY * depthWidth) + depthX;
			int z = 0;
			if (isValidDepthRange(depthIndex)) {
				z = depthBuffer[depthIndex];
			}

			ofxOscMessage message;
			message.setAddress(OSC_ADDRESS);
			message.addIntArg(trackRectCenters[i].x);
			message.addIntArg(trackRectCenters[i].y);
			if (z >= 0)	message.addIntArg(z);

			// 送信ログ
			std::cout << "OSC Send Mesage" << trackRectCenters[i].x << ":" << trackRectCenters[i].y << ":" << z << std::endl;

			// データ送信
			// oscSender.sendMessage(message);
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255, 255, 255);

	// color or depth の表示
	if (depthOrcolor) {
		colorImage.draw(0, 0);
	} else {
		depthImage.draw(0, 0);
	}

	// メニューの表示
	if (showMenu) {
		ofSetColor(0, 0, 0);
		gui.draw();
	}

	// 重心の画面表示用
	if (trackRectCenters.size() > 0) {
		ofSetColor(255, 0, 0);
		ofFill();
		for (int i = 0; i < trackRectCenters.size(); i++) {
			ofDrawCircle(trackRectCenters[i], 2);
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
	case 'h':
		showMenu = !showMenu;
		break;
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}


bool ofApp::isValidDepthRange(int index)
{
	return (minDepth <= depthBuffer[index]) && (depthBuffer[index] <= baseDepth);
}

bool ofApp::isValidColorRange(int x, int y)
{
	return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
}

bool ofApp::isInColorCircle(int x, int y) {
	int wi = circlePointX - x;
	int yi = circlePointY - y;
	int reso = circleResolution;
	if (sqrt(wi * wi + yi * yi) <= circleResolution) {
		return true;
	} else {
		return false;
	}
}