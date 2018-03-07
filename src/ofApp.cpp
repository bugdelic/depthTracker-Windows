#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofBackground(0);
	ofSetFrameRate(30);
	ofSetBackgroundAuto(true);

	// �f�t�H���g��Kinect���擾����
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));

	// Kinect���J��
	ERROR_CHECK(kinect->Open());

	BOOLEAN isOpen = false;
	ERROR_CHECK(kinect->get_IsOpen(&isOpen));
	if (!isOpen) {
		throw std::runtime_error("Kinect���J���܂���");
	}

	// �J���[���[�_�[���擾����
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	// �J���[�摜�̃T�C�Y���擾����
	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->get_FrameDescription(&colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));

	// Depth���[�_�[���擾����
	ComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
	ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

	// Depth�摜�̃T�C�Y���擾����
	ComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
	ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
	ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));

	depthPointX = depthWidth / 2;
	depthPointY = depthHeight / 2;

	// Depth�̍ő�l�A�ŏ��l���擾����
	ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minDepthReliableDistance));
	ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxDepthReliableDistance));

	std::cout << "Depth�f�[�^�̕�   : " << depthWidth << std::endl;
	std::cout << "Depth�f�[�^�̍��� : " << depthHeight << std::endl;

	std::cout << "Depth�ŏ��l       : " << minDepthReliableDistance << std::endl;
	std::cout << "Depth�ő�l       : " << maxDepthReliableDistance << std::endl;

	// �o�b�t�@�[���쐬����
	colorBuffer.resize(colorWidth * colorHeight * ColorBytesPerPixel);
	depthBuffer.resize(depthWidth * depthHeight);

	// GUI���쐬
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

	// OSC�p
	oscSender.setup(SERVER_IP_ADDR, SERVER_PORT);
}

//--------------------------------------------------------------
void ofApp::update(){

	ComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (ret == S_OK) {
		// BGRA�̌`���Ńf�[�^���擾����
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

	// �d�S�f�[�^�̃N���A
	trackRectCenters.clear();

	// �e�탁�����m��
	if (!colorImage.isAllocated())
		colorImage.allocate(colorWidth, colorHeight, OF_IMAGE_COLOR_ALPHA);

	if (!depthImage.isAllocated())
		depthImage.allocate(colorWidth, colorHeight, OF_IMAGE_GRAYSCALE);

	// �A�N�Z�X�p�|�C���^�擾
	unsigned char *dataColor = colorImage.getPixels().getData();
	unsigned char *dataDepth = depthImage.getPixels().getData();

	// depth/color�f�[�^�擾
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
					// �����Ŋ��S�ɓ�l���Ă��ǂ�
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
		// colorImage �ɑ΂���OpenCV�ŋ�`�T��
		depthImage.update();
		dataDepth = depthImage.getPixels().getData();

		ofxCvContourFinder cvCountourFinder;
		ofxCvGrayscaleImage cvGrayScaleImage;
		cvGrayScaleImage.setFromPixels(dataDepth, depthImage.getWidth(), depthImage.getHeight());
		cvGrayScaleImage.threshold(detectThreshold);
		cvCountourFinder.findContours(cvGrayScaleImage, cvMinArea, cvMaxArea, cvMaxTrackRect, false);

		// ���肵����`���ɏd�S����
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

	// �d�S�f�[�^��OSC�ő��M(oscSendFrameCounter�t���[���J�E���^�[����)
	if (trackRectCenters.size() > 0 && ofGetFrameNum() % oscSendFrameCounter == 0) {
		for (int i = 0; i < trackRectCenters.size(); i++) {
			// circlePointX, circlePointY ���g���č��W�ϊ����������悳��
			// �d�S���W(x, y)�ƁA��`�̗֊s�_�̃J��������̋�������(z=��̂̍���)�𑗐M
			
			// ���������(���݂��Ȃ����͂��肦�Ȃ�)
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

			// ���M���O
			std::cout << "OSC Send Mesage" << trackRectCenters[i].x << ":" << trackRectCenters[i].y << ":" << z << std::endl;

			// �f�[�^���M
			// oscSender.sendMessage(message);
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255, 255, 255);

	// color or depth �̕\��
	if (depthOrcolor) {
		colorImage.draw(0, 0);
	} else {
		depthImage.draw(0, 0);
	}

	// ���j���[�̕\��
	if (showMenu) {
		ofSetColor(0, 0, 0);
		gui.draw();
	}

	// �d�S�̉�ʕ\���p
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