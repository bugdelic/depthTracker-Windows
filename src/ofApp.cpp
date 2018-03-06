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


}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255, 255, 255);

	ComPtr<ICoordinateMapper> mapper;
	ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

	std::vector<DepthSpacePoint> depthSpacePoints(colorWidth * colorHeight);
	ERROR_CHECK(mapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0],
		depthSpacePoints.size(), &depthSpacePoints[0]));

	// �J���[�f�[�^��\������
	ofImage colorImage;
	colorImage.allocate(colorWidth, colorHeight, OF_IMAGE_COLOR_ALPHA);
	unsigned char *data = colorImage.getPixels().getData();

	for (int j = 0; j < colorImage.getWidth(); ++j) {
		for (int i = 0; i < colorImage.getHeight(); ++i) {
			int idx = i * colorImage.getWidth() + j;
			int x = (int)depthSpacePoints[idx].X;
			int y = (int)depthSpacePoints[idx].Y;

			int depthIndex = (y * depthWidth) + x;
			int colorIndex = idx * ColorBytesPerPixel;

			if (isInColorCircle(j, i) && isValidColorRange(x, y) && isValidDepthRange(depthIndex)) {
				if (depthOrcolor) {
					data[colorIndex + 0] = colorBuffer[colorIndex + 0];
					data[colorIndex + 1] = colorBuffer[colorIndex + 1];
					data[colorIndex + 2] = colorBuffer[colorIndex + 2];
					data[colorIndex + 3] = 255;
				}
				else {
					data[colorIndex + 0] = ~((depthBuffer[depthIndex] * 255) / maxDepthReliableDistance);
					data[colorIndex + 1] = ~((depthBuffer[depthIndex] * 255) / maxDepthReliableDistance);
					data[colorIndex + 2] = ~((depthBuffer[depthIndex] * 255) / maxDepthReliableDistance);
					data[colorIndex + 3] = 255;
				}
			}
			else {
				data[colorIndex + 0] = 0;
				data[colorIndex + 1] = 0;
				data[colorIndex + 2] = 0;
				data[colorIndex + 3] = 255;
			}
		}
	}

	colorImage.update();
	colorImage.draw(0, 0);

	// colorImage �ɑ΂���OpenCV�ŋ�`�T��

	// ���肵����`���ɏd�S�̃f�[�^�����

	// �d�S�f�[�^��OSC�ő��M

	/*
	// depth�p
	ofImage image;
	image.allocate(depthWidth, depthHeight, OF_IMAGE_GRAYSCALE);

	unsigned char *data = image.getPixels().getData();
	for (int i = 0; i < depthHeight * depthWidth; ++i) {
		data[i] = ~((depthBuffer[i] * 255) / 8000);
	}
	image.update();
	image.draw(0, 0);
	*/

	ofNoFill();
	ofSetColor(255, 255, 255);
	ofDrawCircle(ofPoint((float)circlePointX, (float)circlePointY), circleResolution);

	if (showMenu) {
		ofSetColor(0, 0, 0);
		gui.draw();
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