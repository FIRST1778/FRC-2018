/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <iostream>
#include <string>

#include <ctime>
#include <unistd.h>
#include <sys/time.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "cscore.h"

// image properties
const int frameWidth = 320;
const int frameHeight = 240;
const int maxFps = 20;

// Guidelines
const cv::Point leftGuidelineUL(130, 160);
const cv::Point leftGuidelineLR(40, 220);
const cv::Point rightGuidelineUL(190, 160);
const cv::Point rightGuidelineLR(280, 220);

// Arrow Shapes
const cv::Point arrowBodyUL(160, 170);
const cv::Point arrowBodyLR(160, 220);
cv::Point upArrowHead[1][3] = { cv::Point(160,160), cv::Point(155,170), cv::Point(165,170)};
const cv::Point* upPpt[1] = { upArrowHead[0] };
cv::Point dnArrowHead[1][3] = { cv::Point(160,230), cv::Point(155,220), cv::Point(165,220)};
const cv::Point* dnPpt[1] = { dnArrowHead[0] };
int npt[1] = { 3 };
	
// Target 1
const cv::Point highCenter(160,80);
const cv::Point highSize(80,30);
const cv::Point highUL(highCenter.x - (highSize.x/2), highCenter.y - (highSize.y/2));
const cv::Point highLR(highCenter.x + (highSize.x/2), highCenter.y + (highSize.y/2));

// Target 2
const cv::Point middleCenter(160,120);
const cv::Point middleSize(80,30);
const cv::Point middleUL(middleCenter.x - (middleSize.x/2), middleCenter.y - (middleSize.y/2));
const cv::Point middleLR(middleCenter.x + (middleSize.x/2), middleCenter.y + (middleSize.y/2));

// Target 3
const cv::Point lowCenter(160,160);
const cv::Point lowSize(80,30);
const cv::Point lowUL(middleCenter.x - (lowSize.x/2), lowCenter.y - (lowSize.y/2));
const cv::Point lowLR(middleCenter.x + (lowSize.x/2), lowCenter.y + (lowSize.y/2));

// networktable properties
char roborio_ipaddr[32] = "10.17.78.52";
std::shared_ptr<nt::NetworkTable> table;

// image filtering properties
cs::VideoProperty minHueProp, maxHueProp;
cs::VideoProperty minSatProp, maxSatProp;
cs::VideoProperty minValProp, maxValProp;
int minHue = 0;
int maxHue = 255;
int minSat = 0;
int maxSat = 255;
int minVal = 0;
int maxVal = 255;

cs::VideoProperty minAreaProp, maxAreaProp;
int minArea = 10;
int maxArea = 3000;

cs::VideoProperty dilationProp;
int dilationFactor = 3;

cs::VideoProperty autonomousExposureProp;
int autonomousExposure = 100;
int desiredAutonomousExposure = 100;

cs::VideoProperty teleopExposureProp;
int teleopExposure = 100;
int desiredTeleopExposure = 100;

bool teleopExposureState = true;

cs::VideoProperty manualControlProp;
bool manualControl = false;

cs::VideoProperty autoModeProp;
bool autoMode = false;

cs::VideoProperty autoStageProp;
int autoStage = 4;

cs::VideoProperty saveToFileProp;
bool saveToFile = false;

cs::UsbCamera camera{"usbcam", 0};

// FPS vars
double fps;
timeval startTimer, endTimer;

// draw overlay graphics on image
void draw_overlay(cv::Mat& inImg) 
{	
	// draw gear post guidelines
	cv::line(inImg,leftGuidelineUL,leftGuidelineLR,cv::Scalar(0,255,255), 1, 8);
	cv::line(inImg,rightGuidelineUL,rightGuidelineLR,cv::Scalar(0,255,255), 1, 8);
	
	// draw boxes for boiler
	//cv::rectangle(inImg, highUL, highLR, cv::Scalar(255, 0, 0), 2, 8, 0);					
	//cv::rectangle(inImg,middleUL,middleLR, cv::Scalar(0, 255, 0), 2, 8, 0);					
	//cv::rectangle(inImg,lowUL, lowLR, cv::Scalar(0, 0, 255), 2, 8, 0);					

	// draw collector arrow (if non-zero)
	double collectorStrength = table->GetEntry("collectorStrength").GetDouble(0.0);
	
	if (collectorStrength > 0) {
		cv::fillPoly(inImg,upPpt,npt,1,cv::Scalar(0,0,255));
		cv::line(inImg,arrowBodyUL,arrowBodyLR,cv::Scalar(0,0,255), 2, 8);
	}
	else if (collectorStrength < 0) {
		cv::fillPoly(inImg,dnPpt,npt,1,cv::Scalar(0,255,0));
		cv::line(inImg,arrowBodyUL,arrowBodyLR,cv::Scalar(0,255,0), 2, 8);		
	}
}

// draw Frame Per sec counter and Moniker on screen
void overlay_fps(cv::Mat& inImg)
{
	char str[64];
	
	sprintf(str,"Chill Out! 1778 EYE-cicle Vision");
	cv::putText(inImg,str,cv::Point2f(5,10),cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255,255,0,255), 1, 8);

	sprintf(str,"%3.1f FPS",fps);
	cv::putText(inImg,str,cv::Point2f(5,frameHeight-5),cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0,255,255,255), 1, 8);

	bool clampOn = table->GetEntry("clampOn").GetBoolean(true);
	
	if (clampOn) {
		sprintf(str,"CLAMP ON");
		cv::putText(inImg,str,cv::Point2f(20,160),cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0,255,255,255), 2, 8);
	}

	bool brakeOn = table->GetEntry("brakeOn").GetBoolean(true);
	
	if (brakeOn) {
		sprintf(str,"BRAKE ON");
		cv::putText(inImg,str,cv::Point2f(210,160),cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0,255,255,255), 2, 8);
	}

}

void read_param_file()
{
   // read in parameters (if file exists)
   FILE *parameter_file = fopen("/home/pi/chillout/chillOut_params_2018.txt","r");
	if (parameter_file != NULL) {
		printf("Reading parameters from file\n");
		fscanf(parameter_file,"roborio_ipaddr = %s\n",roborio_ipaddr);
		fscanf(parameter_file,"minArea = %d\n",&minArea);
		fscanf(parameter_file,"maxArea = %d\n",&maxArea);
		fscanf(parameter_file,"minHue = %d\n",&minHue);
		fscanf(parameter_file,"maxHue = %d\n",&maxHue);
		fscanf(parameter_file,"minSat = %d\n",&minSat);
		fscanf(parameter_file,"maxSat = %d\n",&maxSat);
		fscanf(parameter_file,"minVal = %d\n",&minVal);
		fscanf(parameter_file,"maxVal = %d\n",&maxVal);
		fscanf(parameter_file,"dilationFactor = %d\n",&dilationFactor);
		fscanf(parameter_file,"autonomousExposure = %d\n",&autonomousExposure);
		fscanf(parameter_file,"teleopExposure = %d\n",&teleopExposure);
		desiredAutonomousExposure = autonomousExposure;
		desiredTeleopExposure = teleopExposure;
	}
	fclose(parameter_file);
	printf("File read complete.\n");
	
	printf("roborio_ipaddr = %s\n",roborio_ipaddr);
	printf("minArea = %d\n",minArea);
	printf("maxArea = %d\n",maxArea);
	printf("minHue = %d\n",minHue);
	printf("maxHue = %d\n",maxHue);
	printf("minSat = %d\n",minSat);
	printf("maxSat = %d\n",maxSat);
	printf("minVal = %d\n",minVal);
	printf("maxVal = %d\n",maxVal);
	printf("dilationFactor = %d\n",dilationFactor);
	printf("autonomousExposure = %d\n",autonomousExposure);
	printf("teleopExposure = %d\n",teleopExposure);
}

void write_param_file()
{
   // write out parameters
   FILE *parameter_file = fopen("/home/pi/chillout/chillOut_params_2018.txt","w");
	if (parameter_file != NULL) {
		printf("Writing parameters to file\n");
		fprintf(parameter_file,"roborio_ipaddr = %s\n",roborio_ipaddr);
		fprintf(parameter_file,"minArea = %d\n",minArea);
		fprintf(parameter_file,"maxArea = %d\n",maxArea);
		fprintf(parameter_file,"minHue = %d\n",minHue);
		fprintf(parameter_file,"maxHue = %d\n",maxHue);
		fprintf(parameter_file,"minSat = %d\n",minSat);
		fprintf(parameter_file,"maxSat = %d\n",maxSat);
		fprintf(parameter_file,"minVal = %d\n",minVal);
		fprintf(parameter_file,"maxVal = %d\n",maxVal);
		fprintf(parameter_file,"dilationFactor = %d\n",dilationFactor);
		fprintf(parameter_file,"autonomousExposure = %d\n",autonomousExposure);
		fprintf(parameter_file,"teleopExposure = %d\n",teleopExposure);
	}
	fclose(parameter_file);
	printf("File write complete.\n");	
}

// create and link web controls for monitoring
void create_web_properties(cs::CvSource& cvsource)
{
	// add properties controllable by webpage
	manualControlProp = cvsource.CreateBooleanProperty("Manual_Ctrl",false,false);
	teleopExposureProp = cvsource.CreateProperty("Exposure_Teleop",cs::VideoProperty::Kind::kInteger,0,150,1,teleopExposure,teleopExposure);
	saveToFileProp = cvsource.CreateBooleanProperty("Save_to_File",false,false);
}

// retrieves values from the webpage
void get_values()
{
	manualControl = manualControlProp.Get();
	desiredTeleopExposure = teleopExposureProp.Get();
	saveToFile = saveToFileProp.Get();
}


void set_exposure(int exposure) 
{
	printf("Exposure level set to %d.\n",exposure);	
	camera.SetExposureManual(exposure);
}


// teleop image processing function
void processTeleop(cv::Mat& inputImg, cs::CvSource& cvsource)
{
	// interim stage images for teleop
	cv::Mat overlayImg;

	// set teleopExposure to true (if not already done)
	if (teleopExposureState == false)
	{
		set_exposure(teleopExposure);
		teleopExposureState = true;
	}
	
	// check to see if desired exposure changed
	if (teleopExposure != desiredTeleopExposure)
	{
		teleopExposure = desiredTeleopExposure;
		set_exposure(teleopExposure);
	}

	overlayImg = inputImg;
	
	draw_overlay(overlayImg);
	overlay_fps(overlayImg);
	
	cvsource.PutFrame(overlayImg);
}

double calculateFps(int frameCount)
{
	gettimeofday(&endTimer,0);
	
	long sec = (endTimer.tv_sec - startTimer.tv_sec);
	long u_sec = (endTimer.tv_usec - startTimer.tv_usec);
	
	double elapsedSec = (double) sec + (double)u_sec/1e6;
	std::cout << "frameCount = " << frameCount << " elapsedSec = " << elapsedSec << std::endl;
	double fps = (double) frameCount / elapsedSec;
	
	startTimer = endTimer;
	
	return fps;
}

int main() {
	
  // read first timeval
  gettimeofday(&startTimer,0);
  
  // read in parameter file
  read_param_file();
	
  camera.SetVideoMode(cs::VideoMode::kMJPEG, frameWidth, frameHeight, maxFps);
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, frameWidth, frameHeight, maxFps};
  //cs::MjpegServer cvMjpegServer{"cvhttpserver", 8082};
  cs::MjpegServer cvMjpegServer{"cvhttpserver", 1181};
  cvMjpegServer.SetSource(cvsource);

  // set up web controls
  create_web_properties(cvsource);

  // camera (input) image
  cv::Mat inputImg;
   
	// initialize network table for comm with the robot
	auto tableInstance = nt::NetworkTableInstance::GetDefault();
	
	tableInstance.SetServer("Roborio-1778-frc.local");  // send data to roborio
	//tableInstance.StartServer();          // DEBUG ONLY - use only if no roborio present
	
	// set location of the camera stream in the network table (will be picked up by DS)
	// Not usually set by Pi - usually set directly by Roborio
	//std::string streamNames[1];
	//streamNames[0] = "mjpeg:http://10.0.0.179:1181/?action=stream";	
	//tableInstance.GetEntry("/CameraPublisher/ChillOutPiCam/streams").SetStringArray(streamNames);
	
	//Initial state: set networktable clampOn state to TRUE and reset exposure to teleop level 
	table = tableInstance.GetTable("RPIComm/Data_Table");
	table->GetEntry("collectorStrength").SetDouble(0.0);
	table->GetEntry("clampOn").SetBoolean(true);
	table->GetEntry("brakeOn").SetBoolean(true);
	set_exposure(teleopExposure);
	teleopExposureState = true;
	
  // enter forever streaming loop
  int startFrameCtr = 0;
  int frameCtr = 0;
  for (;;) {
    uint64_t time = cvsink.GrabFrame(inputImg);
    if (time == 0) {
      std::cout << "error: " << cvsink.GetError() << std::endl;
      continue;
    }
    frameCtr++;
    
    if ((frameCtr % 15) == 0)
    {
		fps = calculateFps(frameCtr-startFrameCtr);
		startFrameCtr = frameCtr;
		
		std::cout << "got " << frameCtr << " frames at time: " << time << " fps: " << fps << " size: " << inputImg.size()
				<< std::endl;
				
	    // if commanded, save current params to file
        if (saveToFile)
        {
			write_param_file();
			
			// reset saveToFile and web control
			saveToFile = false;
			saveToFileProp.Set(false);
		}
	}
	
	// get webpage control values
	get_values();                 
	 
	 // process image
	processTeleop(inputImg, cvsource);		
  }
}
