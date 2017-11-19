/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <iostream>

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

cs::VideoProperty exposureProp;
int exposure = 100;
int desiredExposure = 100;
bool autoExposureState = true;

cs::VideoProperty manualControlProp;
bool manualControl = false;

cs::VideoProperty autoModeProp;
bool autoMode = false;

cs::VideoProperty autoStageProp;
int autoStage = 4;

cs::VideoProperty saveToFileProp;
bool saveToFile = false;

cs::UsbCamera camera{"usbcam", 0};

void draw_overlay(cv::Mat& inImg) 
{
		
	// draw gear post guidelines
	//cv::line(inImg,gearLineUpperCenter,gearLineUpperBottom,cv::Scalar(0,255,255), 1, 8);
	//cv::line(inImg,gearLineLowerCenter,gearLineLowerBottom,cv::Scalar(0,255,255), 1, 8);
	
	// draw boxes for boiler
	cv::rectangle(inImg, highUL, highLR, cv::Scalar(255, 0, 0), 2, 8, 0);					
	cv::rectangle(inImg,middleUL,middleLR, cv::Scalar(0, 255, 0), 2, 8, 0);					
	cv::rectangle(inImg,lowUL, lowLR, cv::Scalar(0, 0, 255), 2, 8, 0);					
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
		fscanf(parameter_file,"exposure = %d\n",&exposure);
		desiredExposure = exposure;
		
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
	printf("exposure = %d\n",exposure);
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
		fprintf(parameter_file,"exposure = %d\n",exposure);
	}
	fclose(parameter_file);
	printf("File write complete.\n");	
}

void create_web_properties(cs::CvSource cvsource)
{
	// add properties controllable by webpage
	manualControlProp = cvsource.CreateBooleanProperty("Manual_Ctrl",false,false);
	autoModeProp = cvsource.CreateBooleanProperty("Auto_Mode",false,false);
	autoStageProp = cvsource.CreateProperty("Auto_Stage",cs::VideoProperty::Kind::kInteger,1,4,1,autoStage,autoStage);
	minHueProp = cvsource.CreateProperty("Min_Hue",cs::VideoProperty::Kind::kInteger,0,255,1,minHue,minHue);
	maxHueProp = cvsource.CreateProperty("Max_Hue",cs::VideoProperty::Kind::kInteger,0,255,1,maxHue,maxHue);
	minSatProp = cvsource.CreateProperty("Min_Sat",cs::VideoProperty::Kind::kInteger,0,255,1,minSat,minSat);
	maxSatProp = cvsource.CreateProperty("Max_Sat",cs::VideoProperty::Kind::kInteger,0,255,1,maxSat,maxSat);
	minValProp = cvsource.CreateProperty("Min_Value",cs::VideoProperty::Kind::kInteger,0,255,1,minVal,minVal);
	maxValProp = cvsource.CreateProperty("Max_Value",cs::VideoProperty::Kind::kInteger,0,255,1,maxVal,maxVal);
	dilationProp = cvsource.CreateProperty("Dilation",cs::VideoProperty::Kind::kInteger,1,15,1,dilationFactor,dilationFactor);
	exposureProp = cvsource.CreateProperty("Exposure",cs::VideoProperty::Kind::kInteger,0,300,1,exposure,exposure);
	minAreaProp = cvsource.CreateProperty("Min_Area",cs::VideoProperty::Kind::kInteger,1,1000,1,minArea,minArea);
	//maxAreaProp = cvsource.CreateProperty("Max_Area",cs::VideoProperty::Kind::kInteger,1,2000,1,maxArea,maxArea);
	saveToFileProp = cvsource.CreateBooleanProperty("Save_to_File",false,false);

}

void get_values()
{
	manualControl = manualControlProp.Get();
	autoMode = autoModeProp.Get();
	autoStage = autoStageProp.Get();
	minHue = minHueProp.Get();
	maxHue = maxHueProp.Get();
	minSat = minSatProp.Get();
	maxSat = maxSatProp.Get();
	minVal = minValProp.Get();
	maxVal = maxValProp.Get();
	dilationFactor = dilationProp.Get();
	desiredExposure = exposureProp.Get();
	minArea = minAreaProp.Get();
	//maxArea = maxAreaProp.Get();
	saveToFile = saveToFileProp.Get();
}

bool checkAutoState()
{
	if (manualControl) 
	{
		return autoMode;
	}
	else
	{
		// check networktable auto state
		return table->GetEntry("autoCam").GetBoolean(false);
	}
}

void autoExposureOn()
{
	printf("Auto Exposure On.\n");	
	camera.SetExposureAuto();
}

void autoExposureOff()
{
	printf("Auto Exposure Off.\n");	
	camera.SetExposureManual(exposure);
}

void set_exposure(int exposure) 
{
	printf("Exposure level set to %d.\n",exposure);	
	camera.SetExposureManual(exposure);
}

int main() {
	
  // read in parameter file
  read_param_file();
	
  camera.SetVideoMode(cs::VideoMode::kMJPEG, frameWidth, frameHeight, 30);
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, frameWidth, frameHeight, 30};
  cs::MjpegServer cvMjpegServer{"cvhttpserver", 8082};
  cvMjpegServer.SetSource(cvsource);

  // set up web controls
  create_web_properties(cvsource);

  // camera (input) image
  cv::Mat inputImg;
  
  // images used for auto
  cv::Mat hsvImg, binaryImg, dilationImg, contourImg; 
  
  // images for teleop
  cv::Mat overlayImg;

  // reset autoExposure to true 
  autoExposureOn();
  autoExposureState = true;
 
	// initialize network table for comm with the robot
	auto tableInstance = nt::NetworkTableInstance::GetDefault();
	//tableInstance.SetServer("Roborio-1778-frc.local");  // send data to roborio
	tableInstance.StartServer();     // debug only - if roborio not present
	table = tableInstance.GetTable("RPIComm/Data_Table");

  // enter forever streaming loop
  int frameCtr = 0;
  for (;;) {
    uint64_t time = cvsink.GrabFrame(inputImg);
    if (time == 0) {
      std::cout << "error: " << cvsink.GetError() << std::endl;
      continue;
    }
    frameCtr++;
    
    if ((frameCtr % 30) == 0)
    {
		std::cout << "got " << frameCtr << " frames at time " << time << " size " << inputImg.size()
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
	  
	
    if (checkAutoState())   // autonomous state check
    {
		// set autoExposure to false (if not already done)
		if (autoExposureState == true)
		{
			autoExposureOff();
			autoExposureState = false;
		}
		
		// check to see if desired exposure changed
		if (exposure != desiredExposure)
		{
			exposure = desiredExposure;
			set_exposure(exposure);
		}
		
		cv::cvtColor( inputImg, hsvImg, CV_BGR2HSV );
		cv::inRange(hsvImg, cv::Scalar(minHue, minSat, minVal), cv::Scalar(maxHue, maxSat, maxVal), binaryImg);		/*green*/
		if (autoStage == 1)
			cvsource.PutFrame(binaryImg);

		// dilate image (unify pieces)
		int dil = dilationFactor;
		int dil2 = dilationFactor*2 + 1;
		cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dil2,dil2), cv::Point(dil,dil));
		dilate(binaryImg, dilationImg, dilateElement);
		if (autoStage == 2)
			cvsource.PutFrame(dilationImg);

		// find contours from dilated image, place in list (vector)
		std::vector<cv::Vec4i> hierarchy;
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(dilationImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));

		std::vector<std::vector<cv::Point>> hulls (contours.size());	
		for (int i=0; i< contours.size(); i++)
		{
			cv::convexHull(cv::Mat(contours[i]), hulls[i], false);
		}

		// create stats for each convex hull	
		std::vector<cv::Moments>mu(hulls.size());	  // hull moments
		std::vector<cv::Point2f>mc(hulls.size());       // hull mass centers
		std::vector<double>targetArea(hulls.size());   // hull areas

		for (int i=0; i<hulls.size(); i++)
		{
			mu[i] = cv::moments(hulls[i], false);   // find moments
			mc[i] = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
			targetArea[i] = cv::contourArea(hulls[i]);
		}

		int maxTargetArea = -1;
		int targetIndex = -1;
		bool targetDetected = false;
		for (int i=0; i<hulls.size(); i++)
		{
			// see if this target meets the minimum area requirement
			if (targetArea[i] > minArea)
			{
				targetDetected = true;

				// see if this target is the biggest so far
				if (targetArea[i] > maxTargetArea)
				{
					targetIndex = i;
					maxTargetArea = targetArea[i];
				}
			}
		}

		// create contour image
		contourImg = cv::Mat::zeros(binaryImg.size(),CV_8UC3);
		for (int i=0; i< contours.size(); i++)
		{
			cv::Scalar colorGreen = cv::Scalar(0, 0, 255);  // green
			cv::Scalar colorWhite = cv::Scalar(255, 255, 255);  // white
			cv::drawContours(contourImg, hulls, i, colorWhite, 2, 8, hierarchy, 0, cv::Point());
		}
		if (autoStage == 3)
			cvsource.PutFrame(contourImg);

		// if target meets criteria, do stuff
		if (targetDetected)
		{		
			// draw the target on one of the images
			cv::Scalar colorWhite = cv::Scalar(255, 255, 255);  // white
			cv::Scalar colorGreen = cv::Scalar(0, 255, 0);  // green
			cv::Scalar colorBlue = cv::Scalar(255, 0, 0);  // blue
			cv::drawContours(inputImg, hulls, targetIndex, colorGreen, 2, 8, hierarchy, 0, cv::Point());
			cv::circle(inputImg, mc[targetIndex], 3 ,colorBlue,2,6,0);

			// write target info to network table
			table->GetEntry("targets").SetBoolean(true);
			//table->GetEntry("targetX").SetDouble(mc[targetIndex].x - imageCenterX);
			//table->GetEntry("targetY").SetDouble(mc[targetIndex].y - imageCenterY);
			table->GetEntry("targetX").SetDouble(mc[targetIndex].x);
			table->GetEntry("targetY").SetDouble(mc[targetIndex].y);
			table->GetEntry("targetArea").SetDouble(targetArea[targetIndex]);
			table->GetEntry("frameWidth").SetDouble((double)frameWidth);
			table->GetEntry("frameHeight").SetDouble((double)frameHeight);

			//printf("Target area %3.0f detected at (%3.0f,%3.0f)\n",
			//	targetArea[targetIndex], mc[targetIndex].x - imageCenterX,
			//			     mc[targetIndex].y - imageCenterY);
		}
		else
		{
			// write to network table that no target is detected
			table->GetEntry("targets").SetBoolean(false);
		}
		
		if (autoStage == 4)
			cvsource.PutFrame(inputImg);

	}
	else   // teleop
	{
			// reset target field for teleop
			table->GetEntry("targets").SetBoolean(false);
		
		// set autoExposure to true (if not already done)
		if (autoExposureState == false)
		{
			autoExposureOn();
			autoExposureState = true;
		}

		overlayImg = inputImg;
		draw_overlay(overlayImg);
		cvsource.PutFrame(overlayImg);
	}
  }
}
