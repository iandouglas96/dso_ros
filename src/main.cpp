/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"

using namespace dso;

FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);


	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
	undistImg->timestamp=img->header.stamp.toSec(); // relay the timestamp to dso
	fullSystem->addActiveFrame(undistImg, frameID);
	frameID++;
	delete undistImg;

}





int main(int argc, char** argv)
{
	ros::init(argc, argv, "dso_live");
  ros::NodeHandle nh("~");

	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations = 4;
	setting_minOptIterations = 1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;

  // Load parameters from ROS
  std::string calib = "";
  bool showDisp = false;
  nh.getParam("calib", calib);
  nh.getParam("display", showDisp);

	ROS_INFO_STREAM("MODE WITHOUT CALIBRATION, without exposure times!\n");
	setting_photometricCalibration = 0;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;

  undistorter = Undistort::getUndistorterForFile(calib, "", "");

  setGlobalCalib(
          (int)undistorter->getSize()[0],
          (int)undistorter->getSize()[1],
          undistorter->getK().cast<float>());


  fullSystem = new FullSystem();
  fullSystem->linearizeOperation=false;


  if(showDisp)
	  fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	  		 (int)undistorter->getSize()[0],
	  		 (int)undistorter->getSize()[1]));


  //fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


  if(undistorter->photometricUndist != 0)
  	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

  ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);

  ros::spin();
  for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
  {
      ow->join();
      delete ow;
  }

  delete undistorter;
  delete fullSystem;

	return 0;
}

