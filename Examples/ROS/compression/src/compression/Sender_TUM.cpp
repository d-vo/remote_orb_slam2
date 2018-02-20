/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*
*
* Copyright of ORB-SLAM2:
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
#include "boost/algorithm/string.hpp"
#include <boost/lexical_cast.hpp>
#include <mutex>
#include <thread>


#include <opencv2/features2d.hpp>
#include "ORBextractor.h"
#include "feature_coder.h"

#include "compression/msg_features.h"
#include "compression/msg_feedback.h"
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Char.h"
#include "std_msgs/Bool.h"

namespace po = boost::program_options;

// Tracking states
enum eTrackingState{
	SYSTEM_NOT_READY=-1,
	NO_IMAGES_YET=0,
	NOT_INITIALIZED=1,
	OK=2,
	LOST=3
};


std::mutex g_orb_mutex;
std::shared_ptr<ORB_SLAM2::ORBextractor> g_orb;
int g_tracking_state = NOT_INITIALIZED;


// ORB
int nFeatures;
float fScaleFactor;
int nLevels;
int fIniThFAST;
int fMinThFAST;
cv::Mat K0;
cv::Mat DistCoef0;
cv::Mat M1l,M2l;
float bf;

// Coding
int imgWidth = 640;
int imgHeight = 480;
int bufferSize = 1;
int nlevels = 8;
bool inter = true;


void callback(const compression::msg_feedback::ConstPtr &msg)
{
	const int tracking_state = (int) msg->state;

	// Update
	if( tracking_state != g_tracking_state )
	{
		if( tracking_state == NOT_INITIALIZED )
		{
			g_orb = std::shared_ptr<ORB_SLAM2::ORBextractor>(new ORB_SLAM2::ORBextractor(2*nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
			std::cout << "Tracking State: NOT_INITIALIZED" << std::endl;
		}

		else if( tracking_state == OK )
		{
			g_orb = std::shared_ptr<ORB_SLAM2::ORBextractor>(new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
			std::cout << "Tracking State: OK" << std::endl;
		}

		else if( tracking_state == LOST )
		{
			std::cout << "Tracking State: LOST" << std::endl;
		}


		g_tracking_state = tracking_state;
	}
}


void loadSettings(const std::string &settingsFile)
{
	// Load camera parameters from settings file
	cv::FileStorage fSettings(settingsFile, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];

	K0 = cv::Mat::eye(3,3,CV_32F);
	K0.at<float>(0,0) = fx;
	K0.at<float>(1,1) = fy;
	K0.at<float>(0,2) = cx;
	K0.at<float>(1,2) = cy;

	DistCoef0 = cv::Mat(4,1,CV_32F);
	DistCoef0.at<float>(0) = fSettings["Camera.k1"];
	DistCoef0.at<float>(1) = fSettings["Camera.k2"];
	DistCoef0.at<float>(2) = fSettings["Camera.p1"];
	DistCoef0.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if(k3!=0)
	{
		DistCoef0.resize(5);
		DistCoef0.at<float>(4) = k3;
	}

	bf = fSettings["Camera.bf"];

	cout << endl << "Camera Parameters: " << endl;
	cout << "- fx: " << fx << endl;
	cout << "- fy: " << fy << endl;
	cout << "- cx: " << cx << endl;
	cout << "- cy: " << cy << endl;
	cout << "- k1: " << DistCoef0.at<float>(0) << endl;
	cout << "- k2: " << DistCoef0.at<float>(1) << endl;
	if(DistCoef0.rows==5)
		cout << "- k3: " << DistCoef0.at<float>(4) << endl;
	cout << "- p1: " << DistCoef0.at<float>(2) << endl;
	cout << "- p2: " << DistCoef0.at<float>(3) << endl;


	// Load ORB parameters
	nFeatures =  fSettings["ORBextractor.nFeatures"];
	fScaleFactor = fSettings["ORBextractor.scaleFactor"];
	nLevels = fSettings["ORBextractor.nLevels"];
	fIniThFAST = fSettings["ORBextractor.iniThFAST"];
	fMinThFAST = fSettings["ORBextractor.minThFAST"];



	cout << endl  << "ORB Extractor Parameters: " << endl;
	cout << "- Number of Features: " << nFeatures << endl;
	cout << "- Scale Levels: " << nLevels << endl;
	cout << "- Scale Factor: " << fScaleFactor << endl;
	cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
	cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;



	cv::initUndistortRectifyMap(K0,DistCoef0,cv::noArray(),cv::noArray(),cv::Size(imgWidth,imgHeight),CV_32F,M1l,M2l);
}


int main(int argc, char **argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("voc,v", po::value<std::string>(), "Vocabulary path")
		("input,i", po::value<std::string>(), "Image path")
		("coding,c", po::value<std::string>(), "settings path")
		("bitrate,b", po::value<double>(), "bitrate in kbits")
		("settings,s", po::value<string>(), "load yaml file")
		("reference,r", po::value<int>(), "reference frames used")
		("output,o", po::value<std::string>(), "output path");


	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);


	std::string output_path = vm["output"].as<std::string>();
	if( !boost::filesystem::exists(output_path) )
		boost::filesystem::create_directories(output_path);



	std::string strSettingPath = vm["settings"].as<std::string>();
	loadSettings(strSettingPath);


	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();

	ORBVocabulary voc;
	std::cout << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);


	// Load statistics
	std::string settings_path = vm["coding"].as<std::string>();
	std::cout << "Loading statistics from " << settings_path << std::endl;
	CodingStats codingModel;
	codingModel.load(settings_path );

	// Load images
	std::string image_path = vm["input"].as<std::string>();
	std::cout << "Loading image list from " << image_path << std::endl;
	std::vector<double> vTimestamps;
	std::vector<std::string> vCam0;
	Utils::LoadImagesTUM(image_path, vCam0,vTimestamps);


	if( !vm["reference"].empty() )
		bufferSize = vm["reference"].as<int>();

	g_orb = std::shared_ptr<ORB_SLAM2::ORBextractor>(new ORB_SLAM2::ORBextractor(2*nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
	FeatureCoder encoder(voc, codingModel,imgWidth, imgHeight, nLevels, 32, bufferSize, inter);


	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher bitstream_pub = n.advertise<compression::msg_features>("/featComp/bitstream", 1000);
	ros::Publisher shutdown_pub = n.advertise<std_msgs::Bool>("/featComp/shutdown", 1000);

	// Feedback loop
	ros::Subscriber feedback_sub = n.subscribe<compression::msg_feedback>("/featComp/feedback", 1000, callback);


	ros::Rate poll_rate(100);
	while(bitstream_pub.getNumSubscribers() == 0)
		poll_rate.sleep();


	const size_t nImages = vCam0.size();
	for( size_t imgId = 0; imgId < nImages; imgId++ )
	{
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

		// Read left and right images from file
		cv::Mat imLeftDist = cv::imread(vCam0[imgId],cv::IMREAD_GRAYSCALE);

		cv::Mat imLeft;
		cv::remap(imLeftDist,imLeft,M1l,M2l,cv::INTER_LINEAR);


		// Extract features
		cv::Mat descriptors;
		std::vector<cv::KeyPoint> keypoints;
		{
			std::unique_lock<std::mutex> lock(g_orb_mutex);
			(*g_orb)(imLeft, cv::noArray(), keypoints, descriptors);
		}


		if( g_tracking_state == NOT_INITIALIZED )
		{
			encoder.setMaxN(nFeatures);
			encoder.setPriorForInit(true);
		}
		else
		{
			encoder.setPriorForInit(false);
			encoder.setMaxN(std::numeric_limits<int>::max());
		}


		std::vector<uchar> bitstream;
		encoder.encodeImage(keypoints, descriptors, bitstream);

		double tframe = vTimestamps[imgId];
		compression::msg_features msg;
		msg.header.stamp = ros::Time::now();
		msg.tframe = tframe; // in seconds
		msg.data.assign(bitstream.begin(),bitstream.end());
		bitstream_pub.publish(msg);
		ros::spinOnce();


		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();


        // Wait to load the next frame
        double T=0;
        if(imgId<nImages-1)
            T = vTimestamps[imgId+1]-tframe;
        else if(imgId>0)
            T = tframe-vTimestamps[imgId-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
	}

	std_msgs::Bool shutdown_msg;
	shutdown_pub.publish(shutdown_msg);
	ros::spinOnce();
}
