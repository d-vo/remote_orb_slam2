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


#include <opencv2/features2d/features2d.hpp>
#include "System.h"
#include "FrameInfo.h"

#include "feature_coder.h"
#include <chrono>

#include "ros/ros.h"
#include "compression/msg_features.h"
#include "compression/msg_feedback.h"
#include "std_msgs/Bool.h"

namespace po = boost::program_options;


int imgWidth = 640;
int imgHeight = 480;
int bufferSize = 1;
int nlevels = 8;
bool inter = true;




void quit_callback(const std_msgs::Bool::ConstPtr msg, ORB_SLAM2::System *SLAM, std::string &output_path)
{
	SLAM->Shutdown();

	// Save camera trajectory
	std::string traj_path = output_path + "/Trajectory.txt";
	SLAM->SaveTrajectoryTUM(traj_path);
	ros::shutdown();
}


void callback(const compression::msg_features::ConstPtr msg, ORB_SLAM2::System *SLAM, FeatureCoder *decoder, ros::Publisher *feedback_pub)
{
	// Save bitstream
	std::vector<uchar> img_bitstream(msg->data.begin(), msg->data.end());


	ORB_SLAM2::FrameInfo info;
	info.mnHeight = imgHeight;
	info.mnWidth = imgWidth;


	// Decoding
	std::vector<unsigned int> dec_visualWords;
	std::vector<cv::KeyPoint> dec_keypoints;
	cv::Mat dec_descriptors;
	decoder->decodeImage(img_bitstream, dec_keypoints, dec_descriptors, dec_visualWords);


	// Pass the images to the SLAM system
	const double tframe = msg->tframe;
	SLAM->TrackMonocularCompressed(info, dec_keypoints, dec_descriptors, dec_visualWords, tframe);


	// Feedback message
	const int tracking_state = SLAM->GetTrackingState();
	compression::msg_feedback feedback_msg;
	feedback_msg.state = tracking_state;
	feedback_msg.header.stamp = ros::Time::now();
	feedback_pub->publish(feedback_msg);
}


int main(int argc, char **argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()
						("help", "produce help message")
						("voc,v", po::value<std::string>(), "Vocabulary path")
						("settings,s", po::value<std::string>(), "ORB SLAM settings path")
						("coding,c", po::value<std::string>(), "coding model")
						("reference,r", po::value<int>(), "reference frames used")
						("output,o", po::value<std::string>(), "Results output");


	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);



	std::string output_path = vm["output"].as<std::string>();

	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();

	ORBVocabulary voc;
	std::cout << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);


	// Load statistics
	std::string stats_path = vm["coding"].as<std::string>();
	std::cout << "Loading statistics from " << stats_path << std::endl;
	CodingStats codingModel;
	codingModel.load(stats_path);


	// Setup ORB SLAM
	const string &strSettingsFile = vm["settings"].as<std::string>();
	ORB_SLAM2::System SLAM(voc_path,strSettingsFile,ORB_SLAM2::System::MONOCULAR,true);


	if( !vm["reference"].empty() )
		bufferSize = vm["reference"].as<int>();


	srand (time(NULL));
	int i = rand() % 1000 + 1;

	FeatureCoder decoder(voc, codingModel ,imgWidth, imgHeight, nlevels, 32, bufferSize, inter);

	std::string name = "listener" + std::to_string(i);
	ros::init(argc, argv, name.c_str());
	ros::NodeHandle n;

	// Feedback
	ros::Publisher feedback_pub = n.advertise<compression::msg_feedback>("/featComp/feedback", 1000);
	ros::Subscriber bitstream_sub = n.subscribe<compression::msg_features>("/featComp/bitstream", 1000, boost::bind(callback, _1, &SLAM, &decoder, &feedback_pub));
	ros::Subscriber quit_signal_sub = n.subscribe<std_msgs::Bool>("/featComp/shutdown", 1000, boost::bind(quit_callback, _1, &SLAM, output_path));

	ros::Rate poll_rate(100);
	while(feedback_pub.getNumSubscribers() == 0)
		poll_rate.sleep();

	ros::spin();
}
