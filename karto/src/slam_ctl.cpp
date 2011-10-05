/*
 * slam_ctl
 * Copyright (c) 2011, TU München
 *
 * Author: Robert Huitl
 */


#include "ros/ros.h"
#include "ros/console.h"

#include <karto/SaveMapperState.h>
#include <karto/LoadMapperState.h>

#include <string.h>

int
main(int argc, char** argv)
{
	if(argc != 3) {
		fprintf(stderr, "Usage: %s command filename\n  where command is \"load\" or \"save\".\n", argv[0]);
		return 1;
	}

	ros::init(argc, argv, "slam_ctl");
	ros::NodeHandle n;

	const char* cmd = argv[1];
	const char* fn  = argv[2];

	if(!strcmp(cmd, "load")) {
		const static std::string servname = "slam_karto/LoadMapperState";

		ROS_INFO("Loading mapper state using service %s...", n.resolveName(servname).c_str());

		karto::LoadMapperState::Request req;
		karto::LoadMapperState::Response resp;

		req.filename = fn;

		if(!ros::service::call(servname, req, resp)) {
			ROS_ERROR("Service call failed. Check output of slam_karto node.");
			return 1;
		}

		ROS_INFO("Service call succeeded, state has been loaded.");

	} else if(!strcmp(cmd, "save")) {
		const static std::string servname = "slam_karto/SaveMapperState";

		ROS_INFO("Saving mapper state using service %s...", n.resolveName(servname).c_str());

		karto::SaveMapperState::Request req;
		karto::SaveMapperState::Response resp;

		req.filename = fn;

		if(!ros::service::call(servname, req, resp)) {
			ROS_ERROR("Service call failed. Check output of slam_karto node.");
			return 1;
		}

		ROS_INFO("Service call succeeded, state has been saved.");
	}

	return 0;
}
