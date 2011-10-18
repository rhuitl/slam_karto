/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey, Robert Huitl */

/**

@mainpage karto_gmapping

@htmlinclude manifest.html

*/

#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"

#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

#include <dynamic_reconfigure/server.h>
#include <karto/KartoConfig.h>
#include <karto/SaveMapperState.h>
#include <karto/LoadMapperState.h>

#include <Karto/Karto.h>

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
  public:
    SlamKarto();
    ~SlamKarto();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);

  private:
    bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t);
    bool addLaserToKarto(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool addScan(const sensor_msgs::LaserScan::ConstPtr& scan,
                 karto::Pose2& karto_pose);
    bool updateMap();
    void publishTransform();
    void publishLoop(double transform_publish_period);
    void publishGraphVisualization();
    void publishParticlesVisualization();
    void reconfigurationCallback(karto::KartoConfig &config, uint32_t level);

    bool saveMapperState(karto::SaveMapperState::Request& request,
                         karto::SaveMapperState::Response& response);
    bool loadMapperState(karto::LoadMapperState::Request& request,
                         karto::LoadMapperState::Response& response);
    // need a type to use advertiseService() with boost::bind()
    //   see https://code.ros.org/trac/ros/ticket/1245
    typedef boost::function<bool(karto::SaveMapperState::Request&, karto::SaveMapperState::Response&)> SaveMapperStateCallback;
    typedef boost::function<bool(karto::LoadMapperState::Request&, karto::LoadMapperState::Response&)> LoadMapperStateCallback;

    void setLocalizerConfig();


    // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster tfB_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher marker_publisher_;
    ros::Publisher particles_publisher_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    dynamic_reconfigure::Server<karto::KartoConfig> server_;
    karto::KartoConfig config_;
    ros::ServiceServer saveMapperStateService_;
    ros::ServiceServer loadMapperStateService_;

    // The map that will be published / sent to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    ros::Duration map_update_interval_;
    double resolution_;
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;

    // Karto bookkeeping
    karto::mapper::MapperPtr mapper_;
    karto::localizer::LocalizerPtr localizer_;
    std::map<std::string, bool> lasers_added_to_karto_;
    karto::Pose2 last_corrected_pose_;
    karto::Pose2 localizer_offset_;
    bool apply_localizer_offset_;

    // Internal state
    bool got_map_;
    int laser_count_;
    boost::thread* transform_thread_;
    tf::Transform map_to_odom_;
    unsigned marker_count_;

    // debug
    karto::DatasetWriterPtr w1, w2;
};

SlamKarto::SlamKarto() :
        apply_localizer_offset_(false),
        got_map_(false),
        laser_count_(0),
        transform_thread_(NULL),
        marker_count_(0)
{
  map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 1.0;
  map_update_interval_.fromSec(tmp);
  if(!private_nh_.getParam("resolution", resolution_))
  {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    if(!private_nh_.getParam("delta", resolution_))
      resolution_ = 0.05;
  }
  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);

  // Set up advertisements and subscriptions
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);
  particles_publisher_ = node_.advertise<geometry_msgs::PoseArray>("particles_array",1);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));

  // Initialize Karto structures
  mapper_ = karto::mapper::CreateDefaultMapper();
  localizer_ = karto::localizer::CreateDefaultLocalizer();

  // Initialize configuration object. The real defaults will always be set by
  // the first reconfiguration callback (which is called inside setCallback()
  // and sets the parameters from the launch file).
  config_.add_scans = false;
  config_.update_map = false;

  // Callback for dynamic reconfiguration
  server_.setCallback(boost::bind(&SlamKarto::reconfigurationCallback, this, _1, _2));

  // Register services to save and restore the mapper state
  saveMapperStateService_ = node_.advertiseService("slam_karto/SaveMapperState",
    SaveMapperStateCallback(boost::bind(&SlamKarto::saveMapperState, this, _1, _2)));
  loadMapperStateService_ = node_.advertiseService("slam_karto/LoadMapperState",
    LoadMapperStateCallback(boost::bind(&SlamKarto::loadMapperState, this, _1, _2)));

  // Send out map (even if it's empty) so RViz can reflect the current state
  //updateMap();
  // TODO send out an empty map. Not so easy after all.

  ROS_INFO("slam_karto configuration: add_scans = %s, update_map = %s",
           config_.add_scans ? "true" : "false",
           config_.update_map ? "true" : "false");

  w1 = karto::DatasetWriter::CreateWriter("/tmp/dataset_loc.kxd");
  w2 = karto::DatasetWriter::CreateWriter("/tmp/dataset_map.kxd");
}

SlamKarto::~SlamKarto()
{
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

void
SlamKarto::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
    if(laser_count_ != 0)    // play nice with "rosbag play --clock" and don't
      publishTransform();    //   send out transforms before the first scan
    r.sleep();
  }
}

void
SlamKarto::publishTransform()
{
  boost::mutex::scoped_lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_.sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
}

bool
SlamKarto::addLaserToKarto(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Check whether we know about this laser yet
  if(lasers_added_to_karto_.find(scan->header.frame_id) == lasers_added_to_karto_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<btTransform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
	       e.what());
      return false;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
	     scan->header.frame_id.c_str(),
	     laser_pose.getOrigin().x(),
	     laser_pose.getOrigin().y(),
	     yaw);
    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, scan->header.stamp,
                                      scan->header.frame_id);
    q.setRPY(0.0, 0.0, scan->angle_max);
    tf::Stamped<tf::Quaternion> max_q(q, scan->header.stamp,
                                      scan->header.frame_id);
    try
    {
      tf_.transformQuaternion(base_frame_, min_q, min_q);
      tf_.transformQuaternion(base_frame_, max_q, max_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return false;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_max = tf::getYaw(max_q);
    bool inverse = angle_max < angle_min;
    if (!inverse)
      assert(0); // had to drop that functionality //ROS_INFO("laser is mounted upside-down");


    // Create a laser range finder device and copy in data from the first scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, karto::Identifier(name.c_str()));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
                                      laser_pose.getOrigin().y(),
                                      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    //laser_->SetRangeThreshold(12.0);

    ROS_INFO("Adding a laser to the mapper: (%.2f,%.2f) yaw %.1f, range: [%.1f %.1f], angle: [%.1f %.1f]",
             laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw,
             scan->range_min, scan->range_max, scan->angle_min, scan->angle_max);

    // Add to Karto
    if(!mapper_->Process(laser))
      return false;

    // Remember we added this device
    lasers_added_to_karto_[scan->header.frame_id] = true;
  }
  return true;
}

bool
SlamKarto::getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(tf::createQuaternionFromRPY(0,0,0),
                                           btVector3(0,0,0)), t, base_frame_);
  tf::Stamped<btTransform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose =
          karto::Pose2(odom_pose.getOrigin().x(),
                       odom_pose.getOrigin().y(),
                       yaw);
  return true;
}

void
SlamKarto::publishGraphVisualization()
{
  //if(!solver_)
  return;

  std::vector<float> graph;
  //solver_->getGraph(graph);
  //karto::MapperGraph* graph = mapper_->GetGraph();

  visualization_msgs::MarkerArray marray;

  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint id = 0;
  for (uint i=0; i<graph.size()/2; i++)
  {
    m.id = id;
    m.pose.position.x = graph[2*i];
    m.pose.position.y = graph[2*i+1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    if(i>0)
    {
      edge.points.clear();

      geometry_msgs::Point p;
      p.x = graph[2*(i-1)];
      p.y = graph[2*(i-1)+1];
      edge.points.push_back(p);
      p.x = graph[2*i];
      p.y = graph[2*i+1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::Marker::DELETE;
  for (; id < marker_count_; id++)
  {
    m.id = id;
    marray.markers.push_back(visualization_msgs::Marker(m));
  }

  marker_count_ = marray.markers.size();

  marker_publisher_.publish(marray);
}

void
SlamKarto::publishParticlesVisualization()
{
	geometry_msgs::PoseArray arr;
	std::vector<geometry_msgs::Pose> poses;

	if(!config_.update_map) {      // The mapper does not expose its particles
        boost::mutex::scoped_lock(map_mutex_);
		karto_const_forEach(karto::localizer::ParticleList, &localizer_->GetParticles()) {
			const karto::localizer::Particle& p = (*iter);
			geometry_msgs::Pose pose;
			pose.position.x = p.x;
			pose.position.y = p.y;
			pose.position.z = 0;

			tf::Quaternion q = tf::createQuaternionFromYaw(p.heading);
			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();
			arr.poses.push_back(pose);
		}
	}

	arr.header.stamp = ros::Time::now();
	arr.header.frame_id = map_frame_;

	particles_publisher_.publish(arr);

}

void
SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  // Drop scans when user requested that no scans should be added. Can be useful
  // if we have to wait for some external event (e.g., odometry stabilizing).
  if(!config_.add_scans)
    return;

  static ros::Time last_map_update(0,0);

  // Check whether Karto knows about this laser yet
  if(!addLaserToKarto(scan))
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
	     scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if(addScan(scan, odom_pose))
  {
    ROS_INFO("added scan at pose: %.3f %.3f %.3f",
              odom_pose.GetX(),
              odom_pose.GetY(),
              odom_pose.GetHeading());

    publishGraphVisualization();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      if(updateMap())
      {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }

  publishParticlesVisualization();
/*
  try {
  karto::Sensor* s = karto::SensorRegistry::GetInstance()->GetSensorByName("laserHoriz");
  if(s) {
	  karto::Pose2 p = s->GetOffsetPose();
	  ROS_INFO_STREAM("Pose is " << p.ToString().ToCString() <<
			  " last corr: " << last_corrected_pose_.ToString().ToCString());

  }
  } catch(karto::Exception e) {}*/
}

bool
SlamKarto::updateMap()
{
  boost::mutex::scoped_lock(map_mutex_);

  karto::OccupancyGridPtr occ_grid =
          karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if(!occ_grid) {
    ROS_ERROR("No occ grid!");
    return false;
  }

  if(!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if(map_.map.info.width != (unsigned int) width ||
     map_.map.info.height != (unsigned int) height ||
     map_.map.info.origin.position.x != offset.GetX() ||
     map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y=0; y<height; y++)
  {
    for (kt_int32s x=0; x<width; x++)
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  return true;
}

bool
SlamKarto::addScan(const sensor_msgs::LaserScan::ConstPtr& scan,
                   karto::Pose2& karto_pose)
{
  if(!getOdomPose(karto_pose, scan->header.stamp))
     return false;

  // Create a vector of doubles for karto
  karto::RangeReadingsList readings;

  /*if (lasers_inverted_[scan->header.frame_id]) {*/
    for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
      it != scan->ranges.rend();
      ++it)
    {
      readings.Add(*it);
    }
  /*} else {
    for(std::vector<float>::const_iterator it = scan->ranges.begin();
      it != scan->ranges.end();
      ++it)
    {
      readings.Add(*it);
    }
  //}*/

  // Create the Karto identifier of the laser scanner
  karto::Identifier laserId(scan->header.frame_id.c_str());

  // create localized range scan
  karto::LocalizedRangeScan* range_scan = new karto::LocalizedRangeScan(laserId, readings);
  range_scan->SetOdometricPose(karto_pose);

  //localizer_offset_ = karto::Pose2(.5, .5, 0);
  if(!apply_localizer_offset_) {
	  range_scan->SetCorrectedPose(karto_pose);
  } else {
	  range_scan->SetCorrectedPose(karto_pose + localizer_offset_);
	  //apply_localizer_offset_ = false;
	  ROS_INFO_STREAM("Mapper bootstrapped with odometric pose " << karto_pose);
	  ROS_INFO_STREAM("                     and corrected pose " << karto_pose + localizer_offset_);
  }

  //ROS_INFO_STREAM("Karto pose: " << karto_pose);

  // Add the localized range scan to the mapper
  bool processed;

  karto::Module* module = config_.update_map /*(kartoMode_ == Mapping)*/
                        ? static_cast<karto::Module*>(mapper_)
                        : static_cast<karto::Module*>(localizer_);

  boost::mutex::scoped_lock(map_mutex_);
  if((processed = module->Process(range_scan)))
  {
    std::cout << (config_.update_map ? "[MAPPER] " : "[LOCALIZER] ")
              << "Pose: " << range_scan->GetOdometricPose()
              << " Corrected Pose: " << range_scan->GetCorrectedPose()
              << (range_scan->IsScanMatched() ? " MATCHED" : " not matched")
              << std::endl;
    if(!config_.update_map)
    	*w1 << range_scan;
    else
       	*w2 << range_scan;

    if(range_scan->IsScanMatched()) {
		last_corrected_pose_ = range_scan->GetCorrectedPose();
		//if(!apply_localizer_offset_)
		  localizer_offset_ = last_corrected_pose_ - range_scan->GetOdometricPose();

		// Compute the map->odom transform
		tf::Stamped<tf::Pose> odom_to_map;
		try
		{
		  tf_.transformPose(
			  odom_frame_,
			  tf::Stamped<tf::Pose>(
				btTransform(
				  tf::createQuaternionFromRPY(0, 0, last_corrected_pose_.GetHeading()),
				  btVector3(last_corrected_pose_.GetX(), last_corrected_pose_.GetY(), 0.0)
				).inverse(),
				scan->header.stamp,
				base_frame_
			  ),
			  odom_to_map);
		}
		catch(tf::TransformException e)
		{
		  ROS_ERROR("Transform from base_link to odom failed\n");
		  odom_to_map.setIdentity();
		}

		map_to_odom_mutex_.lock();
		map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
									 tf::Point( odom_to_map.getOrigin().getX(),
												odom_to_map.getOrigin().getY(),
												0. /* never adjust Z */ ) ).inverse();
		map_to_odom_mutex_.unlock();
    } else {
    	ROS_INFO("Not adding unmatched scan");
    }
  } else {
    //delete range_scan;
	  //ROS_WARN("Leaking a range scan");
  }
  return processed;
}

bool
SlamKarto::mapCallback(nav_msgs::GetMap::Request  &req,
                       nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void
SlamKarto::reconfigurationCallback(karto::KartoConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock(map_mutex_);

  if(config_.add_scans != config.add_scans)
    ROS_INFO("Incoming laser scans are %s.", config.add_scans
             ? "used" : "NOT used (GMapping is paused)");

  bool set_localizer_config = false;
  bool set_localizer_last_corrected_pose  = false;

  // Note: config is the new configuration, config_ the old one.

  if(config_.update_map != config.update_map) {
    ROS_INFO("Map updates are %s.", config.update_map ? "enabled" : "disabled");

    // Set localizer position when switching from mapping to localizing.
    // Further, call setLocalizerConfig() to copy occupancy grid map from
    // mapper to localizer and initialize the probability distribution.
    if(!config.update_map) {
      // Switching from mapper to localizer
      set_localizer_last_corrected_pose = true;
      set_localizer_config = true;
      apply_localizer_offset_ = false;
    } else {
      // Switching from localizer to mapper

      // Find the scanners and set their pose
      /*const karto::LocalizedObjectList& objects = mapper_->GetAllProcessedObjects();
      karto_const_forEach(karto::LocalizedObjectList, &objects) {
    	  ROS_INFO_STREAM("got an object" << (*iter)->GetIdentifier().GetName().ToCString());
          if(karto::IsSensor((*iter).Get())) {
        	  ROS_INFO("found a sensor");
            mapper_->LocalizeSensorAt(dynamic_cast<karto::Sensor*>((*iter).Get()),
            		last_corrected_pose_);
          }

      }*/
      try {
		  karto::Sensor* s = karto::SensorRegistry::GetInstance()->GetSensorByName("laserHoriz");
		  if(!s)
			  ROS_ERROR("Cannot get sensor");
		  else {
			  ROS_INFO_STREAM("Setting mapper position to "
					   << last_corrected_pose_ /*
					   << " + " << localizer_offset_ << " = "
					   << last_corrected_pose_ + localizer_offset_*/);
			  mapper_->LocalizeSensorAt(s, last_corrected_pose_);
		  }
      } catch (karto::Exception error) {
		ROS_ERROR_STREAM("Exception in karto (reconf cb): " << error.GetErrorMessage());
	  }

      // Apply last correction (the odom_to_map transform) determined by
      // localizer to the mapper. There should be a better way to do this...
      //apply_localizer_offset_ = true;
      //ROS_WARN("ENABLED MAPPER");
    }

  }

  // Changing localizer mode
  if(config_.dead_reckoning != config.dead_reckoning) {
    set_localizer_config = true;
    // If dead-reckoning is being disabled, set the localizer position to the
    // last known position so it can take over from there. If it's being
    // enabled, no start position is required as the particles will be
    // distributed uniformly.
    if(!config.dead_reckoning)
      set_localizer_last_corrected_pose = true;
  }

  config_ = config;

  if(set_localizer_config || set_localizer_last_corrected_pose)
    setLocalizerConfig();

  if(set_localizer_last_corrected_pose) {
	  ROS_INFO_STREAM("Using last corrected pose to bootstrap localizer: "
			  << last_corrected_pose_.ToString().ToCString());
    localizer_->SetStartPosition(last_corrected_pose_);
  }
}

bool
SlamKarto::saveMapperState(karto::SaveMapperState::Request& request,
                           karto::SaveMapperState::Response& response)
{
  boost::mutex::scoped_lock(map_mutex_);

  ROS_INFO_STREAM("Saving mapper state to " << request.filename);
  try {
    if(!karto::mapper::WriteState(mapper_, request.filename.c_str()))
      return false;
  }
  catch (karto::Exception error) {
    ROS_ERROR_STREAM("Exception in karto: " << error.GetErrorMessage());
    return false;
  }
  return true;
}


bool
SlamKarto::loadMapperState(karto::LoadMapperState::Request& request,
                           karto::LoadMapperState::Response& response)
{
  ROS_INFO_STREAM("Loading mapper state from " << request.filename);

  {
    boost::mutex::scoped_lock(map_mutex_);
    try {
      mapper_ = karto::mapper::CreateDefaultMapper();
      lasers_added_to_karto_.clear();

      karto::mapper::ReadState(mapper_, request.filename.c_str());

      // When loading the state, laser scanners are added automatically to
      // Karto. In order to find out which ones where added, iterate the list
      // and mark them as added, so they won't get added a second time in the
      // laserCallback().
      karto::List<karto::Identifier> sensorIds =
        mapper_->GetMapperSensorManager()->GetSensorNames();

      karto_forEach(karto::List<karto::Identifier>, &sensorIds) {
        ROS_INFO_STREAM("Loaded a laser scanner description: " << (*iter).GetName());
        lasers_added_to_karto_[(*iter).GetName().ToCString()] = true;
      }

      // We need to find our current position using the localizer, i.e.,
      // without modifying the map
      //localizer_->Reset();

      config_.update_map = false;      // switch to localizer module
      config_.dead_reckoning = true;   // assume position is unknown
      server_.updateConfig(config_);   // publish modified configuration

      setLocalizerConfig();
      apply_localizer_offset_ = false;

    } catch(karto::Exception error) {
        ROS_ERROR_STREAM("Exception in karto: " << error.GetErrorMessage());
        return false;
    }
  } // release mutex here as updateMap() will lock it, too.

  // send out map
  updateMap();

  ROS_INFO_STREAM("Mapper state loading complete.");
  return true;
}

void
SlamKarto::setLocalizerConfig()
{
  try {
    karto::OccupancyGridPtr occ_grid =
      karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);
    if(!occ_grid.IsValid()) {
      ROS_ERROR_STREAM("Occupancy grid is NULL in setLocalizerConfig()");
      return;
    }

    if(!localizer_->SetOccupancyGrid(occ_grid.Get())) {
      ROS_ERROR_STREAM("SetOccupancyGrid failed in setLocalizerConfig()");
      return;
    }

    if(config_.dead_reckoning) {
      // use large number of particles for the particle filter algorithm
      ROS_INFO_STREAM("Setting localizer module to dead-reckoning mode");
      localizer_->SetParameters("NumberOfParticles", config_.part_count_dead);
      localizer_->SetParameters("SamplingInterval", 1);  // use all scan points
      localizer_->SetParameters("ParticleDistribution", "Uniform");
      localizer_->SetParameters("LocalizedPoseCriterion", "Best Particle");
    } else {
      ROS_INFO_STREAM("Setting localizer module to normal mode");
      localizer_->SetParameters("NumberOfParticles", config_.part_count);
      localizer_->SetParameters("SamplingInterval", 1);  // use all scan points
      localizer_->SetParameters("ParticleDistribution", "Gauss");
      localizer_->SetParameters("LocalizedPoseCriterion", "Weighted Mean");
      //localizer_->SetParameters("LocalizedPoseCriterion", "Best Particle");
    }

    ROS_INFO_STREAM("Resetting probability distribution");
    localizer_->ResetDistribution();

  } catch (karto::Exception error) {
    ROS_ERROR_STREAM("Exception in karto in setLocalizerConfig(): " << error.GetErrorMessage());
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_karto");
  try {
    karto::Environment::Initialize(argc, argv);

    SlamKarto kn;
    ros::spin();

  } catch (karto::Exception error) {
    ROS_ERROR_STREAM("Exception in karto: " << error.GetErrorMessage());
  }

  // After kn has gone out of scope!
  karto::Environment::Terminate();
  ros::shutdown();

  return 0;
}
