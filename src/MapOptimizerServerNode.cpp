/*

Master Thesis Project
Victor Felipe Vallejo Rodriguez
TU Dortmund


Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/thread.hpp>

using namespace rtabmap;

class MapOptimizerServer
{

public:
	MapOptimizerServer() :
		mapFrameId_("map"),
		odomFrameId_("odom"),
		globalOptimization_(true),
		optimizeFromLastNode_(false),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		double epsilon = 0.0;
		bool robust = true;
		bool slam2d =false;
		int strategy = 1; // 0=TORO, 1=g2o, 2=GTSAM
		int iterations = 20;
		bool ignoreVariance = false;

		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("iterations", iterations, iterations);
		pnh.param("ignore_variance", ignoreVariance, ignoreVariance);
		pnh.param("global_optimization", globalOptimization_, globalOptimization_);
		pnh.param("optimize_from_last_node", optimizeFromLastNode_, optimizeFromLastNode_);
		pnh.param("epsilon", epsilon, epsilon);
		pnh.param("robust", robust, robust);
		pnh.param("slam_2d", slam2d, slam2d);
		pnh.param("strategy", strategy, strategy);


		UASSERT(iterations > 0);

		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kOptimizerStrategy(), uNumber2Str(strategy)));
		parameters.insert(ParametersPair(Parameters::kOptimizerEpsilon(), uNumber2Str(epsilon)));
		parameters.insert(ParametersPair(Parameters::kOptimizerIterations(), uNumber2Str(iterations)));
		parameters.insert(ParametersPair(Parameters::kOptimizerRobust(), uBool2Str(robust)));
		parameters.insert(ParametersPair(Parameters::kRegForce3DoF(), uBool2Str(slam2d)));
		parameters.insert(ParametersPair(Parameters::kOptimizerVarianceIgnored(), uBool2Str(ignoreVariance)));
		optimizer_ = Optimizer::create(parameters);

		double tfDelay = 0.05; // 20 Hz
		double tfTolerance = 0.1; // 100 ms
		bool publishTf = true;
		pnh.param("publish_tf", publishTf, publishTf);
		pnh.param("tf_delay", tfDelay, tfDelay);
		pnh.param("tf_tolerance", tfTolerance, tfTolerance);

		mapDataTopic_ = nh.subscribe("mapData", 1, &MapOptimizerServer::mapDataReceivedCallback, this);
		mapDataPub_ = nh.advertise<rtabmap_ros::MapData>(nh.resolveName("mapData")+"_optimized", 1);
		mapGraphPub_ = nh.advertise<rtabmap_ros::MapGraph>(nh.resolveName("mapData")+"Graph_optimized", 1);

		if(publishTf)
		{
			ROS_INFO("map_optimizer_server will publish tf between frames \"%s\" and \"%s\"", mapFrameId_.c_str(), odomFrameId_.c_str());
			ROS_INFO("map_optimizer_server: map_frame_id = %s", mapFrameId_.c_str());
			ROS_INFO("map_optimizer_server: odom_frame_id = %s", odomFrameId_.c_str());
			ROS_INFO("map_optimizer_server: tf_delay = %f", tfDelay);
			ROS_INFO("map_optimizer_server: tf_tolerance  = %f", tfTolerance);
			transformThread_ = new boost::thread(boost::bind(&MapOptimizerServer::publishLoop, this, tfDelay, tfTolerance));
		}
	}

	~MapOptimizerServer()
	{
		if(transformThread_)
		{
			transformThread_->join();
			delete transformThread_;
		}
	}

	void publishLoop(double tfDelay, double tfTolerance)
	{
		if(tfDelay == 0)
			return;
		ros::Rate r(1.0 / tfDelay);
		while(ros::ok())
		{
			mapToOdomMutex_.lock();
			ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfTolerance);
			geometry_msgs::TransformStamped msg;
			msg.child_frame_id = odomFrameId_;
			msg.header.frame_id = mapFrameId_;
			msg.header.stamp = tfExpiration;
			rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);
			tfBroadcaster_.sendTransform(msg);
			mapToOdomMutex_.unlock();
			r.sleep();
		}
	}

	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		static bool initialPose_ = true;
		static int lastProcessedRobot_ = 2000000;
		// save new poses and constraints
		// Assuming that nodes/constraints are all linked together
		UASSERT(msg->graph.posesId.size() == msg->graph.poses.size());

		// To do, add robot numbre to 1) save message information overtime 2) correct message size at the end for publishing
		robotNo_=msg->nodes[0].id/robotIdDiv_;
		ROS_INFO("Processing Robot No: %d", robotNo_);

		// add functinality for multiple robots variable
		// if(msg->robotNo!=lastRobotProcessed)
		if(robotNo_!=lastProcessedRobot_ && lastProcessedRobot_!=/******0******/2000000) //Change for a variable (Test purposes)
		{
			robotChanged_=true;
			ROS_INFO("Processing New Robot!");
		}
		else
		{
			robotChanged_=false;
			ROS_INFO("Processing Previous Robot");
		}

		bool dataChanged = false;

		std::multimap<int, Link> newConstraints;
		for(unsigned int i=0; i<msg->graph.links.size(); ++i)
		{
			Link link = rtabmap_ros::linkFromROS(msg->graph.links[i]);
			newConstraints.insert(std::make_pair(link.from(), link));

			bool edgeAlreadyAdded = false;
			for(std::multimap<int, Link>::iterator iter = cachedConstraints_.lower_bound(link.from());
					iter != cachedConstraints_.end() && iter->first == link.from();
					++iter)
			{
				if(iter->second.to() == link.to())
				{
					edgeAlreadyAdded = true;
					if(iter->second.transform().getDistanceSquared(link.transform()) > 0.0001)
					{
						ROS_INFO("%d ->%d (%s vs %s)",iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str(),
								link.transform().prettyPrint().c_str());
						ROS_WARN("%d ->%d (%s vs %s)",iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str(),
								link.transform().prettyPrint().c_str());
						dataChanged = true;
					}
				}
			}
			if(!edgeAlreadyAdded)
			{
				cachedConstraints_.insert(std::make_pair(link.from(), link));
			}
		}

		std::map<int, Signature> newNodeInfos;
		int id;
		
		if(msg->nodes.size()==1 && initialPose_)
		{
			id = msg->nodes[0].id;
			ROS_INFO("First Node #: %d", id);

			Transform pose = rtabmap_ros::transformFromPoseMsg(msg->nodes[0].pose);
			Signature s = rtabmap_ros::nodeInfoFromROS(msg->nodes[0]);
			newNodeInfos.insert(std::make_pair(id, s));
			cachedNodeInfos_.insert(std::make_pair(id, s));
			initialPose_ = false;
		}
		

		if(newConstraints.size() > 0)
		{
			ROS_INFO("Adding new Odometry poses...");

			// add new odometry poses
			for(unsigned int i=0; i<msg->nodes.size(); ++i)
			{
				id = msg->nodes[i].id;

				ROS_INFO("Node #: %d", id);

				Transform pose = rtabmap_ros::transformFromPoseMsg(msg->nodes[i].pose);
				Signature s = rtabmap_ros::nodeInfoFromROS(msg->nodes[i]);
				newNodeInfos.insert(std::make_pair(id, s));

				std::pair<std::map<int, Signature>::iterator, bool> p = cachedNodeInfos_.insert(std::make_pair(id, s));

				ROS_INFO("Nodes to optimize: %d", (int)cachedNodeInfos_.size());

				if(!p.second && pose.getDistanceSquared(cachedNodeInfos_.at(id).getPose()) > 0.0001)
				{
					dataChanged = true;
				}
			}

			if(dataChanged)
			{
				ROS_INFO("Graph data has changed! Reset cache...");
				ROS_WARN("Graph data has changed! Reset cache...");
				cachedConstraints_ = newConstraints;
				cachedNodeInfos_ = newNodeInfos;
			}
		}

		//match poses in the graph
		std::multimap<int, Link> constraints;
		std::map<int, Signature> nodeInfos;
		if(globalOptimization_)
		{
			ROS_INFO("GLOBAL OPTIMIZATION");
			constraints = cachedConstraints_;
			nodeInfos = cachedNodeInfos_;
		}
		else
		{
			constraints = newConstraints;
			for(unsigned int i=0; i<msg->graph.posesId.size(); ++i)
			{
				std::map<int, Signature>::iterator iter = cachedNodeInfos_.find(msg->graph.posesId[i]);
				if(iter != cachedNodeInfos_.end())
				{
					nodeInfos.insert(*iter);
				}
				else
				{
					ROS_INFO("Odometry pose of node %d not found in cache!", msg->graph.posesId[i]);
					ROS_ERROR("Odometry pose of node %d not found in cache!", msg->graph.posesId[i]);
					return;
				}
			}
		}

		std::map<int, Transform> poses;
		for(std::map<int, Signature>::iterator iter=nodeInfos.begin(); iter!=nodeInfos.end(); ++iter)
		{
			poses.insert(std::make_pair(iter->first, iter->second.getPose()));
		}

		ROS_INFO("Poses in cache: %d", (int)poses.size());
		ROS_INFO("Constraints in cache: %d", (int)constraints.size());

		// Optimize only if there is a subscriber
		if(mapDataPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers())
		{
			UTimer timer;
			std::map<int, Transform> optimizedPoses;
			Transform mapCorrection = Transform::getIdentity();
			std::map<int, rtabmap::Transform> posesOut;
			std::multimap<int, rtabmap::Link> linksOut;
			if(poses.size() > 1 && constraints.size() > 0)
			{
				int fromId = optimizeFromLastNode_?poses.rbegin()->first:poses.begin()->first;

				ROS_INFO("Optimizing from: %d", fromId);
				
				//optimizer_->getConnectedGraph(
				//		fromId,
				//		poses,
				//		constraints,
				//		posesOut,
				//		linksOut);
				posesOut=poses;
				linksOut=constraints;

				optimizedPoses = optimizer_->optimize(fromId, posesOut, linksOut);
				mapToOdomMutex_.lock();
				mapCorrection = optimizedPoses.at(posesOut.rbegin()->first) * posesOut.rbegin()->second.inverse();
				mapToOdom_ = mapCorrection;
				mapToOdomMutex_.unlock();

				ROS_INFO("Poses connected graph: %d", (int)posesOut.size());
			}
			else if(poses.size() == 1 && constraints.size() == 0)
			{
				optimizedPoses = poses;
			}
			else if(poses.size() == 0 && constraints.size())
			{
				ROS_INFO("map_optimizer_server: Poses=%d and edges=%d: poses must "
					   "not be null if there are edges.",
					  (int)poses.size(), (int)constraints.size());
				ROS_ERROR("map_optimizer_server: Poses=%d and edges=%d: poses must "
					   "not be null if there are edges.",
					  (int)poses.size(), (int)constraints.size());
			}

			rtabmap_ros::MapData outputDataMsg;
			rtabmap_ros::MapGraph outputGraphMsg;
			rtabmap_ros::mapGraphToROS(optimizedPoses,
					linksOut,
					mapCorrection,
					outputGraphMsg);

			if(mapGraphPub_.getNumSubscribers())
			{
				outputGraphMsg.header = msg->header;
				mapGraphPub_.publish(outputGraphMsg);
			}

			if(mapDataPub_.getNumSubscribers())
			{
				ROS_INFO("map_optimizer_server: Ready to publish:");
				outputDataMsg.header = msg->header;
				outputDataMsg.graph = outputGraphMsg;
				outputDataMsg.nodes = msg->nodes;
				if(posesOut.size() > msg->nodes.size())
				{
					ROS_INFO("PosesOut larger than message size");
					
					std::set<int> addedNodes;
				
					if(!robotChanged_)
					{
						for(unsigned int i=0; i<msg->nodes.size(); ++i)
						{
							addedNodes.insert(msg->nodes[i].id);
						}
					}
					std::list<int> toAdd;
					for(std::map<int, Transform>::iterator iter=posesOut.begin(); iter!=posesOut.end(); ++iter)
					{
						if(addedNodes.find(iter->first) == addedNodes.end())
						{
							toAdd.push_back(iter->first);
						}
					}
					if(toAdd.size())
					{
						ROS_INFO("map_optimizer_server: Resizing Message");
						ROS_INFO("map_optimizer_server: Old size message: %d", (int)outputDataMsg.nodes.size());
						
						int oi;
						if(robotChanged_)
						{
							oi = 0;
						}
						else
						{
							oi = outputDataMsg.nodes.size();
						}

						outputDataMsg.nodes.resize(oi+toAdd.size());
						for(std::list<int>::iterator iter=toAdd.begin(); iter!=toAdd.end(); ++iter)
						{
							UASSERT(cachedNodeInfos_.find(*iter) != cachedNodeInfos_.end());
							rtabmap_ros::nodeDataToROS(cachedNodeInfos_.at(*iter), outputDataMsg.nodes[oi]);
							++oi;
						}

						ROS_INFO("map_optimizer_server: New size message: %d", (int)outputDataMsg.nodes.size());
					}
				}
				mapDataPub_.publish(outputDataMsg);
				
				//Extract nodes IDs for comparison later on when the message changes size when we recieve a different robot information
				// IF WE CREATE VARIABLE FOR ROBOTS this aproach MUST CHANGE
				//for(unsigned int i=0; i<outputDataMsg->nodes.size(); ++i)
				//{
				//	publishedNodes_.insert(outputDataMsg->nodes[i].id);
				//}
			}

			ROS_INFO("map_optimizer_server: Time graph optimization = %f s", timer.ticks());
		}
		lastProcessedRobot_=robotNo_;
	}

private:
	std::string mapFrameId_;
	std::string odomFrameId_;
	bool globalOptimization_;
	bool optimizeFromLastNode_;
	Optimizer * optimizer_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Subscriber mapDataTopic_;

	ros::Publisher mapDataPub_;
	ros::Publisher mapGraphPub_;

	std::multimap<int, Link> cachedConstraints_;
	std::map<int, Signature> cachedNodeInfos_;

	tf2_ros::TransformBroadcaster tfBroadcaster_;
	boost::thread* transformThread_;

	int robotNo_;
	bool robotChanged_;
	static const int robotIdDiv_ = 1000000;
	//std::set<int> publishedNodes_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_optimizer_server");
	MapOptimizerServer optimizer;
	ros::spin();
	return 0;
}
