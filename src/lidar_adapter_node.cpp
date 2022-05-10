#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <tf2_ros/transform_listener.h>

typedef PointMatcher<float> PM;

std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<PM::Transformation> transformator;
std::string imuFrame;
ros::Publisher pub;
PM::DataPointsFilters filters;

PM::TransformationParameters findTransform(const std::string& sourceFrame, const std::string& targetFrame, const ros::Time& time, const int& transformDimension)
{
	geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
	return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

void cloudCallback(const sensor_msgs::PointCloud2& msgIn)
{
	try
	{
		PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msgIn);
		filters.apply(cloud);
		PM::Matrix stamps = cloud.getDescriptorViewByName("t");
		cloud.addDescriptor("time", stamps/1e9);
		PM::TransformationParameters lidarToImu = findTransform(msgIn.header.frame_id, imuFrame, msgIn.header.stamp, 4);
		cloud = transformator->compute(cloud, lidarToImu);
		sensor_msgs::PointCloud2 msgOut = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(cloud, imuFrame, msgIn.header.stamp);
		msgOut.width = msgOut.width * msgOut.height;
		msgOut.height = 1;
		msgOut.row_step = 0;
		pub.publish(msgOut);
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_adpater_node");
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privateNodeHandle("~");

	std::shared_ptr<PM::DataPointsFilter> nanFilter = PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter");
	filters.push_back(nanFilter);
	PM::Parameters params;
	params["xMin"] = "-0.5";
	params["xMax"] = "0.5";
	params["yMin"] = "-0.4";
	params["yMax"] = "0.4";
	params["zMin"] = "-0.27";
	params["zMax"] = "0.23";
	params["removeInside"] = "1";
	std::shared_ptr<PM::DataPointsFilter> cubeFilter = PM::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
	filters.push_back(cubeFilter);

	transformator = PM::get().TransformationRegistrar.create("RigidTransformation");

	if(!privateNodeHandle.getParam("imu_frame", imuFrame))
	{
		ROS_ERROR("No imu_frame parameter provided");
		return 1;
	}

	tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
	tf2_ros::TransformListener tfListener(*tfBuffer);

	pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);
	ros::Subscriber sub = nodeHandle.subscribe("cloud_in", 10, cloudCallback);

	ros::spin();

	return 0;
}

