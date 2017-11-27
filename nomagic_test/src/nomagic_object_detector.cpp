#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nomagic_test/DetectObject.h"
#include"ransac_line2d.h"

class Workspace {
  public:
    double x_min;
    double x_max;
    double y_min;
    double y_max;
};

sensor_msgs::LaserScan last_scan;

Workspace workspace = {0.5, 1.5, -0.5, 0.5};

void laserToPoints(const sensor_msgs::LaserScan &scan, const Workspace &work, std::vector<sac::Point2D> *points) {
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    if ((scan.ranges[i] < scan.range_min) || (scan.ranges[i] > scan.range_max)) {
      continue;
    }
    
    double angle = scan.angle_min + (scan.angle_max - scan.angle_min) * (double)i/(double)scan.ranges.size(); 
    
    sac::Point2D point;
    
    point.x = scan.ranges[i] * cos(angle);
    point.y = scan.ranges[i] * sin(angle);
    
    if ((point.x > work.x_min) && (point.x < work.x_max) &&
        (point.y > work.y_min) && (point.y < work.y_max)) {
      points->push_back(point);    
    }
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  last_scan = *msg;
}

bool detectObject(nomagic_test::DetectObject::Request  &req,
         nomagic_test::DetectObject::Response &res)
{
#if 0
  res.detected = true;
  
  res.pose.orientation.w = 1.0;
  res.pose.orientation.x = 0.0;
  res.pose.orientation.y = 0.0;
  res.pose.orientation.z = 0.0;
  
  res.pose.position.x = 1.0;
  res.pose.position.y = 0.0;
  res.pose.position.z = 1.3;
  return true;
#else

  std::vector<sac::Point2D> points;
  laserToPoints(last_scan, workspace, &points);

  std::cout << "points : " << points.size() << std::endl;
  
  sac::ransacModelLine2D line2D;
	std::vector<int> inliers;
	std::vector<sac::ModelCoefficient> lines;
	line2D.setDistanceThreshold(0.005);
	line2D.setMaxIterations(1000);
  
  while (points.size() > 5) {
    sac::ModelCoefficient parameters;
		line2D.setInputCloud(points);
		line2D.computeModel();
		line2D.getModelCoefficients(parameters);
  
    lines.push_back(parameters);
    
    std::cout << "Parameter of 2D line: < " << parameters.modelParam[0] << ", " <<
			parameters.modelParam[1] << " >---< " << parameters.modelParam[2] << ", " <<
			parameters.modelParam[3] << " > " << std::endl;
    
    line2D.getInliers(inliers);
    
    std::cout << "inlayers : " << inliers.size() << std::endl;
    
		line2D.removeInliders(points, inliers);
  }

  if (lines.size() == 2) {
    sac::Point2D corner;
    
    sac::Point2D p11(lines[0].modelParam[0], lines[0].modelParam[1]);
		sac::Point2D p12(lines[0].modelParam[2], lines[0].modelParam[3]);  
  
    sac::Point2D p21(lines[1].modelParam[0], lines[1].modelParam[1]);
		sac::Point2D p22(lines[1].modelParam[2], lines[1].modelParam[3]); 
  
    double denominator = (p11.x - p12.x) * (p21.y - p22.y) - (p11.y - p12.y) * (p21.x - p22.x);
  
    corner.x = ((p11.x * p12.y - p11.y * p12.x) * (p21.x - p22.x) - (p21.x * p22.y - p21.y * p22.x) * (p11.x - p12.x))/denominator;
    corner.y = ((p11.x * p12.y - p11.y * p12.x) * (p21.y - p22.y) - (p21.x * p22.y - p21.y * p22.x) * (p11.y - p12.y))/denominator;
    
    double angle;
    if (p12.x > corner.x && p12.y > corner.y) {
      angle = atan2(p12.x - p11.x, p12.y - p11.y);
    } else {
      angle = atan2(p22.x - p21.x, p22.y - p21.y);
    }
    
    
    if (angle < 0.0) {
      angle += 3.14;
    }
    
    if (angle > 3.14/2.0) {
      angle -= 3.14/2.0;
    }
    
    std::cout << "corner point : " << corner.x << " - " << corner.y << " angle : " << angle/3.14 * 180 << std::endl;
    
    corner.x += cos(angle) * 0.05 - sin(angle) * -0.05;
    corner.y += sin(angle) * 0.05 + cos(angle) * -0.05;
    
    res.pose.position.x = 2.0 - corner.x;
    res.pose.position.y = -corner.y;
    res.pose.position.z = 1.3;
    
    res.pose.orientation.w = cos(angle/2.0);
    res.pose.orientation.x = 0;
    res.pose.orientation.y = 0;
    res.pose.orientation.z = sin(angle/2.0);
    
    res.detected = true;
    
    return true;
  } else {
    res.detected = false;
    return true;
  }

#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nomagic_detector");
  ros::NodeHandle node_handle;  
  
  ros::Subscriber sub = node_handle.subscribe("/laser_scan", 10, laserCallback);
  ros::ServiceServer service = node_handle.advertiseService("/detect_object", detectObject);

  ros::spin(); 
  return 0;
}
