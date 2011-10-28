#include "ros/ros.h"
#include <classifier/classifier.h>
#include <visualization_msgs/Marker.h>


class Scan_Marker_Processor{
public:
  visualization_msgs::Marker line_list, points, circle;
  std::vector<visualization_msgs::Marker> circles;
  std::vector<visualization_msgs::Marker> v_pts;
  int id_counter;
  classifier::Collection c1;

  Scan_Marker_Processor():id_counter(0){

    ros::Duration ttl(0.5);
  
    line_list.header.frame_id = "/LDS_base";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "classifier";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    //line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.04;
    line_list.color.r = 1.0; //red color
    line_list.color.a = 1.0;
    line_list.lifetime = ttl;

    points.header.frame_id = "/LDS_base";
    points.header.stamp = ros::Time::now();
    points.ns = "classifier";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    //points.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.02;
    points.scale.y = 0.02;
    points.color.r = 1.0; //red color
    points.color.a = 1.0;
    points.lifetime = ttl;

    circle.header.frame_id = "/LDS_base";
    circle.header.stamp = ros::Time::now();
    circle.ns = "classifier";
    circle.action = visualization_msgs::Marker::ADD;
    circle.pose.orientation.w = 1.0;
    //circle.id = 2;
    circle.type = visualization_msgs::Marker::CYLINDER;
    circle.scale.x = 0.02;
    circle.scale.y = 0.02;
    circle.scale.z = 0.0;
    circle.color.r = 1.0; //red color
    circle.color.a = 1.0;
    circle.lifetime = ttl;



   
  }
  
  int get_id(){
    int temp = id_counter;
    id_counter++;
    return temp;
  }

  void reset_id_counter(){ id_counter = 0;}
  
  visualization_msgs::Marker return_points(){
    return points;
  }
  
  visualization_msgs::Marker return_lines(){
    return line_list;
  }
  
  void callBack(const sensor_msgs::LaserScan& scan){

    std::vector<classifier::Feature*> landmarks;
    //produce_collection(scan, landmarks);
    c1.produce_collection(scan, landmarks);

    std::vector<std::vector<classifier::Point> > point_clusters;
    //produce_cluster_points(scan, point_clusters);
    //produce_collection_from_points(point_clusters, landmarks);

    
    std::cout << "Number of Landmarks: " << landmarks.size() << std::endl;
    std::cout << landmarks << std::endl;
    reset_id_counter();
    line_list.points.clear();
    circles.clear();
    line_list.id = get_id();
    for(size_t ndx = 0; ndx < landmarks.size(); ndx++){
      geometry_msgs::Point p1, p2;
      std::vector<classifier::Point> ends;
      uint8_t type = landmarks[ndx]->get_type();
      if(type == LINE){
      	ends = landmarks[ndx]->get_ends();
	p1.x = ends[0].x/100;
	p1.y = ends[0].y/100;
	p2.x = ends[1].x/100;
	p2.y = ends[1].y/100;
	//std::cout << "Point center[" << ndx << "]: (" << p1.x << ", " << p1.y << ")" << std::endl;
	line_list.points.push_back(p1);
	line_list.points.push_back(p2);
      }
      /**/
      else if(type == CIRCLE){
	classifier::Point center = landmarks[ndx]->get_center();
	float radius = landmarks[ndx]->get_radius();
	circle.id = get_id();
	circle.pose.position.x = center.x/100;
	circle.pose.position.y = center.y/100;
	circle.scale.x = 2*radius/100;
	circle.scale.y = 2*radius/100;
	circles.push_back(circle);
	
      }
      
    }
    /**/
    ros::Duration ttl(0.5);
    v_pts.resize(point_clusters.size());
    for(size_t ndx = 0; ndx < point_clusters.size(); ndx++){
       v_pts[ndx].header.frame_id = "/LDS_base";
       v_pts[ndx].header.stamp = ros::Time::now();
       v_pts[ndx].ns = "classifier";
       v_pts[ndx].action = visualization_msgs::Marker::ADD;
       v_pts[ndx].pose.orientation.w = 1.0;
       v_pts[ndx].id = get_id();
       v_pts[ndx].type = visualization_msgs::Marker::POINTS;
       v_pts[ndx].scale.x = 0.02;
       v_pts[ndx].scale.y = 0.02;
       v_pts[ndx].color.r = (float)ndx/point_clusters.size(); //red color
       v_pts[ndx].color.b = (1 - (float)ndx/point_clusters.size()); //blue color
       v_pts[ndx].color.g = ndx%2;
       v_pts[ndx].color.a = 1.0;
       v_pts[ndx].lifetime = ttl;
       v_pts[ndx].points.clear();
      for(size_t iter = 0; iter < point_clusters[ndx].size(); iter++){
	geometry_msgs::Point p;
	p.x = point_clusters[ndx][iter].x/100;
	p.y = point_clusters[ndx][iter].y/100;
	v_pts[ndx].points.push_back(p);
      }
    }

  }
    

};

int main(int argc, char** argv){
  ros::init(argc, argv, "classifier");
  std::cout << "Classifier started" << std::endl;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<visualization_msgs::Marker>("viz_marker", 10);
  Scan_Marker_Processor smp;
  ros::Subscriber sub = n.subscribe("scan", 1, &Scan_Marker_Processor::callBack, &smp);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    pub.publish(smp.return_lines());
    for(uint8_t ndx = 0; ndx < smp.circles.size(); ndx++){
      pub.publish(smp.circles[ndx]);
    }
    for(size_t ndx = 0; ndx < smp.v_pts.size(); ndx++){
      pub.publish(smp.v_pts[ndx]);
    }
    smp.circles.clear();
    smp.line_list.points.clear();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
