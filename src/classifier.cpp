#include <classifier/classifier.h>


namespace classifier{
  
  RT::RT():radius(0), theta(0){}
  RT::RT(const float& radius, const float& theta):radius(radius), theta(theta){}
  
  Point::Point():x(0), y(0){}
  Point::Point(const float& x, const float& y):x(x), y(y){}
  bool Point::operator!=(const Point& rhs) const{
    if(x != rhs.x){return true;}
    if(y != rhs.y){return true;}
    return false;
  }

  Circle::Circle():radius(0), type(CIRCLE){}
  Circle::Circle(const Point& center, const float& radius):center(center), radius(radius), type(CIRCLE){}
  Circle::Circle(const std::vector<Point>& pts):type(CIRCLE){
    float avg_x, avg_y, u_c, v_c;
    float s_dxx, s_dyy, s_dxy, s_dxxx, s_dyyy, s_dxyy, s_dyxx;
    avg_x = avg_y = 0;
    s_dxx = s_dyy = s_dxy = s_dxxx = s_dyyy = s_dxyy = s_dyxx = 0; //sum of multiplied differences
    
    //Calculate averages
    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      avg_x = (1/ndx)*pts[ndx - 1].x + ((ndx - 1)/ndx)*avg_x;
      avg_y = (1/ndx)*pts[ndx - 1].y + ((ndx - 1)/ndx)*avg_y;
    }

    for(size_t ndx = 0; ndx < pts.size(); ndx++){
      s_dxx += pow(pts[ndx].x - avg_x, 2);
      s_dyy += pow(pts[ndx].y - avg_y, 2);
      s_dxy += (pts[ndx].x - avg_x)*(pts[ndx].y - avg_y);
      s_dxxx += pow(pts[ndx].x - avg_x, 3);
      s_dyyy += pow(pts[ndx].y - avg_y, 3);
      s_dxyy += (pts[ndx].x - avg_x)*pow(pts[ndx].y - avg_y, 2);
      s_dyxx += (pts[ndx].y - avg_y)*pow(pts[ndx].x - avg_x, 2);
    }

    u_c = ((0.5)*(s_dxxx + s_dxyy) - (s_dxy/(2*s_dyy))*(s_dyyy + s_dyxx))/(s_dxx - (pow(s_dxy, 2)/s_dyy));
    v_c = (1/(2*s_dyy))*(s_dyyy + s_dyxx) - (1/s_dyy)*u_c*s_dxy;

    center.x = u_c + avg_x;
    center.y = v_c + avg_y;

    radius = sqrt(pow(u_c, 2) + pow(v_c, 2) + ((s_dxx + s_dyy)/pts.size()));
  }
  Circle::~Circle(){}
  Point Circle::get_center()const{return center;}
  float Circle::get_radius()const{return radius;}
  uint8_t Circle::get_type()const{return type;}
  std::vector<Point> Circle::get_ends()const{ 
    std::vector<Point> null;
    return null;
  }

  Line_seg::Line_seg():length(0), type(LINE){}
  Line_seg::Line_seg(const Point& end1, const Point& end2):end1(end1), end2(end2), type(LINE){
    length = sqrt(pow(end2.x - end1.x, 2) + pow(end2.y - end2.y, 2));
    center = Point((end1.x + end2.x)/2, (end1.y + end2.y)/2);
  }
  Line_seg::Line_seg(const std::vector<Point>& pts): type(LINE){
    float slope, intercept;
    size_t first, last;
    first = 0; last = pts.size() - 1;
    
    DLSR(pts, slope, intercept);
    end1 = return_point(slope, intercept, pts[first]);
    end2 = return_point(slope, intercept, pts[last]);
    length = sqrt(pow(end2.x - end1.x, 2) + pow(end2.y - end2.y, 2));
    center = Point((end1.x + end2.x)/2, (end1.y + end2.y)/2);

  }
  Line_seg::~Line_seg(){}
  Point Line_seg::get_center()const{return center;}
  uint8_t Line_seg::get_type()const{ return type;}
  float Line_seg::get_radius()const{ return 0.0f;}
  std::vector<Point> Line_seg::get_ends()const{
    std::vector<Point> two_points;
    two_points.push_back(end1);
    two_points.push_back(end2);
    return two_points;
  }


  std::ostream& operator<<(std::ostream& os, const Point& rhs){
    os << "(" << rhs.x << ", " << rhs.y << ")";
    return os;
  }

  void Circle::print(std::ostream& os) const {
    os << "Circle:" << std::endl;
    os << "\tCenter: " << center << std::endl;
    os << "\tRadius: " << radius << std::endl;
  }

  void Line_seg::print(std::ostream& os) const {
    os << "Line:" << std::endl;
    os << "\tEnd1: " << end1 << std::endl;
    os << "\tEnd2: " << end2 << std::endl;
    os << "\tLength: " << length << std::endl;
  }
  
  std::ostream& operator<<(std::ostream& os, const Feature& rhs){
    rhs.print(os);
    return os;
  }
  

  std::ostream& operator<<(std::ostream& os, const std::vector<Feature*>& rhs){
    for(size_t ndx = 0; ndx < rhs.size(); ndx++){
      os << *(rhs[ndx]);
    }
    return os;
  }

  bool operator==(const std::vector<Point>& rhs, const std::vector<Point>& lhs){
    if(rhs.size() != lhs.size()){ return false;}
    for(size_t ndx = 0; ndx < rhs.size(); ndx++){
      if(rhs[ndx] != lhs[ndx]){
	return false;
      }
    }
    return true;
  }

  bool operator!=(const std::vector<Point>& rhs, const std::vector<Point>& lhs){
    return !(rhs==lhs);
  }

  Circle circle_regressor(const std::vector<Point>& pts){
    Point center;
    float radius, avg_x, avg_y, u_c, v_c;
    float s_dxx, s_dyy, s_dxy, s_dxxx, s_dyyy, s_dxyy, s_dyxx;
    avg_x = avg_y = 0;
    s_dxx = s_dyy = s_dxy = s_dxxx = s_dyyy = s_dxyy = s_dyxx = 0; //sum of multiplied differences
    
    //Calculate averages
    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      avg_x = (1/ndx)*pts[ndx - 1].x + ((ndx - 1)/ndx)*avg_x;
      avg_y = (1/ndx)*pts[ndx - 1].y + ((ndx - 1)/ndx)*avg_y;
    }

    for(size_t ndx = 0; ndx < pts.size(); ndx++){
      s_dxx += pow(pts[ndx].x - avg_x, 2);
      s_dyy += pow(pts[ndx].y - avg_y, 2);
      s_dxy += (pts[ndx].x - avg_x)*(pts[ndx].y - avg_y);
      s_dxxx += pow(pts[ndx].x - avg_x, 3);
      s_dyyy += pow(pts[ndx].y - avg_y, 3);
      s_dxyy += (pts[ndx].x - avg_x)*pow(pts[ndx].y - avg_y, 2);
      s_dyxx += (pts[ndx].y - avg_y)*pow(pts[ndx].x - avg_x, 2);
    }

    u_c = ((0.5)*(s_dxxx + s_dxyy) - (s_dxy/(2*s_dyy))*(s_dyyy + s_dyxx))/(s_dxx - (pow(s_dxy, 2)/s_dyy));
    v_c = (1/(2*s_dyy))*(s_dyyy + s_dyxx) - (1/s_dyy)*u_c*s_dxy;

    center.x = u_c + avg_x;
    center.y = v_c + avg_y;

    radius = sqrt(pow(u_c, 2) + pow(v_c, 2) + ((s_dxx + s_dyy)/pts.size()));
    return Circle(center, radius);

  }

  void DLSR(const std::vector<Point>& pts, float& slope, float& intercept){
    Point pt_avg;
    float var_xx, var_xy, var_yy;
    var_xx = var_xy = var_yy = 0;

    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      pt_avg.x = (1/ndx)*pts[ndx - 1].x + ((ndx - 1)/ndx)*pt_avg.x;
      pt_avg.y = (1/ndx)*pts[ndx - 1].y + ((ndx - 1)/ndx)*pt_avg.y;
    }
    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      var_xx = (1/ndx)*(pt_avg.x - pts[ndx - 1].x)*(pt_avg.x - pts[ndx - 1].x) + ((ndx - 1)/ndx)*var_xx;
      var_xy = (1/ndx)*(pt_avg.x - pts[ndx - 1].x)*(pt_avg.y - pts[ndx - 1].y) + ((ndx - 1)/ndx)*var_xy;
      var_yy = (1/ndx)*(pt_avg.y - pts[ndx - 1].y)*(pt_avg.y - pts[ndx - 1].y) + ((ndx - 1)/ndx)*var_yy;
    }
    
    if(var_xy == 0){ slope = 0; }
    else{ slope = (var_yy - var_xx + sqrt(pow(var_yy - var_xx, 2) + 4*pow(var_xy, 2)))/(2*var_xy); }
    
    intercept = pt_avg.y - slope*pt_avg.x;
  }

  Point return_point(const float& slope, const float& intercept, const Point& pt_in){
    float x, y;
    x = pt_in.x + (slope/(pow(slope, 2) + 1))*(pt_in.y - intercept - slope*pt_in.x);
    y = slope*x + intercept;
    return Point(x,y);
  }

  void calc_center_radius(const std::vector<Point>& pts, Point& center, float& radius){
    float m21p, m32p, x21, y21, x32, y32;
    
    m21p = (pts[0].x - pts[1].x)/(pts[1].y - pts[0].y);
    m32p = (pts[1].x - pts[2].x)/(pts[2].y - pts[1].y);

    x21 = (pts[1].x - pts[0].x)/2 + pts[0].x;
    y21 = (pts[1].y - pts[0].y)/2 + pts[0].y;
    x32 = (pts[2].x - pts[1].x)/2 + pts[1].x;
    y32 = (pts[2].y - pts[1].y)/2 + pts[1].y;

    center.x = (m21p*x21 - m32p*x32 - y21 + y32)/(m21p - m32p);
    center.y = m21p*(center.x - x21) + y21;
    radius = sqrt(pow(pts[0].x - center.x, 2) + pow(pts[0].y - center.y, 2));

  }

 

  float is_line(const std::vector<Point>& pts, Feature*& ftr_ptr){
    //Variance here is defined as the average squared sum of the orthongonal projections
    //const float VAR_THRESH = 0.5; //cm 
    float slope, intercept, var_orth = 0;
    Point pt;
    
    DLSR(pts, slope, intercept);

    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      pt = return_point(slope, intercept, pts[ndx-1]);
      var_orth = (1/ndx)*(pow((pt.x - pts[ndx-1].x), 2) + pow((pt.y - pts[ndx-1].y), 2)) + ((ndx-1)/ndx)*var_orth;
    }
    
    ftr_ptr = new Line_seg(return_point(slope, intercept, pts.front()), return_point(slope, intercept, pts.back()));
    return var_orth;
  }

  Point return_mean_point(const std::vector<Point>& pts){
    float sum_x, sum_y;
    sum_x = sum_y = 0;
    for(size_t ndx = 0; ndx < pts.size(); ndx++){
      sum_x += pts[ndx].x;
      sum_y += pts[ndx].y;
    }
    return Point(sum_x/pts.size(), sum_y/pts.size());
  }

  float return_distance(const Point& p1, const Point& p2){
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
  }
  
  void k_means(int k, const std::vector<Point>& old_set, std::vector< std::vector<Point> >& clusters){
    //NOTE: The number of elements in the set must be larger than k

    //Generate clusters
    //std::vector<vector<Point> > clusters;
    std::vector<Point> new_means, old_means;
    std::vector<float> distances;
    clusters.resize(k);
    new_means.resize(k);
    old_means.resize(k);		     
    distances.resize(k);

    //Generate initial means via Random Partition
    std::vector<Point> temp_set = old_set;
    //Place points into clusters
    //Ensure each cluster has at least one point
    for(int iter = 0; iter < k; iter++){
      clusters[iter].push_back(temp_set.back());
      temp_set.pop_back();
    }
    //Randomly place the rest
    while(temp_set.size()){
      int r_ndx = rand() % k;
      clusters[r_ndx].push_back(temp_set.back());
      temp_set.pop_back();
    }
    std::cout << "Initial means: " << std::endl;
    //Generate means, reset clusters
    for(int iter = 0; iter < k; iter++){
      new_means[iter] = return_mean_point(clusters[iter]);
      std::cout << new_means[iter];
      clusters[iter].clear();
    }
    std::cout << std::endl;
    
    //Start main loop
    while(old_means != new_means){
      //Clear clusters
      for(int ndx = 0; ndx < k; ndx++){
	clusters[ndx].clear();
      }

      //Reassign points
      temp_set = old_set;
      while(temp_set.size()){
	for(int iter = 0; iter < k; iter++){
	  distances[iter] = return_distance(new_means[iter], temp_set.back());
	}
	std::vector<float>::iterator k_itr = min_element(distances.begin(), distances.end());
	int k_ndx = k_itr - distances.begin();
	clusters[k_ndx].push_back(temp_set.back());
	temp_set.pop_back();
      }

      //Calculate new means
      old_means = new_means;
      new_means.clear();
      for(int ndx = 0; ndx < k; ndx++){
	new_means[ndx] = return_mean_point(clusters[ndx]);
      }
    }
    
    
  }

void k_lines(int k, const std::vector<Point>& old_set, std::vector< std::vector<Point> >& clusters){
  //NOTE: The number of elements in the set must be larger than k

  //Generate clusters
  std::vector<std::pair<float,float> > new_lines, old_lines; //slope, intercept
  std::vector<float> distances;

  clusters.resize(k);
  new_lines.resize(k);
  old_lines.resize(k);		     
  distances.resize(k);
  std::vector<Point> temp_set = old_set;
  /*
  //Generate initial lines via Random Partition
  //Place points into clusters
  //Ensure each cluster has at least one point
  for(int iter = 0; iter < k; iter++){
    if(iter%2){ clusters[iter].push_back(temp_set.front());}
    else{ clusters[iter].push_back(temp_set.back());}
    temp_set.pop_back();
  }
  //Randomly place the rest
  while(temp_set.size()){
    int r_ndx = rand() % k;
    clusters[r_ndx].push_back(temp_set.back());
    temp_set.pop_back();
  }
  */
  /*
  //Generate initial lines via end point iteration round robin
  while(temp_set.size()){
    for(int iter = 0; iter < k; iter++){
      if(temp_set.size()){
	clusters[iter].push_back(temp_set.back());
	temp_set.pop_back();
      }
    }
  }
  */
  //Generate initial clusters via k means
  k_means(k, old_set, clusters);
  std::cout << "Initial lines: " << std::endl;
  //Generate lines, reset clusters
  for(int iter = 0; iter < k; iter++){
    float slope, intercept;
    DLSR(clusters[iter], slope, intercept);
    new_lines[iter] = std::make_pair(slope, intercept);
    //std::cout << new_means[iter];
    clusters[iter].clear();
  }
  std::cout << std::endl;
  
  //Start main loop
  while(old_lines != new_lines){
    //Clear clusters
    for(int ndx = 0; ndx < k; ndx++){
      clusters[ndx].clear();
    }
    
    //Reassign points
    temp_set = old_set;
    while(temp_set.size()){
      for(int iter = 0; iter < k; iter++){
	Point pt_on_line = return_point(new_lines[iter].first, new_lines[iter].second, temp_set.back());
	distances[iter] = return_distance(pt_on_line, temp_set.back());
      }
      std::vector<float>::iterator k_itr = min_element(distances.begin(), distances.end());
      int k_ndx = k_itr - distances.begin();
      clusters[k_ndx].push_back(temp_set.back());
      temp_set.pop_back();
    }
    
    //Calculate new lines
    old_lines = new_lines;
    new_lines.clear();
    for(int ndx = 0; ndx < k; ndx++){
      float slope, intercept;
      DLSR(clusters[ndx], slope, intercept);
      new_lines[ndx] = std::make_pair(slope, intercept);
    }
  }
  
  
}

 float is_multiple_lines(int k, const std::vector<Point>& pts, std::vector<Feature* >& features){
    //Check for multiple lines
   std::vector<std::vector<Point> > clusters;
   std::vector<float> vars;
   float sum_vars = 0;
   Feature* ftr_ptr = NULL;
   features.clear();
   k_lines(2, pts, clusters);
   
   for(size_t ndx = 0; ndx < clusters.size(); ndx++){
     vars.push_back(is_line(clusters[ndx], ftr_ptr));
     features.push_back(ftr_ptr);  
   }
   
   for(size_t ndx = 0; ndx < vars.size(); ndx++){
     sum_vars = vars[ndx];
   }

   return sum_vars;
  }

  Point return_point_circle(const Circle& circle, const Point& pt){
    Point ctr = circle.get_center();
    float slope = (pt.y - ctr.y)/(pt.x - ctr.x);
    float intercept = pt.y - slope*pt.x;

    float a = (1 + pow(slope, 2));
    float b = 2*(slope*intercept - ctr.x - slope*ctr.y);
    float c = (pow(ctr.x, 2) + pow(intercept, 2) - 2*intercept*ctr.y + pow(ctr.y, 2) - pow(circle.get_radius(), 2));
    float d = pow(b, 2) - 4*a*c;
    if(d < 0){return Point();}

    float x1 = (-b + sqrt(d))/(2*a);
    float x2 = (-b - sqrt(d))/(2*a);
    float y1 = slope*x1 + intercept;
    float y2 = slope*x2 + intercept;

    float d1 = return_distance(pt, Point(x1, y1));
    float d2 = return_distance(pt, Point(x2, y2));

    if(d1 < d2){
      return Point(x1, y1);
    }
    return Point(x2, y2);

  }

  float is_circle(const std::vector<Point>& pts, Feature*& ftr_ptr){
    Point pt;
    //const float RADIUS_THRESH = 60.0; //cm
    //const float VAR_THRESH = 1.5; //cm
    float var_orth = 0;
    Circle c1 = circle_regressor(pts);

    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      pt = return_point_circle(c1, pts[ndx-1]);
      var_orth = (1/ndx)*(pow((pt.x - pts[ndx-1].x), 2) + pow((pt.y - pts[ndx-1].y), 2)) + ((ndx-1)/ndx)*var_orth;
    }
    
    ftr_ptr = new Circle(c1.get_center(), c1.get_radius());
    return var_orth;
  }

  /*
    bool is_circle(const std::vector<Point>& pts, Feature*& ftr_ptr){
    Point pt;
    const float RADIUS_THRESH = 60.0; //cm
    const float VAR_THRESH = 1.5; //cm
    float var_orth = 0;
    Circle c1 = circle_regressor(pts);

    for(float ndx = 1; ndx < (pts.size() + 1); ndx++){
      pt = return_point_circle(c1, pts[ndx-1]);
      var_orth = (1/ndx)*(pow((pt.x - pts[ndx-1].x), 2) + pow((pt.y - pts[ndx-1].y), 2)) + ((ndx-1)/ndx)*var_orth;
    }
    
    if((c1.get_radius() < RADIUS_THRESH) && (var_orth < VAR_THRESH)){ 
      ftr_ptr = new Circle(c1.get_center(), c1.get_radius());
      return true; 
    }
    else{ return false; }
  }
   */
  /*
  bool is_circle(const std::vector<Point>& pts){
    const float RADIUS_THRESH = 60.0; //cm
    const float VAR_THRESH = 9.0; //cm
    std::vector<Point> three_pts;
    Point center_est;
    float radius_est, radius_avg, radius_var;

    //Calculate average radius and variance
    for(float ndx = 1; ndx < (pts.size() - 1); ndx++){
      three_pts.clear();
      three_pts.push_back(pts[ndx - 1]);
      three_pts.push_back(pts[ndx]);
      three_pts.push_back(pts[ndx + 1]);

      calc_center_radius(three_pts, center_est, radius_est);
      
      radius_avg = (1/ndx)*radius_est + ((ndx - 1)/ndx)*radius_avg;
      radius_var = (1/ndx)*pow((radius_avg - radius_est), 2) + ((ndx-1)/ndx)*radius_var;
    }
    if((radius_avg < RADIUS_THRESH) && (radius_var < VAR_THRESH)){ return true; }
    else{ return false; }
  }
  */

  void parse_scan(const sensor_msgs::LaserScan& scan, std::vector< std::vector<Point> >& vv_pts){
    const float SEP_THRESH = 0.125; // meters
    const int MIN_NUM_OF_RANGES = 3; //Minimum number of ranges needed to produce points
    std::vector<RT> vect_ranges;
    std::vector<Point> vect_points;
    
    for(size_t ndx = 0; ndx < (scan.ranges.size() - 1); ndx++){
      //std::cout << "Range difference: " << abs(scan.ranges[ndx] - scan.ranges[ndx + 1]) << std::endl;
      // If the ranges are similar to each other then they are candidates
      if((fabs(scan.ranges[ndx] - scan.ranges[ndx + 1]) < SEP_THRESH)
	 && (scan.ranges[ndx] > scan.range_min) && (scan.ranges[ndx] < scan.range_max))
	{
	  // If this is the first time anything has been added, add both
	  if(vect_ranges.size() == 0){
	    vect_ranges.push_back(RT(scan.ranges[ndx], scan.angle_increment*ndx + scan.angle_min));
	  }
	  vect_ranges.push_back(RT(scan.ranges[ndx + 1], scan.angle_increment*ndx + scan.angle_min));
	  
	  //std::cout << "Pushed range: " << scan.ranges[ndx + 1] << std::endl;
	}
      // If we are not adding ranges and there are more than two candidate ranges,
      // then add them to the points vector
      else if(vect_ranges.size() >= MIN_NUM_OF_RANGES){
	for(size_t ndx = 0; ndx < vect_ranges.size(); ndx++){
	  float x = vect_ranges[ndx].radius*cos(vect_ranges[ndx].theta);
	  float y = vect_ranges[ndx].radius*sin(vect_ranges[ndx].theta);
	  //Push the point in cm
	  vect_points.push_back(Point(x*100,y*100));
	}
	vv_pts.push_back(vect_points);
	vect_points.clear();
	vect_ranges.clear();
      }
      // If we are not adding ranges, and the current range vector contains too few candidates
      else{
	vect_ranges.clear();
      }
      
      //Check the special case of an object being split by the beginning and end of the scan
      if(ndx == (scan.ranges.size() - 2)){
	if((fabs(scan.ranges[scan.ranges.back()] - scan.ranges[scan.ranges.front()]) < SEP_THRESH)
	   && (scan.ranges[scan.ranges.back()] > scan.range_min) && (scan.ranges.back() < scan.range_max))
	  {
	    vect_ranges.push_back(RT(scan.ranges[ndx + 1], scan.angle_increment*ndx + scan.angle_min));
	    for(size_t iter = 1; iter < (scan.ranges.size() - 1); iter++){
	      if((fabs(scan.ranges[iter] - scan.ranges[iter + 1]) < SEP_THRESH)
		 && (scan.ranges[iter] > scan.range_min))
		{
		  // If this is the first time anything has been added, add both
		  if(vect_ranges.size() == 0){
		    vect_ranges.push_back(RT(scan.ranges[iter], scan.angle_increment*iter + scan.angle_min));
		  }
		  vect_ranges.push_back(RT(scan.ranges[iter + 1], scan.angle_increment*iter + scan.angle_min));
		  
		  //std::cout << "Pushed range: " << scan.ranges[ndx + 1] << std::endl;
		}
	      // If we are not adding ranges and there are more than two candidate ranges,
	      // then add them to the points vector
	      else if(vect_ranges.size() >= MIN_NUM_OF_RANGES){
		for(size_t ndx = 0; ndx < vect_ranges.size(); ndx++){
		  float x = vect_ranges[ndx].radius*cos(vect_ranges[ndx].theta);
		  float y = vect_ranges[ndx].radius*sin(vect_ranges[ndx].theta);
		  //Push the point in cm
		  vect_points.push_back(Point(x*100,y*100));
		}
		vv_pts.push_back(vect_points);
		vect_points.clear();
		vect_ranges.clear();
	      }
	      // If we are not adding ranges, and the current range vector contains too few candidates
	      else{
		break;
	      }
	    }
	  }
      } 
    }
    
  }

  bool comp_pair_first(std::pair<float, std::vector<Feature*> > p1, std::pair<float, std::vector<Feature*> > p2){
    return p1.first < p2.first;
  }
  /**/
  void produce_feature(std::vector<Point> pts, std::vector<Feature*>& features){
    const float VAR_THRESH = 0.2;
    Feature* ftr_ptr = NULL;
    //This vector holds the variances of the points to their associated feature(s)
    std::vector<std::pair<float, std::vector<Feature*> > > vars_ftrs;
    std::pair<float, std::vector<Feature*> > temp_pair;
    features.clear();

    temp_pair.first = is_line(pts, ftr_ptr);
    temp_pair.second.push_back(ftr_ptr);
    vars_ftrs.push_back(temp_pair);

    temp_pair.first = is_circle(pts, ftr_ptr);
    temp_pair.second.push_back(ftr_ptr);
    vars_ftrs.push_back(temp_pair);

    temp_pair.first = is_multiple_lines(2, pts, temp_pair.second);
    vars_ftrs.push_back(temp_pair);
    
    std::vector<std::pair<float, std::vector<Feature*> > >::iterator min_ftr;
    min_ftr = min_element(vars_ftrs.begin(), vars_ftrs.end(), comp_pair_first);
    if(min_ftr->first < VAR_THRESH){
      features = min_ftr->second;
    }
  }

  /*
  void produce_feature(std::vector<Point> pts, std::vector<Feature*>& features){
    features.clear();
    Feature* ftr_ptr = NULL;
    if(is_line(pts, ftr_ptr)){
      features.push_back(ftr_ptr);
    }
    
    else if(is_circle(pts, ftr_ptr)){
      features.push_back(ftr_ptr);
    }
    else if(is_multiple_lines(2, pts, features)){

    }
  }
  */ 
  /*
  Feature* produce_feature(std::vector<Point> pts){
    Feature* ftr_ptr = NULL;
    if(is_line(pts, ftr_ptr)){
      //return new Line_seg(pts);
      //return ftr_ptr;
    }
    else if(is_circle(pts, ftr_ptr)){
      //return new Circle(pts);
      //return ftr_ptr;
    }
    return ftr_ptr;
  }
  */

  void produce_collection(const sensor_msgs::LaserScan& scan, std::vector<Feature*>& landmarks){
    std::vector< std::vector<Point> > vv_pts;
    std::vector<Feature*> temp_features;
    
    parse_scan(scan, vv_pts);
    
    std::cout << "Number of Candidates: " << vv_pts.size() << std::endl;

    for(size_t ndx = 0; ndx < vv_pts.size(); ndx++){
      produce_feature(vv_pts[ndx], temp_features);
      if(temp_features.size()){
	for(size_t iter = 0; iter < temp_features.size(); iter++){
	  landmarks.push_back(temp_features[iter]);
	}
      }
      /*
      else{
	int k = 2;
	std::vector< std::vector<Point> > clusters;
	k_lines(k, vv_pts[ndx], clusters);
	for(size_t iter = 0; iter < clusters.size(); iter++){
	  temp_feature = produce_feature(clusters[iter]);
	  if(temp_feature){
	    landmarks.push_back(temp_feature);
	  }
	}
       
      }
      */
      std::cout << "Vector[" << ndx << "]:" << std::endl;
      for(size_t iter = 0; iter < vv_pts[ndx].size(); iter++){
	std::cout << vv_pts[ndx][iter] << std::endl;
      }
      /*
      for(size_t ndx = 0; ndx < k; ndx++){
	std::cout << "Cluster: " << ndx << std::endl;
	std::cout << "Size: " << clusters[ndx].size();
	for(int iter = 0; iter < clusters[ndx].size(); iter++){
	  std::cout << clusters[ndx][iter];
	}
	std::cout << std::endl;
      }
      */
      std::cout << std::endl;
    }
  }

  bool comp_pair_second(std::pair<int, int > p1, std::pair<int, int > p2){
    return p1.second < p2.second;
  }

  void produce_collection(const sensor_msgs::LaserScan& scan, std::vector<std::vector<Feature*> >& prev_land, std::vector<Feature*>& landmarks){
    std::vector< std::vector<Point> > vv_pts;
    std::vector<Feature*> temp_features;
    const float DIST_THRESH 6.0; //cm
    
    parse_scan(scan, vv_pts);
    
    std::cout << "Number of Candidates: " << vv_pts.size() << std::endl;

    for(size_t ndx = 0; ndx < vv_pts.size(); ndx++){
      produce_feature(vv_pts[ndx], temp_features);
      if(temp_features.size()){
	for(size_t iter = 0; iter < temp_features.size(); iter++){
	  landmarks.push_back(temp_features[iter]);
	}
      }

     
     
      std::cout << "Vector[" << ndx << "]:" << std::endl;
      for(size_t iter = 0; iter < vv_pts[ndx].size(); iter++){
	std::cout << vv_pts[ndx][iter] << std::endl;
      }
      std::cout << std::endl;
    }
    
    // Take the majority vote of the landmark type from current and previous hypothesis
    // The last element of each vector is the oldest estimate

    // Associate the new hypothesis with the old ones
    std::pair<size_t, float> closest_landmark;
    closest_landmark.second = FLT_MAX;
    for(size_t ndx = 0; ndx < landmarks.size(); ndx++){
      for(size_t iter = 0; iter < prev_land.size(); iter++){
	float temp_distance = return_distance(landmarks[ndx]->get_center(), prev_land[iter][0]->get_center());
	if((temp_distance < closest_landmark.second) && (temp_distance < DIST_THRESH)){
	  closest_landmark.first = iter;
	  closest_landmark.second = temp_distance;
	}
      }
      prev_land[closest_landmark.first].push_back(landmark[ndx]);
    }
    
    // Find the most popular object in each set and instantiate the output
    // landmark set with that object type
    landmark.clear();
    std::vector<pair<int, int> > type_counts;
    type_counts.push_back(pair<int, int>(CIRCLE, 0));
    type_counts.push_back(pair<int, int>(LINE, 0));

    for(size_t ndx = 0; ndx < prev_land.size(); ndx++){
      for(size_t iter = 0; iter < prev_land[ndx].size(); iter++){
	for(size_t type_iter = 0; type_iter < type_counts.size(); type_iter++){
	  if(prev_land[ndx][iter]->get_type() == type_counts[type_iter].first){
	    type_counts[type_iter].second++;
	    break;
	  }  
	}
      
	std::vector<pair<int, int> >::iterator max_type;
	max_type = max_element(type_counts.begin(), type_counts.end(), comp_pair_second);
      
	landmark.push_back();
      }
    }
    
  }

  void produce_cluster_points(const sensor_msgs::LaserScan& scan, std::vector<std::vector<Point> >& point_clusters){
    std::vector< std::vector<Point> > vv_pts;
    
    
    parse_scan(scan, vv_pts);
    
    std::cout << "Number of Candidates: " << vv_pts.size() << std::endl;
    point_clusters.clear();
    
    for(size_t ndx = 0; ndx < vv_pts.size(); ndx++){
      int k = 2;
      std::vector< std::vector<Point> > clusters;
      //k_means(k, vv_pts[ndx], clusters);
      k_lines(k, vv_pts[ndx], clusters);
      
      std::cout << "Vector[" << ndx << "]:" << std::endl;
      for(size_t iter = 0; iter < vv_pts[ndx].size(); iter++){
	std::cout << vv_pts[ndx][iter] << std::endl;
      }
      for(size_t ndx = 0; ndx < k; ndx++){
	std::cout << "Cluster: " << ndx << std::endl;
	std::cout << "Size: " << clusters[ndx].size();
	point_clusters.push_back(clusters[ndx]);
	for(int iter = 0; iter < clusters[ndx].size(); iter++){
	  std::cout << clusters[ndx][iter];
	}
	std::cout << std::endl;
      }
      std::cout << std::endl;
    }
  }
  /*
  void produce_collection_from_points(std::vector<std::vector<Point> >& point_sets, std::vector<Feature*>& landmarks){
    Feature* temp_feature;
    
    std::cout << "Number of Candidates: " << point_sets.size() << std::endl;

    for(size_t ndx = 0; ndx < point_sets.size(); ndx++){
      temp_feature = produce_feature(point_sets[ndx]);
      if(temp_feature){
	landmarks.push_back(temp_feature);
      }
      
      std::cout << "Vector[" << ndx << "]:" << std::endl;
      for(size_t iter = 0; iter < point_sets[ndx].size(); iter++){
	std::cout << point_sets[ndx][iter] << std::endl;
      }
      
      std::cout << std::endl;
    }
  }
  */



}// End of namespace




