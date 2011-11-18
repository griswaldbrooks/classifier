#ifndef CLASSIFIER
#define CLASSIFIER

#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <deque>
#include <math.h>
#include <iostream>
#include <float.h>

#define CIRCLE 1
#define LINE   2
#define NO_ASSOC -1

namespace classifier{
  
  class RT{
  public:
    RT();
    RT(const float& radius, const float& theta);
    float radius, theta;
  };

  class Point{
    friend std::ostream& operator<<(std::ostream& os, const Point& rhs);
  public:
    Point();
    Point(const float& x, const float& y);
    bool operator!=(const Point& rhs) const;
    bool operator==(const Point& rhs) const;
    float x, y;
  };

  class Feature{
    friend std::ostream& operator<<(std::ostream& os, const Feature& rhs);
  public:
    virtual Point get_center() const = 0;
    virtual uint8_t get_type() const = 0;
    virtual float get_radius() const = 0;
    virtual std::vector<Point> get_ends() const = 0;
  private:
    virtual void print(std::ostream& os) const = 0;
    uint8_t type;
  };

  class Circle : public Feature{
    friend std::ostream& operator<<(std::ostream& os, const Circle& rhs);
  public:
    Circle();
    Circle(const Point& center, const float& radius);
    Circle(const std::vector<Point>& pts);
    ~Circle();
    Point get_center() const;
    float get_radius() const;
    uint8_t get_type() const;
    std::vector<Point> get_ends() const;
  private: 
    void print(std::ostream& os) const;
   
    float radius;
    Point center;
    uint8_t type;
  };

  class Line_seg : public Feature{
    friend std::ostream& operator<<(std::ostream& os, const Line_seg& rhs);
  public:
    Line_seg();
    Line_seg(const Point& end1, const Point& end2);
    Line_seg(const std::vector<Point>& pts);
    ~Line_seg();
    Point get_center() const;
    uint8_t get_type() const;
    float get_radius() const;
    std::vector<Point> get_ends() const;
  private:
    void print(std::ostream& os) const;

    Point center, end1, end2;
    float length;
    uint8_t type;
  };

  class Collection{
  public:
    Collection();
    void produce_collection(const sensor_msgs::LaserScan& scan, std::vector<Feature*>& landmarks);
    Feature* ret_ftr_ptr(unsigned int ftr_ndx, unsigned int t_ndx);
   private:
    //This vector contains pairs, representing previous landmarks. The landmarks are represented by pairs containing
    //a vector of previous Feature hypotheses and their associated vector of Points
    std::vector<std::pair<std::deque<Feature*>, std::deque<std::vector<Point> > > > prev_land;
  };

  std::ostream& operator<<(std::ostream& os, const std::vector<Feature*>& rhs);
  bool operator==(const std::vector<Point>& rhs, const std::vector<Point>& lhs);
  bool operator!=(const std::vector<Point>& rhs, const std::vector<Point>& lhs);

  Circle circle_regressor(const std::vector<Point>& pts);
  void DLSR(const std::vector<Point>& pts, float& slope, float& intercept);
  Point return_point(const float& slope, const float& intercept, const Point& pt_in);
  Point return_point_circle(const Circle& circle, const Point& pt);
  Point return_mean_point(const std::vector<Point>& pts);
  float return_distance(const Point& p1, const Point& p2);
  void calc_center_radius(const std::vector<Point>& pts, Point& center, float& radius);
  
  float is_line(const std::vector<Point>& pts, Feature*& ftr_ptr);
  float is_circle(const std::vector<Point>& pts, Feature*& ftr_ptr);
  float is_multiple_lines(int k, const std::vector<Point>& pts, std::vector<Feature* >& features);

  void k_means(int k, const std::vector<Point>& old_set, std::vector< std::vector<Point> >& clusters);
  void k_lines(int k, const std::vector<Point>& old_set, std::vector< std::vector<Point> >& clusters);
  
  void parse_scan(const sensor_msgs::LaserScan& scan, std::vector< std::vector<Point> >& vv_pts);
  void produce_feature(std::vector<Point> pts, std::vector<Feature*>& features);
  bool comp_pair_first(std::pair<float, std::vector<Feature*> > p1, std::pair<float, std::vector<Feature*> > p2);
  bool comp_pair_second(std::pair<int, int > p1, std::pair<int, int > p2);
  //Feature* produce_feature(std::vector<Point> pts);
  void produce_collection(const sensor_msgs::LaserScan& scan, std::vector<Feature*>& landmarks);
  
  void produce_cluster_points(const sensor_msgs::LaserScan& scan, std::vector<std::vector<Point> >& point_clusters);
  //void produce_collection_from_points(std::vector<std::vector<Point> >& point_sets, std::vector<Feature*>& landmarks);
}

#endif
