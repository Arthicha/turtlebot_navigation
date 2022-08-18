#include<simple_layers/simple_layer.h>  
#include <pluginlib/class_list_macros.h>  
#include <iostream>
using namespace std;
  
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)//Register plug-in
  
using costmap_2d::LETHAL_OBSTACLE;  

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx),
         f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var)
{
  return sqrt(-2 * var * log(cutoff / A));
}

  
namespace simple_layer_namespace  
{  
  
SimpleLayer::SimpleLayer() {}  
  
void SimpleLayer::onInitialize()  
{  
  ros::NodeHandle nh("~/" + name_);
  
  current_ = true;
  
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
  
      &SimpleLayer::reconfigureCB, this, _1, _2);
  
  dsrv_->setCallback(cb);

  
}  
  
  
void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)  
{  
  enabled_ = config.enabled;
  
}  
  
void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,  
                                           double* min_y, double* max_x, double* max_y)
  
{  
  if (!enabled_)
  
    return;
  
  
  mark_x_ = origin_x + cos(origin_yaw);
  
  mark_y_ = origin_y + sin(origin_yaw);
  
  
  *min_x = std::min(*min_x, mark_x_);
  
  *min_y = std::min(*min_y, mark_y_);
  
  *max_x = std::max(*max_x, mark_x_);
  
  *max_y = std::max(*max_y, mark_y_);

  
}  
  
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,  
                                          int max_j)
  
{ 
  //costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  //cout << "Welcome to GFG\t" << costmap << endl; 
  

  /*if (!enabled_)
  
    return;
  
  unsigned int mx;
  
  unsigned int my;

  
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    
    master_grid.setCost(mx, my, 200);//LETHAL_OBSTACLE);
  
  }*/
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  double cx = mark_x_, cy = mark_y_;

  double angle = atan2(cy, cx);
    double factor = 1.0 ;
    double base = get_radius(cutoff_, amplitude_, covar_);
    double point = get_radius(cutoff_, amplitude_, covar_ * factor);

    unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                 height = std::max(1, static_cast<int>((base + point) / res));

    double ox, oy;
    if (sin(angle) > 0)
      oy = cy - base;
    else
      oy = cy + (point - base) * sin(angle) - base;

    if (cos(angle) >= 0)
      ox = cx - base;
    else
      ox = cx + (point - base) * cos(angle) - base;


    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2,
           by = oy + res / 2;
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        //unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        /*if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        double x = bx + i * res, y = by + j * res;
        double ma = atan2(y - cy, x - cx);
        double a;
        a = 0.0;//gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);

        if (a < cutoff_)
          continue;
        unsigned char cvalue = (unsigned char) a;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));*/
      }
    }
  
}  
  
} //end namespace