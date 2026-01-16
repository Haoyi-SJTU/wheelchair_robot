#include <robot_base_navigation/simple_layer.h>  
#include <pluginlib/class_list_macros.h>  
  
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)  
  
using costmap_2d::LETHAL_OBSTACLE;  
  
namespace simple_layer_namespace  
{  
  
SimpleLayer::SimpleLayer() 
{
  /*
  origin_x_ = 1.0;    //0.85
  origin_y_ = 0.6;    //0.45
  length_x_ = 1.23;    //1.53
  length_y_ = 2.6;     //2.8
  iterator_ = 0.05;
  */
  origin_x_ = 2.14;    //2.03   2.28
  origin_y_ = 0.59;    //0.35   0.47
  length_x_ = 2.4;    //2.6
  length_y_ = 1.2;     //1.38
  iterator_ = 0.02;

}  
  
void SimpleLayer::onInitialize()  
{  
  ros::NodeHandle nh("~/" + name_);  
  current_ = true;  
  std::cout << "simple layer has been set" << std::endl;

  
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
  
  *min_x = std::min(*min_x, origin_x_);  
  *min_y = std::min(*min_y, origin_y_);  
  *max_x = std::max(*max_x, origin_x_ + length_x_);  
  *max_y = std::max(*max_y, origin_y_ + length_y_);  
}  
  
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,  
                                          int max_j)  
{  
  if (!enabled_)  
    return;  

  unsigned int mx,my;

  for(double i = origin_x_; i < origin_x_ + length_x_ ; i = i+iterator_)
  {
    for(double j = origin_y_; j < origin_y_ + length_y_ ; j = j+iterator_)
    {
      if(master_grid.worldToMap(i, j, mx, my))
      {  
        master_grid.setCost(mx, my, 254);  
      } 
    }
  }

}  
  
} // end namespace  
