#ifndef CAMERA_SETTINGS_H
#define CAMERA_SETTINGS_H

#include <iostream>
#include <string>

class CameraSetting
{
public:
  explicit CameraSetting()
  : m_camera_name{"camera_name"},
    m_camera_sn{"camera_sn"},
    m_framerate{10},
    m_work_mode{0},
    m_color_resolution{0},
    m_xdr_mode{0},
    m_depth_publish{true},
    m_ir_publish{false},
    m_color_publish{false},
    m_transformed_color{false},
    m_transformed_depth{false},
    m_depth_cloud_point{false},
    m_depth2color_cloud_point{false}
  {

  }
  explicit CameraSetting(
    const std::string & camera_name, const std::string & camera_sn,
    int framerate, int work_mode, int color_resolution, int xdr_mode, bool depth_publish, bool ir_publish, bool color_publish,
    bool transformed_color, bool transformed_depth, bool depth_cloud_point, bool depth2color_cloud_point)
    : m_camera_name{camera_name},
    m_camera_sn{camera_sn},
    m_framerate{framerate},
    m_work_mode{work_mode},
    m_color_resolution{color_resolution},
    m_xdr_mode{xdr_mode},
    m_depth_publish{depth_publish},
    m_ir_publish{ir_publish},
    m_color_publish{color_publish},
    m_transformed_color{transformed_color},
    m_transformed_depth{transformed_depth},
    m_depth_cloud_point{depth_cloud_point},
    m_depth2color_cloud_point{depth2color_cloud_point}
  {
    std::cout << " Camera Setting Display " << std::endl << " camera_name: " << m_camera_name
              << " camera_sn: " << m_camera_sn << " work_mode: " << m_work_mode << " color_resolution: " << m_color_resolution
              << " xdr_mode:" << xdr_mode << " framerate:" << m_framerate 
              << " depth_publish: " << m_depth_publish << " ir_publish: " << m_ir_publish  << " color_publish: " << m_color_publish
              << " transformed_color: " << m_transformed_color << " transformed_depth: " << m_transformed_depth  << " depth_cloud_point " << m_depth_cloud_point
              << " depth2color_cloud_point: " << m_depth2color_cloud_point << std::endl;
  }

  std::string get_camera_name() { return m_camera_name; }
  std::string get_camera_sn() { return m_camera_sn; }

  int get_framerate() { return m_framerate; }
  void set_framerate(int framerate) { m_framerate = framerate;}

  int get_workmode() { return m_work_mode; }
  void set_workmode(int work_mode) { m_work_mode = work_mode;}

  int get_color_resolution() { return m_color_resolution; }
  void set_color_resolution(int color_resolution) { m_color_resolution = color_resolution;}

  int get_xdrmode() { return m_xdr_mode; }
  void set_xdrmode(int xdr_mode) { m_xdr_mode = xdr_mode;}

  bool get_depth_publish() { return m_depth_publish;}
  void set_depth_publish(bool depth_publish) { m_depth_publish = depth_publish; }

  bool get_color_publish() { return m_color_publish;}
  void set_color_publish(bool color_publish) { m_color_publish = color_publish; }

  bool get_ir_publish() { return m_ir_publish;}
  void set_ir_publish(bool ir_publish) { m_ir_publish = ir_publish; }

  bool get_transformed_color_publish() { return m_transformed_color;}
  void set_transformed_color_publish(bool transformed_color) { m_transformed_color = transformed_color; }

  bool get_transformed_depth_publish() { return m_transformed_depth;}
  void set_transformed_depth_publish(bool transformed_depth) { m_transformed_depth = transformed_depth; }

  bool get_depth_cloud_point() { return m_depth_cloud_point;}
  void set_depth_cloud_point(bool depth_cloud_point) { m_depth_cloud_point = depth_cloud_point; }

  bool get_depth2color_cloud_point() { return m_depth2color_cloud_point;}
  void set_depth2color_cloud_point(bool depth2color_cloud_point) { m_depth2color_cloud_point = depth2color_cloud_point; }

private:
  std::string m_camera_name;
  std::string m_camera_sn;
  int m_framerate;
  int m_work_mode;
  int m_color_resolution;
  int m_xdr_mode;
  bool m_depth_publish;
  bool m_ir_publish;
  bool m_color_publish;
  bool m_transformed_color;
  bool m_transformed_depth;
  bool m_depth_cloud_point;
  bool m_depth2color_cloud_point;
};

#endif  // BUILD_CAMERA_SETTINGS_H
