#include <pcl_recognizer/config.h>

Config::Config()
{
  dynamic_reconfigure::Server<pcl_recognizer::RecognizerConfig>::CallbackType params_server_cb;
  params_server_cb = boost::bind(&Config::params_cb, this, _1, _2);
  server.setCallback(params_server_cb);
}

pcl_recognizer::RecognizerConfig& Config::get(){
  static Config cfg;
  return cfg.cfg;
}

void Config::params_cb(pcl_recognizer::RecognizerConfig &config, uint32_t level)
{
  if (config.recalculate)
    cfg = config;
}