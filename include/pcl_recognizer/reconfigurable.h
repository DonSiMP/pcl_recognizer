#ifndef PCL_RECOGNIZER_RECONFIGURABLE_H
#define PCL_RECOGNIZER_RECONFIGURABLE_H

#include <dynamic_reconfigure/server.h>

template <typename ConfigType>
class Reconfigurable
{
public:
  Reconfigurable(std::string name);

protected:
  ConfigType cfg_;

private:
  dynamic_reconfigure::Server<ConfigType> cfg_srv_;
  void cfg_cb(ConfigType &config, uint32_t level) { cfg_ = config; }
};

template <typename ConfigType>
Reconfigurable<ConfigType>::Reconfigurable(std::string name) : cfg_srv_(name)
{
  typename dynamic_reconfigure::Server<ConfigType>::CallbackType cfg_srv_cb;
  cfg_srv_cb = boost::bind(&Reconfigurable<ConfigType>::cfg_cb, this, _1, _2);
  cfg_srv_.setCallback(cfg_srv_cb);
}

#endif //PCL_RECOGNIZER_RECONFIGURABLE_H
