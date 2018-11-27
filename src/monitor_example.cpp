#include "uwds_qsr_clients/monitor_example.h"

namespace uwds_qsr_clients
{
  void MonitorExample::onInit()
  {
    // ros param init comes here
    uwds::ReconfigurableClient::onInit();
  }

  void MonitorExample::onChanges(const std::string& world,
                         const std_msgs::Header& header,
                         const Invalidations& invalidations)
  {
    // Here goes the code that is executed on every changes
  }

  void MonitorExample::onReconfigure(const std::vector<std::string>& new_input_worlds)
  {
    // Here goes the code that is executed on each reconfiguration
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_qsr_clients::MonitorExample, nodelet::Nodelet)
