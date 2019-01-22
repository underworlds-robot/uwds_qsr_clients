#include "uwds_qsr_clients/monitor_example.h"

namespace uwds_qsr_clients
{
  void MonitorExample::onInit()
  {
    // ros param init comes here (thresholds etc.)
    uwds::ReconfigurableClient::onInit();
    // Not after due to default configuration that happend in the init ;)
  }

  void MonitorExample::onChanges(const std::string& world,
                         const std_msgs::Header& header,
                         const Invalidations& invalidations)
  {
    // Here goes the code that is executed on every changes
    for (const auto& subject_id : invalidations.node_ids_updated)
    {
      Node subject = worlds()[world].scene().nodes()[subject_id];
      if(subject.type == MESH)
      {
        for (const auto& object_ptr : worlds()[world].scene().nodes())
        {
          Node object = *object_ptr;
          if(object.type == MESH)
          {
            // Evaluate subject and object relations
            // <--- :P
            //
          }
        }
      }
    }
  }

  void MonitorExample::onReconfigure(const std::vector<std::string>& new_input_worlds)
  {
    // Here goes the code that is executed on each reconfiguration
    // reset the qsr traces in the server ?
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_qsr_clients::MonitorExample, nodelet::Nodelet)
