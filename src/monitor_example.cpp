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
    for (const auto& subject_id : invalidations.nodes_id_updated)
    {
      subject = worlds()[world].scene().nodes()[subject_id];
      if(subject.type == MESH)
      {
        // Iterate over the content of the map so object and subject are the same type ;)
        for (const auto& object : worlds()[world].scene().nodes())
        {
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
