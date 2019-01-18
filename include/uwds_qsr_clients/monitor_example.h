#ifndef MONITOR_EXAMPLE_HPP
#define MONITOR_EXAMPLE_HPP

#include <uwds/uwds.h>
#include <uwds/reconfigurable_client.h>

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_qsr_clients
{
  class MonitorExample : public uwds::ReconfigurableClient
  {
  public:
    /**@brief
     * The default constructor.
     */
    MonitorExample(): uwds::ReconfigurableClient(uwds::FILTER) {}

    /**@brief
     * The default destructor.
     */
    ~MonitorExample() = default;

    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  protected:

    /** @brief
     * This method is called when there is a change in a world.
     *
     * @param world The world that have been updated
     * @param header The header
     * @param invalidations The invalidations received
     */
    virtual void onChanges(const std::string& world,
                           const std_msgs::Header& header,
                           const Invalidations& invalidations);

    virtual void onSubscribeChanges(const std::string world) {}

    virtual void onUnsubscribeChanges(const std::string world) {}

    virtual void onReconfigure(const std::vector<std::string>& worlds);

  };
}

#endif
