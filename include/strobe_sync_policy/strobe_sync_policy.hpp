#pragma once

#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/synchronizer.h"

#include <ros/assert.h>
#include <ros/message_event.h>
#include <ros/message_traits.h>
#include <boost/mpl/at.hpp>

#include <mutex>
#include <tuple>
#include <vector>

namespace message_filters::evo_sync_policies
{
  namespace mpl = boost::mpl;
  namespace mt = ros::message_traits;

  /**
   * @brief A message synchronization policy that aligns messages based on their timestamps with configurable time shifts.
   *
   * The StrobeSync policy synchronizes incoming messages from multiple topics by aligning their timestamps
   * after applying configurable time shifts. It maintains buffers of recent messages for each topic and
   * attempts to find sets of messages whose shifted timestamps fall within a specified tolerance.
   *
   * Synchronization Logic:
   * 1. When a message is received, its timestamp is adjusted by the configured shift
   * 2. The policy attempts to find messages in other buffers with matching shifted timestamps
   * 3. If a complete set is found (one from each buffer with aligned timestamps), they are emitted
   * 4. Older messages beyond the max delay are periodically purged   *
   */
  template <typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
            typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
  struct StrobeSync : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
  {
    typedef Synchronizer<StrobeSync> Sync;
    typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
    typedef typename Super::Messages Messages;
    typedef typename Super::Signal Signal;
    typedef typename Super::Events Events;
    typedef typename Super::RealTypeCount RealTypeCount;
    typedef typename Super::M0Event M0Event;
    typedef typename Super::M1Event M1Event;
    typedef typename Super::M2Event M2Event;
    typedef typename Super::M3Event M3Event;
    typedef typename Super::M4Event M4Event;
    typedef typename Super::M5Event M5Event;
    typedef typename Super::M6Event M6Event;
    typedef typename Super::M7Event M7Event;
    typedef typename Super::M8Event M8Event;

    typedef std::tuple<std::vector<M0Event>, std::vector<M1Event>, std::vector<M2Event>, std::vector<M3Event>,
                       std::vector<M4Event>, std::vector<M5Event>, std::vector<M6Event>, std::vector<M7Event>,
                       std::vector<M8Event>>
        MsgsEventsBuffer;

    typedef std::tuple<M0Event, M1Event, M2Event, M3Event, M4Event, M5Event, M6Event, M7Event, M8Event> SyncedBuffer;

    /**
     * @brief Construct a new StrobeSync policy
     *
     * @param shifts Array of time shifts to apply to each topic's timestamp.
     *               The array size must match the number of message types being synchronized.
     *               Shift values are added to message timestamps.
     * @param max_delay Maximum time window to keep messages in buffers before discarding.
     * @param max_aligned_time_eps Tolerance for considering timestamps to be aligned.
     */
    StrobeSync(const std::array<double, RealTypeCount::value> &shifts, const double max_delay,
               const double max_aligned_time_eps = 1e-1)
        : shifts_(shifts), max_delay_(max_delay), max_aligned_time_eps_(max_aligned_time_eps)
    {
    }

    StrobeSync(const StrobeSync &e) { *this = e; }
    StrobeSync &operator=(const StrobeSync &rhs)
    {
      parent_ = rhs.parent_;
      buffer_ = rhs.buffer_;
      shifts_ = rhs.shifts_;
      max_delay_ = rhs.max_delay_;
      max_aligned_time_eps_ = rhs.max_aligned_time_eps_;

      return *this;
    }

    /**
     * @brief Add a message to the synchronizer
     *
     * @tparam i The index of the message type (0-8)
     * @param evt The message event to add
     */
    template <int i>
    void add(const typename mpl::at_c<Events, i>::type &evt)
    {
      std::scoped_lock lock(buffer_access_mutex_);
      ROS_ASSERT(parent_);

      const auto recv_msgs_time = get_time_from_msg_sec<i>(evt);
      fit_buffer_size_<i>(recv_msgs_time);

      std::get<i>(buffer_).push_back(evt);

      if (all_buffers_are_non_empty_())
      {
        const auto evt_actual_time = recv_msgs_time - shifts_[i];

        SyncedBuffer synced;
        bool is_synced = try_sync_buffers_<i>(synced, evt_actual_time);

        if (is_synced)
        {
          std::get<i>(synced) = std::get<i>(buffer_).back();

          parent_->signal(std::get<0>(synced), std::get<1>(synced), std::get<2>(synced), std::get<3>(synced),
                          std::get<4>(synced), std::get<5>(synced), std::get<6>(synced), std::get<7>(synced),
                          std::get<8>(synced));

          remove_synced_elements_(synced);
        }
      }
    }

  protected:
    /**
     * @brief Initialize the parent synchronizer
     * @param parent Pointer to the parent Synchronizer instance
     */
    void initParent(Sync *parent) { parent_ = parent; }

    /**
     * @brief Purge old messages from the specified buffer
     * @param current_time The reference time for determining old messages
     *
     * @note Assumes buffer_access_mutex_ is already locked
     */
    template <int i>
    void fit_buffer_size_(const double current_time)
    {
      const auto &max_delay = max_delay_;
      auto &current_buff = std::get<i>(buffer_);
      const auto buff_new_end =
          std::remove_if(current_buff.begin(), current_buff.end(), [&current_time, &max_delay](const auto &evt)
                         {
          const auto msg_time = get_time_from_msg_sec<i>(evt);
          return std::abs(current_time - msg_time) > max_delay; });
      current_buff.erase(buff_new_end, current_buff.end());
    }

    /**
     * @brief Attempt to find a message in buffer i that matches the reference time
     * 
     * @param[out] synced The synchronized message tuple to populate if found
     * @param evt_actual_time The reference time to match against
     * @param[in,out] synced_count Counter of how many messages have been matched so far
     */
    template <int i>
    void try_sync_buffer_(SyncedBuffer &synced, const double evt_actual_time, int &synced_count) const
    {
      const auto &i_buffer = std::get<i>(buffer_);
      for (const auto &el : i_buffer)
      {
        const auto i_actual_time = get_time_from_msg_sec<i>(el) - shifts_[i];
        if (std::abs(evt_actual_time - i_actual_time) < max_aligned_time_eps_)
        {
          std::get<i>(synced) = el;
          ++synced_count;
          return;
        }
      }
    }

    /**
     * @brief Recursively attempt to synchronize messages across all buffers except the excluded one
     * @param[out] synced The synchronized message tuple to populate
     * @param evt_actual_time The reference time to match against
     * @param[in,out] synced_count Counter of how many messages have been matched so far
     *
     * @note Assumes buffer_access_mutex_ is already locked
     */
    template <int Start, int End, int Exclude>
    void try_sync_buffers_(SyncedBuffer &synced, [[maybe_unused]] const double evt_actual_time, int &synced_count) const
    {
      if constexpr (Start < End)
      {
        if constexpr (Start != Exclude)
        {
          try_sync_buffer_<Start>(synced, evt_actual_time, synced_count);
        }
        try_sync_buffers_<Start + 1, End, Exclude>(synced, evt_actual_time, synced_count);
      }
    }

    /**
     * @brief Attempt to synchronize messages across all buffers except the specified one
     * @param[out] synced The synchronized message tuple to populate
     * @param evt_actual_time The reference time to match against
     * 
     * @return true if a complete set of synchronized messages was found
     *
     * @note Assumes buffer_access_mutex_ is already locked
     */
    template <int i>
    bool try_sync_buffers_(SyncedBuffer &synced, [[maybe_unused]] const double evt_actual_time) const
    {
      int synced_count = 1;
      try_sync_buffers_<0, RealTypeCount::value, i>(synced, evt_actual_time, synced_count);
      return synced_count == RealTypeCount::value;
    }

    /**
     * @brief Recursively check if any buffers are empty
     * @param[in,out] res The result flag (will be ANDed with emptiness check)
     */
    template <int Start, int End>
    void buffers_is_empty_(bool &res) const
    {
      if constexpr (Start < End)
      {
        res &= Start < RealTypeCount::value && std::get<Start>(buffer_).empty();
        buffers_is_empty_<Start + 1, End>(res);
      }
    }

    /**
     * @brief Check if all buffers contain at least one message
     * @return true if all buffers are non-empty
     */
    bool all_buffers_are_non_empty_() const
    {
      bool res = true;
      buffers_is_empty_<0, RealTypeCount::value>(res);
      return !res;
    }

    /**
     * @brief Recursively remove synchronized messages from buffers
     * @param sb The synchronized message tuple containing messages to remove
     *
     * @note Assumes buffer_access_mutex_ is already locked
     */
    template <int Start, int End>
    void remove_synced_elements_(const SyncedBuffer &sb)
    {
      if constexpr (Start < End)
      {
        auto &current_buff = std::get<Start>(buffer_);
        const auto synced_time = get_time_from_msg_sec<Start>(std::get<Start>(sb));

        const auto &removed_end = std::remove_if(current_buff.begin(), current_buff.end(), [synced_time](const auto &el)
                                                 {
        const auto el_time = get_time_from_msg_sec<Start>(el);

        // Will not work if 1 / freq < TIME_CMP_TOLERANCE
        return el_time < (synced_time + TIME_CMP_TOLERANCE); });
        current_buff.erase(removed_end, current_buff.end());
        remove_synced_elements_<Start + 1, End>(sb);
      }
    }

    /**
     * @brief Remove all synchronized messages from buffers
     * @param sb The synchronized message tuple containing messages to remove
     *
     * @note Assumes buffer_access_mutex_ is already locked
     */
    void remove_synced_elements_(const SyncedBuffer &sb) { remove_synced_elements_<0, RealTypeCount::value>(sb); }

    /**
     * @brief Extract timestamp from a message event
     * @param evt The message event
     * @return double The timestamp in seconds
     */
    template <int i>
    static double get_time_from_msg_sec(const typename mpl::at_c<Events, i>::type &evt)
    {
      return mt::TimeStamp<typename mpl::at_c<Messages, i>::type>::value(*evt.getConstMessage()).toSec();
    }

  protected:
    Sync *parent_ = nullptr;                          ///< Pointer to parent Synchronizer instance
    std::array<double, RealTypeCount::value> shifts_; ///< Time shifts for each message type
    MsgsEventsBuffer buffer_;                         ///< Buffers for each message type
    double max_delay_ = .0;                           ///< Maximum time to keep messages in buffers
    double max_aligned_time_eps_ = .0;                ///< Timestamp alignment tolerance

    std::mutex buffer_access_mutex_; ///< Mutex for thread-safe buffer access

    static constexpr double TIME_CMP_TOLERANCE = 1e-5; ///< Time comparison tolerance for message removal
  };

} // namespace message_filters::evo_sync_policies
