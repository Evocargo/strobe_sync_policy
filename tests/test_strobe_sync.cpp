#include <gtest/gtest.h>

#include "ros/ros.h"
#include "message_filters/synchronizer.h"
#include "strobe_sync_policy/strobe_sync_policy.hpp"

using namespace message_filters;
using namespace message_filters::evo_sync_policies;

constexpr double MAX_DEFAULT_DELAY = 1.;
constexpr double MAX_EPS = .01;

struct Header
{
  ros::Time stamp;
};

struct Msg
{
  Header header;
  int data;
};

typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

namespace ros
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static ros::Time value(const Msg& m)
  {
    return m.header.stamp;
  }
};
}
}

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb()
  {
    ++count_;
  }

  int32_t count_;
};

typedef StrobeSync<Msg, Msg> Policy2;
typedef StrobeSync<Msg, Msg, Msg> Policy3;

typedef Synchronizer<Policy2> Sync2;
typedef Synchronizer<Policy3> Sync3;

TEST(StrobeSync, SyncTwoTopicsWithoutShift)
{
  std::array<double, 2> shifts = {.0, .0};
  Sync2 sync(Policy2(shifts, MAX_DEFAULT_DELAY));
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  
  MsgPtr m(boost::make_shared<Msg>());
  m->header.stamp = ros::Time(.0);
  
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.0);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(StrobeSync, SyncTwoTopicsWithShift)
{
  std::array<double, 2> shifts = {.1, .0};
  Sync2 sync(Policy2(shifts, MAX_DEFAULT_DELAY));
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  
  MsgPtr m(boost::make_shared<Msg>());
  m->header.stamp = ros::Time(.1);
  
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.0);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(StrobeSync, FailSyncMoreThanMaxDelay)
{
  std::array<double, 2> shifts = {.0, .0};
  Sync2 sync(Policy2(shifts, MAX_DEFAULT_DELAY / 2));
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  
  MsgPtr m(boost::make_shared<Msg>());
  m->header.stamp = ros::Time(.0);
  
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(1.);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
}

TEST(StrobeSync, SyncThreeTopicsWithoutDelay)
{
  std::array<double, 3> shifts = {.0, .0, .0};
  Sync3 sync(Policy3(shifts, MAX_DEFAULT_DELAY));
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  
  MsgPtr m(boost::make_shared<Msg>());
  m->header.stamp = ros::Time(.0);
  
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.0);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.0);
  sync.add<2>(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(StrobeSync, SyncThreeTopicsWithShift)
{
  std::array<double, 3> shifts = {.1, .2, .0};
  Sync3 sync(Policy3(shifts, MAX_DEFAULT_DELAY));
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  
  MsgPtr m(boost::make_shared<Msg>());
  m->header.stamp = ros::Time(.1);
  
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.2);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.0);
  sync.add<2>(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(StrobeSync, FaileSyncThreeTopicsMoreThanAlignedTimeEps)
{
  std::array<double, 2> shifts = {0.0, 0.0};
  Sync2 sync(Policy2(shifts, MAX_DEFAULT_DELAY, MAX_EPS));
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  
  MsgPtr m(boost::make_shared<Msg>());
  m->header.stamp = ros::Time(.0);
  
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  
  m = boost::make_shared<Msg>();
  m->header.stamp = ros::Time(.02);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_strobe_sync");

  ros::Time::init();
  ros::Time::setNow(ros::Time());

  return RUN_ALL_TESTS();
} 