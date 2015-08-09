#include <gtest/gtest.h>
#include <ros/ros.h>
#include <roscpp_message_reflection/message.h>
#include <roscpp_message_reflection/message_impl.h>
#include <roscpp_message_reflection/node_handle.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Trigger.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

using namespace roscpp_message_reflection;

class NodeHandleTest : public ::testing::Test {
public:
  NodeHandleTest() : nh(ros_nh) {}

protected:
  virtual void SetUp() {
    const ::testing::TestInfo* const test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();
    test_topic_ = "/roscpp_message_reflection/" + std::string(test_info->test_case_name()) +
      "/" + std::string(test_info->name());
  }

  std::string test_topic() {
    return test_topic_;
  }

  ros::NodeHandle ros_nh;
  NodeHandle nh;

private:
  std::string test_topic_;
};

template <class M>
class TestSubscriberCallback {
public:
  operator boost::function<void(const typename M::ConstPtr&)>() {
    return boost::bind(&TestSubscriberCallback::callback, this, _1);
  }

  void callback(const typename M::ConstPtr& message) {
    last_message_ = message;
    cv_.notify_all();
  }

  typename M::ConstPtr getLastMessage() {
    return last_message_;
  }

  template<class duration_type>
  bool waitForMessage(const duration_type& duration) {
    boost::unique_lock<boost::mutex> lock(m_);
    return cv_.timed_wait(lock, duration);
  }

private:
  typename M::ConstPtr last_message_;

  boost::mutex m_;
  boost::condition_variable cv_;
};


TEST_F(NodeHandleTest, publishSharedPointer) {
  Publisher pub = nh.advertise(test_topic(), "geometry_msgs/Vector3");
  Message::Ptr m(new Message(pub.getMessageType()));
  (*m)["x"] = 1.2;
  (*m)["y"] = 2.2;
  (*m)["z"] = 3.3;

  TestSubscriberCallback<geometry_msgs::Vector3> callback;
  ros::Subscriber sub = ros_nh.subscribe<geometry_msgs::Vector3>(test_topic(), 1, callback);

  pub.publish(m);

  EXPECT_EQ(1, sub.getNumPublishers());

  ASSERT_TRUE(callback.waitForMessage(boost::posix_time::seconds(1)));

  EXPECT_EQ(1.2, callback.getLastMessage()->x);
  EXPECT_EQ(2.2, callback.getLastMessage()->y);
  EXPECT_EQ(3.3, callback.getLastMessage()->z);
}

TEST_F(NodeHandleTest, publishReference) {
  Publisher pub = nh.advertise(test_topic(), "geometry_msgs/Vector3");
  Message m = pub.createMessage();
  m["x"] = 1.2;
  m["y"] = 2.2;
  m["z"] = 3.3;

  TestSubscriberCallback<geometry_msgs::Vector3> callback;
  ros::Subscriber sub = ros_nh.subscribe<geometry_msgs::Vector3>(test_topic(), 1, callback);

  pub.publish(m);

  EXPECT_EQ(1, sub.getNumPublishers());

  ASSERT_TRUE(callback.waitForMessage(boost::posix_time::seconds(1)));

  EXPECT_EQ(1.2, callback.getLastMessage()->x);
  EXPECT_EQ(2.2, callback.getLastMessage()->y);
  EXPECT_EQ(3.3, callback.getLastMessage()->z);
}


TEST_F(NodeHandleTest, subscribe) {
  ros::Publisher pub = ros_nh.advertise<geometry_msgs::Vector3>(test_topic(), 1);
  geometry_msgs::Vector3 m;
  m.x = 3.0;
  m.y = 1.1;
  m.z = 8.2;

  TestSubscriberCallback<Message> callback;
  Subscriber sub = nh.subscribe(test_topic(), "geometry_msgs/Vector3", callback);

  pub.publish(m);

  ASSERT_TRUE(callback.waitForMessage(boost::posix_time::seconds(1)));

  EXPECT_EQ(3.0, (*callback.getLastMessage())["x"].as<double>());
  EXPECT_EQ(1.1, (*callback.getLastMessage())["y"].as<double>());
  EXPECT_EQ(8.2, (*callback.getLastMessage())["z"].as<double>());
}


bool serviceClientTestSuccessCallback(std_srvs::Trigger::Request& req,
			       std_srvs::Trigger::Response& res) {
  res.message = "TEST MESSAGE!";
  res.success = false;
  return true;
}

TEST_F(NodeHandleTest, serviceClientSuccess) {
  ServiceClient client = nh.serviceClient(test_topic(), "std_srvs/Trigger");

  ros::ServiceServer server = ros_nh.advertiseService(test_topic(), serviceClientTestSuccessCallback);

  Message req = client.createRequestMessage();
  Message res = client.createResponseMessage();
  bool result = client.call(req, res);

  EXPECT_TRUE(result);
  EXPECT_EQ("TEST MESSAGE!", res["message"].as<std::string>());
  EXPECT_EQ(false, res["success"].as<bool>());
}

bool serviceClientTestFailureCallback(std_srvs::Trigger::Request& req,
			       std_srvs::Trigger::Response& res) {
  return false;
}

TEST_F(NodeHandleTest, serviceClientFailure) {
  ServiceClient client = nh.serviceClient(test_topic(), "std_srvs/Trigger");

  ros::ServiceServer server = ros_nh.advertiseService(test_topic(), serviceClientTestFailureCallback);

  Message req = client.createRequestMessage();
  Message res = client.createResponseMessage();
  bool result = client.call(req, res);

  EXPECT_FALSE(result);
}

bool serviceServerTestSuccessCallback(const Message& req,
				      Message& res) {
  res["message"] = "TEST MESSAGE!";
  res["success"] = false;
  return true;
}

TEST_F(NodeHandleTest, serviceServerSuccess) {
  ros::ServiceClient client = ros_nh.serviceClient<std_srvs::Trigger>(test_topic());

  ServiceServer server = nh.advertiseService(test_topic(), "std_srvs/Trigger", serviceServerTestSuccessCallback);

  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  bool result = client.call(req, res);

  EXPECT_TRUE(result);
  EXPECT_EQ("TEST MESSAGE!", res.message);
  EXPECT_EQ(false, res.success);
}

bool serviceServerTestFailureCallback(const Message& req,
				      Message& res) {
  return false;
}

TEST_F(NodeHandleTest, serviceServerFailure) {
  ros::ServiceClient client = ros_nh.serviceClient<std_srvs::Trigger>(test_topic());

  ServiceServer server = nh.advertiseService(test_topic(), "std_srvs/Trigger", serviceServerTestFailureCallback);

  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  bool result = client.call(req, res);

  EXPECT_FALSE(result);
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "node_handle_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  return RUN_ALL_TESTS();
}

