#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <roscpp_message_reflection/message_description_provider.h>
#include <roscpp_message_reflection/service_description.h>
#include "test_utils.h"

using namespace roscpp_message_reflection;

TEST(ServiceDescription, construct_simple_service)
{
  std::vector<FieldDescription> req_fields;
  req_fields.push_back(FieldDescription::CreateFromFullType("a", "int32"));
  MessageDescription::Ptr req(new MessageDescription("a_package/AServiceRequest", "992ce8a1687cec8c8bd883ec73ca1234",
          "int32 a\n\n", req_fields, std::map<std::string, MessageDescription::Ptr>()));

  std::vector<FieldDescription> res_fields;
  res_fields.push_back(FieldDescription::CreateFromFullType("b", "uint16"));
  res_fields.push_back(FieldDescription::CreateFromFullType("d", "string"));
  MessageDescription::Ptr res(new MessageDescription("a_package/AServiceResponse", "992ce8a1687cec8c8bd883ed73ca1234",
          "uint16 b\nstring d\n\n", res_fields, std::map<std::string, MessageDescription::Ptr>()));

  ServiceDescription description("a_package/AService", "992ce8a1687cec8c8bd883ec73ca1235", req, res);

  EXPECT_EQ("a_package/AService", description.name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ec73ca1235", description.md5sum);

  EXPECT_EQ(req, description.request);
  EXPECT_EQ("a_package/AServiceRequest", description.request->name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ec73ca1234", description.request->md5sum);
  EXPECT_GT(description.request->full_text.size(), 0);
  ASSERT_EQ(1, description.request->fields.size());
  EXPECT_VALUE_FIELD("a", "int32", description.request->fields[0]);

  EXPECT_EQ(res, description.response);
  EXPECT_EQ("a_package/AServiceResponse", description.response->name);
  EXPECT_EQ("992ce8a1687cec8c8bd883ed73ca1234", description.response->md5sum);
  EXPECT_GT(description.response->full_text.size(), 0);
  ASSERT_EQ(2, description.response->fields.size());
  EXPECT_VALUE_FIELD("b", "uint16", description.response->fields[0]);
  EXPECT_VALUE_FIELD("d", "string", description.response->fields[1]);
}


class ServiceDescriptionProviderTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    provider.reset(MessageDescriptionProvider::Create(nh));
  }
  ros::NodeHandle nh;
  boost::scoped_ptr<MessageDescriptionProvider> provider;
};

TEST_F(ServiceDescriptionProviderTest, get_std_srvs_Empty_description)
{
  ServiceDescription::Ptr description = provider->getServiceDescription("std_srvs/Empty");
  ASSERT_TRUE(description);
  EXPECT_EQ("std_srvs/Empty", description->name);
  EXPECT_EQ("d41d8cd98f00b204e9800998ecf8427e", description->md5sum);

  ASSERT_TRUE(description->request);
  EXPECT_EQ("std_srvs/EmptyRequest", description->request->name);
  EXPECT_EQ("d41d8cd98f00b204e9800998ecf8427e", description->request->md5sum);
  EXPECT_GT(description->request->full_text.size(), 0);
  ASSERT_EQ(0, description->request->fields.size());

  ASSERT_TRUE(description->response);
  EXPECT_EQ("std_srvs/EmptyResponse", description->response->name);
  EXPECT_EQ("d41d8cd98f00b204e9800998ecf8427e", description->response->md5sum);
  EXPECT_GT(description->response->full_text.size(), 0);
  ASSERT_EQ(0, description->response->fields.size());
}

TEST_F(ServiceDescriptionProviderTest, get_std_srvs_description)
{
  ServiceDescription::Ptr description = provider->getServiceDescription("sensor_msgs/SetCameraInfo");
  ASSERT_TRUE(description);
  EXPECT_EQ("sensor_msgs/SetCameraInfo", description->name);
  EXPECT_EQ("bef1df590ed75ed1f393692395e15482", description->md5sum);

  ASSERT_TRUE(description->request);
  EXPECT_EQ("sensor_msgs/SetCameraInfoRequest", description->request->name);
  EXPECT_EQ("ee34be01fdeee563d0d99cd594d5581d", description->request->md5sum);
  EXPECT_GT(description->request->full_text.size(), 0);
  ASSERT_EQ(1, description->request->fields.size());
  EXPECT_VALUE_FIELD("camera_info", "sensor_msgs/CameraInfo", description->request->fields[0]);

  ASSERT_TRUE(description->response);
  EXPECT_EQ("sensor_msgs/SetCameraInfoResponse", description->response->name);
  EXPECT_EQ("2ec6f3eff0161f4257b808b12bc830c2", description->response->md5sum);
  EXPECT_GT(description->response->full_text.size(), 0);
  ASSERT_EQ(2, description->response->fields.size());
  EXPECT_VALUE_FIELD("success", "bool", description->response->fields[0]);
  EXPECT_VALUE_FIELD("status_message", "string", description->response->fields[1]);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "service_description_test");

  return RUN_ALL_TESTS();
}
