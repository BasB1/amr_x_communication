#include <ros.h>
#include <std_msgs/Float64.h>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

uint8_t ranging_protocol = POZYX_RANGE_PROTOCOL_FAST; // ranging protocol of the Pozyx.
int status = 0;

ros::NodeHandle  nh;

std_msgs::Float64 r_0_msg;
std_msgs::Float64 r_1_msg;
std_msgs::Float64 r_2_msg;
std_msgs::Float64 r_3_msg;

ros::Publisher r_0_range("r_0_range", &r_0_msg);
ros::Publisher r_1_range("r_1_range", &r_1_msg);
ros::Publisher r_2_range("r_2_range", &r_2_msg);
ros::Publisher r_3_range("r_3_range", &r_3_msg);

float r_0_prev;
float r_1_prev;
float r_2_prev;
float r_3_prev;

void setup() {
  if (Pozyx.begin() == POZYX_FAILURE) {
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    abort();
  }

  Pozyx.setRangingProtocol(ranging_protocol, 0x6719);
  Pozyx.setRangingProtocol(ranging_protocol, 0x671b);
  Pozyx.setRangingProtocol(ranging_protocol, 0x6711);
  Pozyx.setRangingProtocol(ranging_protocol, 0x6762);

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.advertise(r_0_range);
  nh.advertise(r_1_range);
  nh.advertise(r_2_range);
  nh.advertise(r_3_range);
}

void loop() {
  device_range_t r_0;
  device_range_t r_1;
  device_range_t r_2;
  device_range_t r_3;

  status = Pozyx.doRemoteRanging(0x6719, 0x671b, &r_0);
  status = Pozyx.doRemoteRanging(0x6762, 0x671b, &r_2);
  status = Pozyx.doRemoteRanging(0x6719, 0x6711, &r_1);
  status = Pozyx.doRemoteRanging(0x6762, 0x6711, &r_3);

  if (r_0.distance == 0) {
    r_0_msg.data = r_0_prev;
  }
  else {
    r_0_prev = r_0.distance / 1000.;
    r_0_msg.data = r_0.distance / 1000.;
  }

  if (r_1.distance == 0) {
    r_1_msg.data = r_1_prev;
  }
  else {
    r_1_prev = r_1.distance / 1000.;
    r_1_msg.data = r_1.distance / 1000.;
  }

  if (r_2.distance == 0) {
    r_2_msg.data = r_2_prev;
  }
  else {
    r_2_prev = r_2.distance / 1000.;
    r_2_msg.data = r_2.distance / 1000.;
  }

  if (r_3.distance == 0) {
    r_3_msg.data = r_3_prev;
  }
  else {
    r_3_prev = r_3.distance / 1000.;
    r_3_msg.data = r_3.distance / 1000.;
  }


  r_0_range.publish( &r_0_msg );
  r_1_range.publish( &r_1_msg );
  r_2_range.publish( &r_2_msg );
  r_3_range.publish( &r_3_msg );

  nh.spinOnce();
}
