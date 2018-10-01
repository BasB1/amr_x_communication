
#include <ros.h>
#include <ArduinoJson.h>

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
ros::NodeHandle nh;

uint16_t source_id;                 // the network id of this device
uint16_t destination_id = 0;        // the destination network id. 0 means the message is broadcasted to every device in range
String inputString = "";            // a string to hold incoming data

volatile int TIME_STORE_0 = 0;
volatile int TIME_STORE_1 = 0;
int hz = 200;

int i = 0;
int j = 0;
int k = 0;

boolean chopStatus = false;
int _length;
boolean receivingString = false;
String totalString;
String inputString_string;

StaticJsonDocument<512> jsonDATA;

geometry_msgs::Pose pose_msg;

void messageCb( const std_msgs::Empty& toggle_msg) {
  nh.loginfo("Message received");
}
ros::Subscriber<std_msgs::Empty> test("test", messageCb );

void odomCb( const geometry_msgs::Pose& msg) {
  JsonObject root = jsonDATA.to<JsonObject>();

  /* Pose */
  root["pose_position_x"] = msg.position.x;
  root["pose_position_y"] = msg.position.y;
  root["pose_position_z"] = msg.position.z;
  root["pose_orientation_x"] = msg.orientation.x;
  root["pose_orientation_y"] = msg.orientation.y;
  root["pose_orientation_z"] = msg.orientation.z;
  root["pose_orientation_w"] = msg.orientation.w;

  serializeJson(root, inputString);

  uint8_t _length = inputString.length();
  if (_length - 1 >= 100) {
    chopStatus = true;
    while (chopStatus) {
      for (i = 0 + j; i <= 99 + j; i++) {
        inputString_string += inputString[i];
      }
      Serial.println(inputString_string);
      uint8_t length = inputString_string.length();
      uint8_t buffer[length];
      inputString_string.getBytes(buffer, length);
      int status = Pozyx.writeTXBufferData(buffer, length);
      status = Pozyx.sendTXBufferData(destination_id);
      delay(50);
      if ((_length - 100) >= 0) {
        _length -= 99;
        j += 99;
        k = 1;
        inputString_string = "";
      }
      else {
        inputString_string = "##";
        uint8_t length = inputString_string.length();
        uint8_t buffer[length];
        inputString_string.getBytes(buffer, length);
        int status = Pozyx.writeTXBufferData(buffer, length);
        status = Pozyx.sendTXBufferData(destination_id);

        j = 0;
        i = 0;
        k = 0;
        inputString = "";
        inputString_string = "";
        chopStatus = false;
      }
    }
  }

  else if (_length - 1 <= 99) {
    uint8_t length = inputString.length();
    uint8_t buffer[length];
    inputString.getBytes(buffer, length);
    int status = Pozyx.writeTXBufferData(buffer, length);
    status = Pozyx.sendTXBufferData(destination_id);

    delay(20);

    inputString = "##";
    length = inputString.length();
    buffer[length];
    inputString.getBytes(buffer, length);
    status = Pozyx.writeTXBufferData(buffer, length);
    status = Pozyx.sendTXBufferData(destination_id);

    inputString = "";
  }
}
ros::Subscriber<geometry_msgs::Pose> uwb_com_tx("uwb_com_tx", odomCb );

ros::Publisher uwb_com_rx("uwb_com_rx", &pose_msg);

void setup() {
  // read the network id of this device
  Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);

  // reserve 100 bytes for the inputString:
  inputString.reserve(512);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(test);
  nh.advertise(uwb_com_rx);
  nh.subscribe(uwb_com_tx);

  if (! Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_RX_DATA, 0)) {
    nh.logerror("UWB ERROR");
    abort();
  }
  Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);
}

void loop() {
  TIME_STORE_0 = micros();

  if (Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA, 1))
  {
    nh.loginfo("Receiving");
    receivingString = true;

    while (receivingString) {
      uint8_t length = 0;
      uint16_t messenger = 0x00;

      Pozyx.getLastDataLength(&length);
      Pozyx.getLastNetworkId(&messenger);

      char data[length];
      Pozyx.readRXBufferData((uint8_t *) data, length);

      nh.loginfo(data);
      if (strstr(data, "#")) {
        receivingString = false;
      }
      else {
        for (i = 0; i <= strlen(data) - 1; i++) {
          totalString += data[i];
        }
        Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA, 200);
      }
    }

    deserializeJson(jsonDATA, totalString);
    JsonObject root = jsonDATA.as<JsonObject>();

    /* Pose */
    pose_msg.position.x = root["pose_position_x"];
    pose_msg.position.y = root["pose_position_y"];
    pose_msg.position.z = root["pose_position_z"];
    pose_msg.orientation.x = root["pose_orientation_x"];
    pose_msg.orientation.y = root["pose_orientation_y"];
    pose_msg.orientation.z = root["pose_orientation_z"];
    pose_msg.orientation.w = root["pose_orientation_w"];

    totalString = "";
    uwb_com_rx.publish( &pose_msg );
  }
  nh.spinOnce();
  delayMicroseconds((1000000 / hz) - (micros() - TIME_STORE_0));
}

