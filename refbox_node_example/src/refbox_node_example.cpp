
/***************************************************************************
 *  refbox_node.cpp - Refbox Node for RoCkin
 *
 *  Created: Tue Mar 21 00:44:58 2015
 *  Copyright  2015  Kai Seidensticker
 ****************************************************************************/
/*
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include "ros/ros.h"
#include <peer.h>

#include <AttentionMessage.pb.h>
#include <BeaconSignal.pb.h>
#include <Time.pb.h>
#include <Camera.pb.h>


//publisher
#include <refbox_msgs_example/AttentionMessage.h>

//subscribe
#include <refbox_msgs_example/CameraControl.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <unistd.h>

#include <sstream>

using namespace protobuf_comm;
using namespace rockin_msgs;

static bool quit = false;
static boost::asio::deadline_timer *timer_ = NULL;
std::string name, team_name;
unsigned long seq_ = 0;
ProtobufBroadcastPeer *peer_public = NULL, *peer_team = NULL;

//Publisher
ros::Publisher AttentionMessage_pub;

//Subscriber
ros::Subscriber CameraConrol_sub;

void signal_handler(const boost::system::error_code &error, int signum){
  if (!error){
    quit = true;
    if (timer_){
      timer_->cancel();
    }
  }
}

void handle_send_error(std::string msg){
  ROS_WARN("Send error: %s\n", msg.c_str());
}

void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                        std::string msg){
  ROS_WARN("Recv error: %s\n", msg.c_str());

}

void handle_message(boost::asio::ip::udp::endpoint &sender,
                    uint16_t component_id, uint16_t msg_type,
                    std::shared_ptr<google::protobuf::Message> msg){

  std::shared_ptr<AttentionMessage> am;
  if((am = std::dynamic_pointer_cast<AttentionMessage>(msg))){
    refbox_msgs_example::AttentionMessage attention_msg;
    attention_msg.message      = am->message();
    attention_msg.time_to_show = am->time_to_show();
    attention_msg.team         = am->team();
    AttentionMessage_pub.publish(attention_msg);
  }
}

void CameraControl(refbox_msgs_example::CameraControl msg){
  std::shared_ptr<CameraCommand> cam_control(new CameraCommand);
  peer_team->send(cam_control);
}

void send_beacon(){
  timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  int32_t sec = start.tv_sec;
  int32_t nsec = start.tv_nsec;

  std::shared_ptr<BeaconSignal> signal(new BeaconSignal());

  Time *time = signal->mutable_time();
  time->set_sec(sec);
  time->set_nsec(nsec);

  signal->set_peer_name(name);
  signal->set_team_name(team_name);
  signal->set_seq(++seq_);
  peer_team->send(signal);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "refbox_node");
  ROS_INFO("Refbox is running!");

  ros::NodeHandle nh;
  std::string hostname = "192.168.1.100";
  int team_port = 4440, public_port = 4448;

  name      = "nobody";
  team_name = "nobody";

  ROS_INFO("Hostname: %s", hostname.c_str());
  ROS_INFO("Team Port: %i", team_port);
  ROS_INFO("Public Port: %i", public_port);
  ROS_INFO("Name: %s", name.c_str());
  ROS_INFO("Team Name: %s", team_name.c_str());

  boost::asio::io_service io_service;

  peer_public = new ProtobufBroadcastPeer(hostname, public_port);

  MessageRegister &message_register = peer_public->message_register();
  message_register.add_message_type<AttentionMessage>();

  peer_team = new ProtobufBroadcastPeer(hostname, team_port, &message_register);

  peer_public->signal_received().connect(handle_message);
  peer_public->signal_send_error().connect(handle_send_error);
  peer_public->signal_recv_error().connect(handle_recv_error);

  peer_team->signal_received().connect(handle_message);
  peer_team->signal_send_error().connect(handle_send_error);
  peer_team->signal_recv_error().connect(handle_recv_error);

#if BOOST_ASIO_VERSION >= 100601
  // Construct a signal set registered for process termination.
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

  // Start an asynchronous wait for one of the signals to occur.
  signals.async_wait(signal_handler);
#endif

  //Publisher
  AttentionMessage_pub = nh.advertise<refbox_msgs_example::AttentionMessage> ("attention_message", 10);

  //Subscriber
  CameraConrol_sub = nh.subscribe<refbox_msgs_example::CameraControl>("camera_control", 1000, CameraControl);

  ros::Rate loop_rate(1); // Em Hz

  int count = 0;

  while (ros::ok()) {
    send_beacon();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  delete timer_;
  delete peer_public;
  delete peer_team;

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
