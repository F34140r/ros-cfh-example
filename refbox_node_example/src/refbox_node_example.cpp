
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

/*
 * Handler for send errors.
 *
 * This handler is called for send errors.
 * Print the message that should be send as Ros Warning
 *
 */
void handle_send_error(std::string msg){
  ROS_WARN("Send error: %s\n", msg.c_str());
}

/*
 * Handler for recive errors.
 *
 * This handler is called for recive errors.
 * Print the message that should be recive as Ros Warning
 *
 */
void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                        std::string msg){
  ROS_WARN("Recv error: %s\n", msg.c_str());

}

/*
 * Handler for recive messages .
 *
 * This handler is called for recive message.
 * Main function in this node.
 *
 */
void handle_message(boost::asio::ip::udp::endpoint &sender,
                    uint16_t component_id, uint16_t msg_type,
                    std::shared_ptr<google::protobuf::Message> msg){

  std::shared_ptr<AttentionMessage> am;                         /* shared_ptr on the message type */
  if((am = std::dynamic_pointer_cast<AttentionMessage>(msg))){  /* test it is the right message type */
    refbox_msgs_example::AttentionMessage attention_msg;        /* ros message handling */
    attention_msg.message      = am->message();
    attention_msg.time_to_show = am->time_to_show();
    attention_msg.team         = am->team();
    AttentionMessage_pub.publish(attention_msg);
  }
}

/*
 * Example for a send command
 *
 * This subscribe control the qa camera.
 * Use subscriber or Action server/client to send commands to the refbox
 *
 */
void CameraControl(refbox_msgs_example::CameraControl msg){
  std::shared_ptr<CameraCommand> cam_control(new CameraCommand); /*create a new message */
  peer_team->send(cam_control);                                  /*send the Message over team peer */
}

/*
 * Beacon Signal
 *
 * This function send the beacon signal
 * The function is called from the main loop
 */
void send_beacon(){
  timespec start;                            /*generate the timestamp over boost or chrono (C++11) */
  clock_gettime(CLOCK_REALTIME, &start);
  int32_t sec = start.tv_sec;
  int32_t nsec = start.tv_nsec;

  std::shared_ptr<BeaconSignal> signal(new BeaconSignal());  /*create the message */

  Time *time = signal->mutable_time();  /*seperate the time segment of the message */
  time->set_sec(sec);                   /*write the timestamp into the message*/
  time->set_nsec(nsec);

  signal->set_peer_name(name);           /*write the name und co into the message*/
  signal->set_team_name(team_name);
  signal->set_seq(++seq_);               /*increase the sequenz number*/
  peer_team->send(signal);               /*send over team peer*/
}
/*
 * Main function
 *
 * Use to create and configure the peers
 *
 */
int main(int argc, char **argv){

  ros::init(argc, argv, "refbox_node");
  ROS_INFO("Refbox is running!");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1); // one Hz
  int count = 0;

  /*
   * Confgiuration Parameters
   * - can use the ros parameter
   *   Server to handle this
   */ 
  std::string hostname = "127.0.0.1";
  int team_port = 4440, public_port = 4448;

  name      = "nobody";
  team_name = "nobody";

  ROS_INFO("Hostname: %s", hostname.c_str());
  ROS_INFO("Team Port: %i", team_port);
  ROS_INFO("Public Port: %i", public_port);
  ROS_INFO("Name: %s", name.c_str());
  ROS_INFO("Team Name: %s", team_name.c_str());

  boost::asio::io_service io_service;

  /*
   *  peer creation
   *  Section of peer creation
   *  creation the public and team peer
   *
   */
  peer_public = new ProtobufBroadcastPeer(hostname, public_port);       /*create public peer*/

  MessageRegister &message_register = peer_public->message_register();  /*create internal message handler*/
  message_register.add_message_type<AttentionMessage>();                /*added messagetype to the handler*/

  peer_team = new ProtobufBroadcastPeer(hostname, team_port, &message_register); /*create team peer and linked to internal message handler*/

  peer_public->signal_received().connect(handle_message);              /*bind the peers to the callback funktions*/
  peer_public->signal_send_error().connect(handle_send_error);
  peer_public->signal_recv_error().connect(handle_recv_error);

  peer_team->signal_received().connect(handle_message);
  peer_team->signal_send_error().connect(handle_send_error);
  peer_team->signal_recv_error().connect(handle_recv_error);

#if BOOST_ASIO_VERSION >= 100601
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);       /*Construct a signal set registered for process termination. */
  signals.async_wait(signal_handler);                                 /*Start an asynchronous wait for one of the signals to occur.*/
#endif

  /*
   * Ros Part
   * creation the Publisher und Subscriber
   * for Ros
   */
  //Publisher
  AttentionMessage_pub = nh.advertise<refbox_msgs_example::AttentionMessage> ("attention_message", 10);

  //Subscriber
  CameraConrol_sub = nh.subscribe<refbox_msgs_example::CameraControl>("camera_control", 1000, CameraControl);


  /*
   *  Main loop
   *
   */
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
