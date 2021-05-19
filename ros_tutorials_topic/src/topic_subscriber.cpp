#include "ros/ros.h"                          // ROSの基本的なヘッダーファイル
#include "ros_tutorials_topic/MsgTutorial.h"  // MsgTutorial.msgファイルをビルドして自動生成されるヘッダーファイル

// メッセージのコールバック関数
// ros_tutorial_msgを受信した時に動作する
void msgCallback(const ros_tutorials_topic::MsgTutorial::ConstPtr& msg)
{
  ROS_INFO("recieve msg = %d", msg->stamp.sec);   // stamp.secメッセージを表示する
  ROS_INFO("recieve msg = %d", msg->stamp.nsec);  // stamp.nsecメッセージを表示する
  ROS_INFO("recieve msg = %d", msg->data);        // dataメッセージを表示する
}

int main(int argc, char **argv)                         // ノードのメイン関数
{
  ros::init(argc, argv, "topic_subscriber");            // ノード名の初期化

  ros::NodeHandle nh;                                   // ROSシステムとの通信を行うためのノードハンドルの宣言

  // サブスクライバーの宣言
  // トピック名：ros_tutorial_msg
  // コールバック関数名：msgCallback
  // サブスクライバー名：ros_tutorial_sub
  // サブスクライバーキューサイズ：100
  ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, msgCallback);

  // コールバック関数を呼び出すための関数
  // メッセージの受信まで待機し、受信した場合コールバック関数を実行する
  ros::spin();

  return 0;
}
