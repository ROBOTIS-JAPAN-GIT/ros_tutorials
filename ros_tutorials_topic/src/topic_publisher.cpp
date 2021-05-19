#include "ros/ros.h"                            // ROSの基本的なヘッダーファイル
#include "ros_tutorials_topic/MsgTutorial.h"    // MsgTutorial.msgファイルをビルドして自動生成されるヘッダーファイル

int main(int argc, char **argv)                 // ノードのメイン関数
{
  ros::init(argc, argv, "topic_publisher");     // ノード名の初期化
  ros::NodeHandle nh;                           // ROSシステムとの通信を行うためのノードハンドルの宣言

  // パブリッシャーの宣言
  // トピック名：ros_tutorial_msg
  // トピック型：ros_tutorials_topic::MsgTutorial
  // パブリッシャー名：ros_tutorial_pub
  // パブリッシャーキューサイズ：100
  ros::Publisher ros_tutorial_pub = nh.advertise<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100);

  // ループ周期を設定する。ここでは周期を10Hz(0.1秒間隔でのループ)に設定
  ros::Rate loop_rate(10);

  ros_tutorials_topic::MsgTutorial msg;     // メッセージの宣言
  int count = 0;                            // メッセージで使用する変数の宣言
  while (ros::ok())
  {
    msg.stamp = ros::Time::now();           // 現在時刻をmsgのstampに設定する
    msg.data  = count;                      // countの値をmsgのcountに設定する

    ROS_INFO("send msg = %d", msg.stamp.sec);   // stamp.secメッセージを表示する
    ROS_INFO("send msg = %d", msg.stamp.nsec);  // stamp.nsecメッセージを表示する
    ROS_INFO("send msg = %d", msg.data);        // dataメッセージを表示する

    ros_tutorial_pub.publish(msg);          // msgをパブリッシュする

    loop_rate.sleep();                      // 設定したループ周期に合わせてスリープする

    ++count;                                // countを１づつ加算する
  }

  return 0;
}
