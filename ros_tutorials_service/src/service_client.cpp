#include "ros/ros.h"                          // ROSの基本的なヘッダーファイル
#include "ros_tutorials_service/SrvTutorial.h"// SrvTutorial.srvファイルをビルドして自動生成されるヘッダーファイル
#include <cstdlib>                            // atoll関数を含んだライブラリ

int main(int argc, char **argv)               // ノードのメイン関数
{
  ros::init(argc, argv, "service_client");    // ノード名の初期化

  if (argc != 3)  // 入力値エラーの処理
  {
    ROS_INFO("cmd : rosrun ros_tutorials_service service_client arg0 arg1");
    ROS_INFO("arg0: double number, arg1: double number");
    return 1;
  }

  ros::NodeHandle nh;       // ROSシステムとの通信を行うためのノードハンドルの宣言

  // サービスクライアントの宣言
  // サービス名：ros_turorials_service
  // サービス型：ros_tutorials_service::SrvTutorial
  // サービスクライアント名：ros_tutorials_service_client
  ros::ServiceClient ros_tutorials_service_client = nh.serviceClient<ros_tutorials_service::SrvTutorial>("ros_tutorial_srv");

  // サービスの宣言
  ros_tutorials_service::SrvTutorial srv;

  // サービス要請を行うノードの実行時に、キー入力の値をサービスa,bに設定する
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  // サービス要請をし、要請が受け付けられた場合に返された応答の値を表示する
  if (ros_tutorials_service_client.call(srv))
  {
    ROS_INFO("send srv, srv.Request.a and b: %ld, %ld", (long int)srv.request.a, (long int)srv.request.b);
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service ros_tutorial_srv");
    return 1;
  }
  return 0;
}
