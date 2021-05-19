#include "ros/ros.h"                          // ROSの基本的なヘッダーファイル
#include "ros_tutorials_service/SrvTutorial.h"// SrvTutorial.srvファイルをビルドして自動生成されるヘッダーファイル

// サービス要請はreq引数に、サービス応答はres引数に与えられる
bool calculation(ros_tutorials_service::SrvTutorial::Request &req,
                 ros_tutorials_service::SrvTutorial::Response &res)
{
  // サービス要請を受けた時、aとbの値を足し、サービス応答のメッセージに設定する
  res.result = req.a + req.b;

  // サービス要請に使用したa,bの値を出力し、サービス応答の際に送信するresultの値を表示する
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: %ld", (long int)res.result);

  return true;
}

int main(int argc, char **argv)              // ノードのメイン関数
{
  ros::init(argc, argv, "service_server");   // ノード名の初期化
  ros::NodeHandle nh;                        // ROSシステムとの通信を行うためのノードハンドルの宣言

  // サービスサーバーの宣言
  // サービス名：ros_tutorial_srv
  // サービスクライアント名：ros_tutorials_service_server
  // コールバック関数名：calculation
  ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

  ROS_INFO("ready srv server!");

  // コールバック関数を呼び出すための関数
  // サービスの受信まで待機し、受信した場合コールバック関数を実行する
  ros::spin();

  return 0;
}
