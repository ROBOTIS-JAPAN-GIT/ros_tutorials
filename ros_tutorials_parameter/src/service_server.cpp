#include "ros/ros.h"                            // ROSの基本的なヘッダーファイル
#include "ros_tutorials_parameter/SrvTutorial.h"// SrvTutorial.srvファイルをビルドして自動生成されるヘッダーファイル

#define PLUS            1   // 足し算
#define MINUS           2   // 引き算
#define MULTIPLICATION  3   // 掛け算
#define DIVISION        4   // 割り算

int g_operator = PLUS;

// サービス要請はreq引数に、サービス応答はres引数に与えられる
bool calculation(ros_tutorials_parameter::SrvTutorial::Request &req,
                 ros_tutorials_parameter::SrvTutorial::Response &res)
{
  // サービス要請を受けた際、得られたa,bの値をパラメーターの値に従い演算する
  // 演算後、サービス応答のメッセージに値を与える
  switch(g_operator)
  {
    case PLUS:
         res.result = req.a + req.b; break;
    case MINUS:
         res.result = req.a - req.b; break;
    case MULTIPLICATION:
         res.result = req.a * req.b; break;
    case DIVISION:
         if(req.b == 0)
         {
           res.result = 0; break;
         }
         else
         {
           res.result = req.a / req.b; break;
         }
    default:
         res.result = req.a + req.b; break;
  }

  // サービス要請に使用したa,bの値を出力し、サービス応答の際に送信するresultの値を表示する
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.result);

  return true;
}

int main(int argc, char **argv)             // ノードのメイン関数
{
  ros::init(argc, argv, "service_server");  // ノード名の初期化
  ros::NodeHandle nh;                       // ROSシステムとの通信を行うためのノードハンドルの宣言

  nh.setParam("calculation_method", PLUS);  // Reset Parameter Settings

  // サービスサーバーの宣言
  // サービス名：ros_tutorial_srv
  // サービスクライアント名：ros_tutorials_service_server
  // コールバック関数名：calculation
  ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

  ROS_INFO("ready srv server!");

  // ループ周期を設定する。ここでは周期を10Hz(0.1秒間隔でのループ)に設定
  ros::Rate r(10);

  while (ros::ok())
  {
    // 演算子をパラメータにしたがって変更する
    nh.getParam("calculation_method", g_operator);
    // コールバック関数を呼び出すための関数
    ros::spinOnce();
    // 設定したループ周期に合わせてスリープする
    r.sleep();
  }

  return 0;
}
