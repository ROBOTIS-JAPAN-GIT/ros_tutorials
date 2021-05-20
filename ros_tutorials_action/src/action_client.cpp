#include <ros/ros.h>                              // ROSの基本的なヘッダーファイル
#include <actionlib/client/simple_action_client.h>// actionライブラリのヘッダーファイル
#include <actionlib/client/terminal_state.h>      // actionライブラリの目標状態のヘッダーファイル
#include <ros_tutorials_action/FibonacciAction.h> // Fibonacci.actionファイルをビルドして自動生成されるヘッダーファイル

int main (int argc, char **argv)          // ノードのメイン関数
{
  ros::init(argc, argv, "action_client"); // ノード名の初期化

  // アクションクライアントの宣言
  // アクション名：ros_tutorial_action
  // アクション型：ros_tutorials_action::FibonacciAction
  // サービスクライアント名：ac
  actionlib::SimpleActionClient<ros_tutorials_action::FibonacciAction> ac("ros_tutorial_action", true);

  ROS_INFO("Waiting for action server to start.");
  // アクションサーバーが実行されるまで待機
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  ros_tutorials_action::FibonacciGoal goal; // アクション目標オブジェクトの宣言
  goal.order = 20;    // アクション目標の指定（フィボナッチ数列の20まで演算）
  ac.sendGoal(goal);  // アクション目標の転送

  // アクション目標の達成に対するタイムリミットを設定（ここでは30秒に設定）
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  // アクション目標の到達に対するタイムリミット内にアクション結果が受信された場合
  if (finished_before_timeout)
  {
    // アクション目標の状態を受信し、画面に表示する
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // 終了
  return 0;
}
