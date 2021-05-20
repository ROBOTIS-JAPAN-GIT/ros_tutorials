#include <ros/ros.h>                              // ROSの基本的なヘッダーファイル
#include <actionlib/server/simple_action_server.h>// actionライブラリのヘッダーファイル
#include <ros_tutorials_action/FibonacciAction.h> // Fibonacci.actionファイルをビルドして自動生成されるヘッダーファイル

class FibonacciAction
{
protected:
  // ROSシステムとの通信を行うためのノードハンドルの宣言
  ros::NodeHandle nh_;  
  
  // アクションサーバーの宣言
  actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> as_;

  // アクション名の変数を宣言
  std::string action_name_;

  // パブリッシュのためのアクションフィードバックおよび
  // アクション結果のオブジェクトを宣言
  ros_tutorials_action::FibonacciFeedback feedback_;
  ros_tutorials_action::FibonacciResult result_;

public:
  // アクションサーバーの初期化（ノードハンドル、アクション名、アクションコールバック関数）
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  // アクション目標（Goal）を受信し、指定したアクションを実行する
  void executeCB(const ros_tutorials_action::FibonacciGoalConstPtr &goal)
  {
    // ループ周期を設定する。ここでは周期を1Hz(1秒間隔でのループ)に設定
    ros::Rate r(1);

    // アクションの成功/失敗を表す変数
    bool success = true;

    // フィボナッチ数列の初期化
    // フィードバックの１番目（0）、2番目（1）を追加
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // アクション名、目標、フィボナッチ数列の初めの数値２つを出力
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // アクションの内容
    for(int i=1; i<=goal->order; i++)
    {
      // アクションクライアントからアクションの取り消しを確認
      if (as_.isPreemptRequested() || !ros::ok())
      {
        // アクションの取り消しを通知
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // アクションの取り消し
        as_.setPreempted();
        // アクションを失敗したので、変数にfalseを設定する
        success = false;
        break;
      }
      // アクションを取り消すか、アクションの目標に到達する前にフィードバックに
      // 現在のの数値と以前の数値を足した値を設定する
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // フィードバックをパブリッシュする
      as_.publishFeedback(feedback_);
      // 設定したループ周期に合わせてスリープする
      r.sleep();
    }

    // アクションの目標値を達成した場合、現在のフィボナッチ数列の値を転送する
    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }

};

int main(int argc, char** argv)                     // ノードのメイン関数
{
  ros::init(argc, argv, "action_server");           // ノード名の初期化
  FibonacciAction fibonacci("ros_tutorial_action"); // フィボナッチアクションの宣言
  ros::spin();                                      // アクション目標の受信まで待ち
  return 0;
}
