# ビルドを行うために最低限必要なCMakeのバージョンを記述する
cmake_minimum_required(VERSION 2.8.3)
# パッケージ名を記述する
project(ros_tutorials_action)

# catkinビルドの実行時に要求される依存パッケージを記述する
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  actionlib
  roscpp
)

# Boostライブラリのsystemを依存パッケージに追加する
find_package(Boost REQUIRED COMPONENTS system)

# アクションファイルを記述する
add_action_files(FILES Fibonacci.action)
# 依存するメッセージファイルを記述する
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

# catkinビルドのオプションを記述する
# LIBRARIES オプションでros_tutorials_actionパッケージにあるライブラリを参照する
# CATKIN_DEPENDS オプションでstd_msgs,roscppへの依存関係を設定する
# DEPENDS オプションでBoostへの依存関係を設定する
catkin_package(
  LIBRARIES ros_tutorials_action
  CATKIN_DEPENDS std_msgs actionlib_msgs actionlib roscpp
  DEPENDS Boost
)

# インクルードフォルダを記述する
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# ビルド後に生成する実行ファイルを記述する
add_executable(action_server src/action_server.cpp)
# ライブラリや実谷ファイルを生成する前に生成する依存関係メッセージを記述する
add_dependencies(action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 実行ファイルを生成する際にリンクが必要なライブラリを記述する
target_link_libraries(action_server ${catkin_LIBRARIES})

# ビルド後に生成する実行ファイルを記述する
add_executable(action_client src/action_client.cpp)
# ライブラリや実谷ファイルを生成する前に生成する依存関係メッセージを記述する
add_dependencies(action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 実行ファイルを生成する際にリンクが必要なライブラリを記述する
target_link_libraries(action_client ${catkin_LIBRARIES})
