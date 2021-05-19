# ビルドを行うために最低限必要なCMakeのバージョンを記述する
cmake_minimum_required(VERSION 2.8.3)
# パッケージ名を記述する
project(ros_tutorials_parameter)

# catkinビルドの実行時に要求される依存パッケージを記述する
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

# サービスファイルを記述する
add_service_files(FILES SrvTutorial.srv)
# 依存するメッセージファイルを記述する
generate_messages(DEPENDENCIES std_msgs)

# catkinビルドのオプションを記述する
# LIBRARIES オプションでros_tutorials_parameterパッケージにあるライブラリを参照する
# CATKIN_DEPENDS オプションでstd_msgs,roscppへの依存関係を設定する
catkin_package(
  LIBRARIES ros_tutorials_parameter
  CATKIN_DEPENDS std_msgs roscpp
)

# インクルードフォルダを記述する
include_directories(${catkin_INCLUDE_DIRS})

# ビルド後に生成する実行ファイルを記述する
add_executable(service_server_with_parameter src/service_server.cpp)
# ライブラリや実谷ファイルを生成する前に生成する依存関係メッセージを記述する
add_dependencies(service_server_with_parameter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 実行ファイルを生成する際にリンクが必要なライブラリを記述する
target_link_libraries(service_server_with_parameter ${catkin_LIBRARIES})

# ビルド後に生成する実行ファイルを記述する
add_executable(service_client_with_parameter src/service_client.cpp)
# ライブラリや実谷ファイルを生成する前に生成する依存関係メッセージを記述する
add_dependencies(service_client_with_parameter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 実行ファイルを生成する際にリンクが必要なライブラリを記述する
target_link_libraries(service_client_with_parameter ${catkin_LIBRARIES})
