# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(audio_player_cpp)

# C++ 標準の設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 必要なパッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 実行ファイルの追加
add_executable(audio_player_node src/audio_player_node.cpp)

# ターゲットの依存関係を指定
ament_target_dependencies(audio_player_node rclcpp std_msgs)

# インストール設定
install(TARGETS
  audio_player_node
  DESTINATION lib/${PROJECT_NAME})

# パッケージのエクスポート
ament_package()
