// src/audio_player_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// 再生プロセス制御用ヘッダ
#include <unistd.h>     // fork(), execlp(), kill(), etc.
#include <sys/types.h>
#include <sys/wait.h>   // waitpid()
#include <signal.h>     // SIGTERM

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <sys/stat.h>   // stat()

class AudioPlayerNode : public rclcpp::Node
{
public:
  AudioPlayerNode()
    : Node("audio_player_node"), shutdown_flag_(false), current_playback_pid_(-1)
  {
    // "/play_audio" トピックを購読（メッセージ内容は音声ファイルのパス）
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "play_audio",
      10,
      std::bind(&AudioPlayerNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Audio Player Node started, listening on /play_audio topic.");

    // 再生リクエストを順次処理する専用スレッドを起動
    playback_thread_ = std::thread(&AudioPlayerNode::playback_loop, this);
  }

  ~AudioPlayerNode()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      shutdown_flag_ = true;
      // 再生中の子プロセスがあれば終了シグナルを送る
      if (current_playback_pid_ != -1)
      {
        RCLCPP_INFO(this->get_logger(), "Terminating playback process %d", current_playback_pid_);
        kill(current_playback_pid_, SIGTERM);
      }
    }
    queue_cond_.notify_all();
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
  }

private:
  // トピックからの再生リクエストコールバック
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string audio_path = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received request to play audio: '%s'", audio_path.c_str());

    if (file_exists(audio_path))
    {
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        playback_queue_.push(audio_path);
        RCLCPP_INFO(this->get_logger(), "Queued audio file: '%s'", audio_path.c_str());
      }
      queue_cond_.notify_one();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Audio file does not exist: '%s'", audio_path.c_str());
    }
  }

  // 専用スレッド内で、キューの再生リクエストを順次処理するループ
  void playback_loop()
  {
    while (true)
    {
      std::string next_audio;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cond_.wait(lock, [this]{
          return !playback_queue_.empty() || shutdown_flag_;
        });

        if (shutdown_flag_ && playback_queue_.empty()) {
          break;
        }
        next_audio = playback_queue_.front();
        playback_queue_.pop();
      }
      // 1件分の音声再生を実行
      play_audio(next_audio);
    }
  }

  // 子プロセスを用いて音声ファイルを再生する関数
  void play_audio(const std::string &audio_path)
  {
    pid_t pid = fork();
    if (pid < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to fork process for audio playback.");
      return;
    }
    else if (pid == 0)
    {
      // 子プロセス: 指定された再生コマンドを実行
      if (has_suffix(audio_path, ".wav"))
      {
        execlp("aplay", "aplay", audio_path.c_str(), (char*)NULL);
      }
      else if (has_suffix(audio_path, ".mp3"))
      {
        execlp("mpg123", "mpg123", audio_path.c_str(), (char*)NULL);
      }
      // exec が失敗した場合は子プロセスを終了
      _exit(1);
    }
    else
    {
      // 親プロセス: 再生中の子プロセスの PID を記録し、終了を待つ
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        current_playback_pid_ = pid;
      }
      int status = 0;
      while (true)
      {
        pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == 0)
        {
          // 子プロセスはまだ動作中
          {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (shutdown_flag_)
            {
              RCLCPP_INFO(this->get_logger(), "Shutdown requested. Terminating playback process %d", pid);
              kill(pid, SIGTERM);
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (result == pid)
        {
          // 子プロセスが終了
          break;
        }
        else
        {
          // エラーの場合
          break;
        }
      }
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        current_playback_pid_ = -1;
      }
      RCLCPP_INFO(this->get_logger(), "Finished playing audio: '%s'", audio_path.c_str());
    }
  }

  // 指定されたパスにファイルが存在するか確認する関数
  bool file_exists(const std::string &path)
  {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
  }

  // 文字列が指定されたサフィックス（拡張子）で終わるか確認する関数
  bool has_suffix(const std::string &str, const std::string &suffix)
  {
    if (str.length() >= suffix.length())
    {
      return (0 == str.compare(str.length() - suffix.length(), suffix.length(), suffix));
    }
    return false;
  }

  // メンバ変数
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::queue<std::string> playback_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_;
  std::thread playback_thread_;
  bool shutdown_flag_;
  pid_t current_playback_pid_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioPlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
