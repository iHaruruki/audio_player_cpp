// src/audio_player_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>      // system() 用
#include <string>
#include <sys/stat.h>   // stat() 用
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

class AudioPlayerNode : public rclcpp::Node
{
public:
    AudioPlayerNode() : Node("audio_player_node"), shutdown_flag_(false)
    {
        // "/play_audio" トピックを購読（メッセージは音声ファイルのパス）
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "play_audio",
            10,
            std::bind(&AudioPlayerNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Audio Player Node started, listening on /play_audio topic.");

        // 再生リクエストを処理する専用スレッドを起動
        playback_thread_ = std::thread(&AudioPlayerNode::playback_loop, this);
    }

    ~AudioPlayerNode()
    {
        {
            // shutdown_flag_ を true にしてループ終了を促す
            std::lock_guard<std::mutex> lock(queue_mutex_);
            shutdown_flag_ = true;
        }
        queue_cond_.notify_all();
        if (playback_thread_.joinable()) {
            playback_thread_.join();
        }
    }

private:
    // トピックから音声ファイルパスを受信したときのコールバック
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string audio_path = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received request to play audio: '%s'", audio_path.c_str());

        if (file_exists(audio_path))
        {
            // 再生リクエストをキューに追加
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

    // 専用スレッド内でキューの再生リクエストを順次処理するループ
    void playback_loop()
    {
        while (true)
        {
            std::string next_audio;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                // キューに要素があるか、または shutdown_flag_ が true になるまで待つ
                queue_cond_.wait(lock, [this]{
                    return !playback_queue_.empty() || shutdown_flag_;
                });

                // shutdown_flag_ が立ち、かつキューが空の場合はループを抜ける
                if (shutdown_flag_ && playback_queue_.empty()) {
                    break;
                }
                next_audio = playback_queue_.front();
                playback_queue_.pop();
            }
            // 1件分の音声再生を実行（再生が終わるまで system() がブロッキングする）
            play_audio(next_audio);
        }
    }

    // 指定された音声ファイルの再生を実行する関数
    void play_audio(const std::string &audio_path)
    {
        std::string command;
        if (has_suffix(audio_path, ".wav"))
        {
            command = "aplay \"" + audio_path + "\"";
        }
        else if (has_suffix(audio_path, ".mp3"))
        {
            command = "mpg123 \"" + audio_path + "\"";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported audio format: '%s'", audio_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Playing audio: '%s'", audio_path.c_str());
        int ret = system(command.c_str());
        if (ret == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute command: '%s'", command.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Finished playing audio: '%s'", audio_path.c_str());
        }
    }

    // 指定されたパスにファイルが存在するか確認する関数
    bool file_exists(const std::string &path)
    {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }

    // 文字列が指定されたサフィックスで終わるか確認する関数
    bool has_suffix(const std::string &str, const std::string &suffix)
    {
        if (str.length() >= suffix.length())
        {
            return (0 == str.compare(str.length() - suffix.length(), suffix.length(), suffix));
        }
        else
        {
            return false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::queue<std::string> playback_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cond_;
    std::thread playback_thread_;
    bool shutdown_flag_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioPlayerNode>());
    rclcpp::shutdown();
    return 0;
}
