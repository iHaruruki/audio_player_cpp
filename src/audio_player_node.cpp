// src/audio_player_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>      // for system()
#include <string>
#include <sys/stat.h>   // for stat()
#include <thread>       // for std::thread
#include <mutex>        // for std::mutex
#include <vector>       // for std::vector

class AudioPlayerNode : public rclcpp::Node
{
public:
    AudioPlayerNode() : Node("audio_player_node")
    {
        // '/play_audio' トピックを購読
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "play_audio",
            10,
            std::bind(&AudioPlayerNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Audio Player Node has been started and is listening to /play_audio topic.");
    }

    ~AudioPlayerNode()
    {
        // 全ての再生スレッドが終了するのを待つ
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto &t : threads_)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string audio_path = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received request to play audio: '%s'", audio_path.c_str());

        // 音声ファイルの存在確認
        if (file_exists(audio_path))
        {
            // 非同期で音声を再生
            std::lock_guard<std::mutex> lock(mutex_);
            threads_.emplace_back(&AudioPlayerNode::play_audio, this, audio_path);
            RCLCPP_INFO(this->get_logger(), "Started asynchronous playback for: '%s'", audio_path.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Audio file does not exist: '%s'", audio_path.c_str());
        }
    }

    // 音声再生を別スレッドで実行
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

        RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());
        int ret = system(command.c_str());
        if (ret == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute command: '%s'", command.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Finished playing audio: '%s'", audio_path.c_str());
        }

        // スレッドが終了したら削除するために mutex を使用
        std::lock_guard<std::mutex> lock(mutex_);
        // スレッドが終了した後は特に操作は不要
    }

    // ファイル存在確認関数
    bool file_exists(const std::string &path)
    {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }

    // サフィックス確認関数
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
    std::vector<std::thread> threads_;
    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioPlayerNode>());
    rclcpp::shutdown();
    return 0;
}
