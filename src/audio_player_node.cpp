// src/audio_player_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib> // for system()
#include <string>
#include <sstream>

class AudioPlayerNode : public rclcpp::Node
{
public:
    AudioPlayerNode() : Node("audio_player_node")
    {
        // トピック名を '/play_audio' に設定
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "play_audio",
            10,
            std::bind(&AudioPlayerNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Audio Player Node has been started and is listening to /play_audio topic.");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string audio_path = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received request to play audio: '%s'", audio_path.c_str());

        // 音声ファイルの存在確認
        if (file_exists(audio_path))
        {
            // 音声ファイルの拡張子に応じて再生コマンドを選択
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

            // 音声再生コマンドの実行
            int ret = system(command.c_str());
            if (ret == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute command: '%s'", command.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Playing audio: '%s'", audio_path.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Audio file does not exist: '%s'", audio_path.c_str());
        }
    }

    bool file_exists(const std::string &path)
    {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }

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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioPlayerNode>());
    rclcpp::shutdown();
    return 0;
}
