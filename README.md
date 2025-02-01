# audio_player_cpp
## Discription
Subscribeした音声を再生するNode
## Node and Topic
```mermaid
flowchart LR
    A(["/play_audio"]) ==> B["/audio_player_node"]
```
## Install
```
$ sudo apt-get update
$ sudo apt-get install alsa-utils   #WAVファイル用
$ sudo apt-get install mpg123       #MP3ファイル用
```
## How to try
```
# 環境設定
$ source ~/ros2_ws/install/setup.bash
# ノードの実行
$ ros2 run audio_player_cpp audio_player_node

# **トピックにメッセージを送信**
# 音声再生のリクエスト（例: WAV ファイル）
$ ros2 topic pub /play_audio std_msgs/String "data: '/absolute/path/to/your/sample_audio.wav'"
# または MP3 ファイルの場合
$ ros2 topic pub /play_audio std_msgs/String "data: '/absolute/path/to/your/sample_audio.mp3'"
```
