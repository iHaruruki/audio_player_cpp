# audio_player_cpp
## Discription
## Install
```

```
## How to try
```
# 環境設定
$ source ~/ros2_ws/install/setup.bash
# ノードの実行
$ ros2 run audio_player_cpp audio_player_node
# 環境設定
$ source ~/ros2_ws/install/setup.bash
# 音声再生のリクエスト（例: WAV ファイル）
$ ros2 topic pub /play_audio std_msgs/String "data: '/absolute/path/to/your/sample_audio.wav'"
# または MP3 ファイルの場合
$ ros2 topic pub /play_audio std_msgs/String "data: '/absolute/path/to/your/sample_audio.mp3'"
```
