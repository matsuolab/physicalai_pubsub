# Physical AI PubSub パッケージ

CRANE+ V2ロボットを使用したROS2のトピック通信を行うためのパッケージです．

## 機能

- Subscriberによる関節状態の受信
- Publisherによるロボットへの行動指示

## インストール方法
1. 環境構築

以下のリポジトリに従ってDocker環境構築 

https://github.com/AI-Robot-Book-Humble/docker-ros2-desktop-ai-robot-book-humble

GazeboでCRANE+ V2 マニピュレータの起動
```bash
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

2. リポジトリをROS2ワークスペースにクローン:

```bash
cd ~/pai_ws/src
git clone https://github.com/matsuolab/physicalai_pubsub.git
```

3. 依存関係のインストール:

```bash
cd ~/pai_ws
rosdep install --from-paths src --ignore-src -r -y
```


4. パッケージのビルド:

```bash
colcon build --symlink-install --packages-select physicalai_pubsub
```

5. ワークスペースのソース:

```bash
source install/setup.bash
```

## 使用方法

### ロボットの関節状態をSubscriberで受信

```bash
ros2 run physicalai_pubsub state_subscriber
```

### ロボットにPublisherでターゲット関節角を指示

一連の動作の実行:
```bash
ros2 run physicalai_pubsub action_publisher
```

## ライセンス

このソフトウェアはMITライセンスの下で公開されています． 詳細は[LICENSE](LICENSE)ファイルを参照してください。

## 作者

上條 達也 (tatsukamijo@ieee.org)
