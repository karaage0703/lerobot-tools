# LeRobot Tools

このリポジトリは[LeRobot](https://github.com/huggingface/lerobot)を使用したロボット制御ツール集です。

> **Note**: このプロジェクトは[Hugging Face LeRobot](https://github.com/huggingface/lerobot)プロジェクトを基盤としています。LeRobotは Apache-2.0 ライセンスの下で公開されています。

## 機能

- Motion Editor: キーボード操作によるポイントtoポイントモーション作成・編集ツール
- uvを使用した仮想環境管理
- ruffによるコード品質管理

## セットアップ

### 1. uvによる環境構築

```bash
uv sync
```

これにより以下がインストールされます：
- lerobot
- lerobot[feetech] (Feetech Servo SDK含む)
- ruff (開発依存関係)

### 2. 仮想環境の有効化

```bash
source .venv/bin/activate
```

または

```bash
uv run <command>
```

## Motion Editor

キーボード操作でポイントtoポイントモーションを作成・編集できるツールです。

### 使用方法

```bash
uv run python motion_editor.py --robot.type=so101_follower --robot.id=lerobot_follower --robot.port=/dev/ttyUSB0
```

### キーボードコマンド

- **WASD**: 各軸の移動 (W/S: shoulder_pan, A/D: shoulder_lift)
- **IJKL**: その他の軸 (I/K: elbow_flex, J/L: wrist_flex)
- **Q/E**: wrist_roll
- **Z/X**: gripper
- **M**: 現在位置をポイントとして記録
- **P**: 記録されたモーションを再生
- **S**: モーションをファイルに保存
- **L**: モーションをファイルから読み込み
- **R**: モーションをリセット（2回押しで確定）
- **ESC**: 終了

## Motion Player

保存されたモーションファイルを再生する独立したツールです。

### 使用方法

#### 基本的な再生

```bash
uv run python motion_player.py --robot.type=so101_follower --robot.id=lerobot_follower \
    --robot.port=/dev/ttyUSB0 --motion=motion_20250813_194018.json
```

#### 利用可能なモーションファイル一覧表示

```bash
uv run python motion_player.py --robot.type=so101_follower --robot.id=lerobot_follower \
    --robot.port=/dev/ttyUSB0 --list
```

#### 再生速度調整

```bash
# 2倍速で再生
uv run python motion_player.py --robot.type=so101_follower --robot.id=lerobot_follower \
    --robot.port=/dev/ttyUSB0 --motion=motion.json --speed=2.0

# 半分の速度で再生
uv run python motion_player.py --robot.type=so101_follower --robot.id=lerobot_follower \
    --robot.port=/dev/ttyUSB0 --motion=motion.json --speed=0.5
```

#### デバッグモード

```bash
uv run python motion_player.py --robot.type=so101_follower --robot.id=lerobot_follower \
    --robot.port=/dev/ttyUSB0 --motion=motion.json --verbose
```

### オプション

- `--motion`: 再生するモーションファイル名
- `--motion-dir`: モーションファイルのディレクトリ（デフォルト: ./motions）
- `--list`: 利用可能なモーションファイル一覧表示
- `--speed`: 再生速度倍率（デフォルト: 1.0）
- `--verbose`: 詳細なデバッグ情報を表示

### 対応ロボット

- so101_follower

### 他のプロジェクトでの活用

Motion PlayerとMotion Utilsは独立したモジュールとして設計されており、他のプロジェクトでサンプルコードとして参考にできます：

- `motion_utils.py`: モーションデータ構造とファイルI/O
- `motion_player.py`: モーション再生ロジック
- `motions/`: JSONフォーマットのモーションデータ

## 開発

### コード品質チェック

```bash
# Lintチェック
uv run ruff check

# 自動修正
uv run ruff check --fix

# フォーマット
uv run ruff format
```

### 設定

ruffの設定は`pyproject.toml`で管理されています：
- line-length: 127
- 基本的なlintルール (E, F, W, I) を有効化

## プロジェクト構造

```
.
├── motion_editor.py     # Motion Editorメインファイル
├── motion_player.py     # Motion Playerメインファイル
├── motion_utils.py      # Motion関連共通ユーティリティ
├── motions/             # 保存されたモーションファイル
├── pyproject.toml       # uv/ruff設定
├── .venv/              # 仮想環境 (uv syncで自動作成)
└── README.md           # このファイル
```

## ライセンス

このプロジェクト自体はMITライセンスの下で公開されています。

### 使用しているプロジェクト

- [LeRobot](https://github.com/huggingface/lerobot): Apache-2.0 ライセンス
- 本プロジェクトで作成したコード: MIT ライセンス

LeRobotライブラリを使用する場合は、Apache-2.0ライセンスの条項に従ってください。