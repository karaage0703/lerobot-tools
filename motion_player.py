#!/usr/bin/env python3

"""
Motion Player - 保存されたモーションファイルを再生するツール

使用方法:
    python motion_player.py --robot.type=so101_follower --robot.id=lerobot_follower \\
        --robot.port=/dev/ttyUSB0 --motion=motion.json
"""

import logging
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import draccus
from lerobot.robots import (
    RobotConfig,
    make_robot_from_config,
    so101_follower,  # noqa: F401
)
from lerobot.utils.utils import init_logging

from motion_utils import apply_pid_settings, get_home_position, load_motion_from_file

# pynput の import
PYNPUT_AVAILABLE = True
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        logging.info("No DISPLAY set. Skipping pynput import.")
        raise ImportError("pynput blocked intentionally due to no display.")

    from pynput import keyboard
except ImportError:
    keyboard = None
    PYNPUT_AVAILABLE = False
    logging.warning("pynput not available. ESC detection will be disabled.")


@dataclass
class MotionPlayerConfig:
    robot: RobotConfig
    # 再生するモーションファイル
    motion: str = ""
    # モーションファイルディレクトリ
    motion_dir: str = "./motions"
    # 再生速度の倍率 (1.0が通常速度)
    speed: float = 0.5  # デフォルトを0.5倍速（安全のため）
    # デバッグ情報を表示するか
    verbose: bool = False
    # 開始時にホームポジションに移動するか
    go_to_home: bool = True
    # ホーム移動の時間（秒）
    home_duration: float = 3.0
    # モーション終了後にホームポジションに戻るか
    return_to_home: bool = True
    # モーション終了後の待機時間（秒）
    end_hold_time: float = 2.0
    # 最適化PID設定を適用するか
    use_optimized_pid: bool = True


class MotionPlayer:
    """モーション再生クラス"""

    def __init__(self, cfg: MotionPlayerConfig):
        self.cfg = cfg
        self.robot = None
        self.motion = None
        self.should_exit = False

        # ログレベル設定
        if cfg.verbose:
            logging.getLogger().setLevel(logging.DEBUG)

        # ロボット接続
        print(f"🤖 ロボットに接続中... (type: {cfg.robot.type})")
        self.robot = make_robot_from_config(cfg.robot)
        self.robot.connect()
        print("✅ ロボット接続完了")

        # PID設定を適用
        if cfg.use_optimized_pid:
            apply_pid_settings(self.robot, use_optimized=True)
        else:
            apply_pid_settings(self.robot, use_optimized=False)

        # ホームポジションに移動
        if cfg.go_to_home:
            self._go_to_home()

    def _go_to_home(self):
        """ロボットをホームポジションに移動"""
        print("🏠 ホームポジションに移動中...")
        home_position = get_home_position()

        try:
            # 現在位置を表示
            if self.cfg.verbose:
                current_obs = self.robot.get_observation()
                print("現在位置:")
                for joint in home_position.keys():
                    current_pos = current_obs.get(joint, 0.0)
                    print(f"  {joint}: {current_pos:.2f}")

            # ホームポジションに移動
            self.robot.send_action(home_position)
            print(f"⏱️  ホーム移動時間: {self.cfg.home_duration}秒")
            time.sleep(self.cfg.home_duration)
            print("✅ ホームポジション到達")

        except Exception as e:
            print(f"❌ ホーム移動エラー: {e}")

    def _on_key_press(self, key):
        """キー押下イベント処理"""
        try:
            if key == keyboard.Key.esc:
                print("\n🔚 ESCキーが押されました - 終了します")
                self.should_exit = True
                return False  # リスナーを停止
        except AttributeError:
            pass

    def _hold_position_until_esc(self):
        """ESCキーが押されるまで現在位置を保持"""
        print("🔒 ホームポジション保持中...")
        print("   ESCキーを押すと終了します")

        if not PYNPUT_AVAILABLE:
            print("   (pynput利用不可 - 5秒後に自動終了)")
            time.sleep(5)
            return

        # キーボードリスナーを開始
        listener = keyboard.Listener(on_press=self._on_key_press)
        listener.start()

        try:
            home_position = get_home_position()
            while not self.should_exit:
                # 定期的にホームポジションを送信してモーター保持
                self.robot.send_action(home_position)
                time.sleep(0.5)  # 0.5秒間隔で保持

        except KeyboardInterrupt:
            print("\n🔚 Ctrl+Cで終了します")
        finally:
            listener.stop()

    def load_motion(self, motion_file: str) -> bool:
        """モーションファイルを読み込み"""
        # motion_fileが既にパスを含んでいる場合はそのまま使用
        if "/" in motion_file or "\\" in motion_file:
            motion_path = Path(motion_file)
        else:
            motion_path = Path(self.cfg.motion_dir) / motion_file

        if not motion_path.exists():
            print(f"❌ モーションファイルが見つかりません: {motion_path}")
            return False

        try:
            self.motion = load_motion_from_file(str(motion_path))
            print(f"📖 モーション読み込み完了: {self.motion.name}")
            print(f"   - ポイント数: {len(self.motion.points)}")
            print(f"   - 作成日時: {self.motion.created_at}")
            print(f"   - ロボット型: {self.motion.robot_type}")

            if self.motion.robot_type != self.cfg.robot.type:
                print(f"⚠️ 警告: モーションのロボット型 ({self.motion.robot_type}) と設定 ({self.cfg.robot.type}) が異なります")

            return True
        except Exception as e:
            print(f"❌ モーション読み込みエラー: {e}")
            return False

    def play_motion(self):
        """モーションを再生"""
        if not self.motion:
            print("❌ モーションが読み込まれていません")
            return False

        if not self.motion.points:
            print("❌ モーションポイントがありません")
            return False

        print(f"▶️ モーション再生開始: {self.motion.name}")
        print(f"   速度倍率: {self.cfg.speed}x")
        print("   (Ctrl+C で中断)")

        try:
            for i, point in enumerate(self.motion.points, 1):
                print(f"📍 ポイント {i}/{len(self.motion.points)}: {point.name}")

                # デバッグ情報
                if self.cfg.verbose:
                    for joint, pos in point.positions.items():
                        print(f"     {joint}: {pos:.2f}")

                # ロボットを目標位置に移動
                self.robot.send_action(point.positions)

                # 移動時間待機
                time.sleep(point.duration / self.cfg.speed)

                # 移動完了を待機
                time.sleep(0.1)

            print("✅ モーション再生完了")

            # モーション終了後の処理
            if self.cfg.return_to_home:
                print(f"⏱️  終了位置で{self.cfg.end_hold_time}秒待機...")
                time.sleep(self.cfg.end_hold_time)
                print("🏠 ホームポジションに戻り中...")
                self._go_to_home()
                # ホームポジション到達後、ESCまで保持
                self._hold_position_until_esc()
            else:
                print(f"⏸️  現在位置で{self.cfg.end_hold_time}秒待機...")
                time.sleep(self.cfg.end_hold_time)

            return True

        except KeyboardInterrupt:
            print("\n⏸️ モーション再生が中断されました")
            return False
        except Exception as e:
            print(f"❌ モーション再生エラー: {e}")
            return False

    def run(self):
        """メイン実行"""
        try:
            # モーションファイルチェック
            if not self.cfg.motion:
                print("❌ --motion でモーションファイルを指定してください")
                return

            # モーション読み込みと再生
            if self.load_motion(self.cfg.motion):
                self.play_motion()

        finally:
            # ロボット切断
            if self.robot and hasattr(self.robot, "is_connected") and self.robot.is_connected:
                try:
                    self.robot.disconnect()
                    print("🔌 ロボットから切断しました")
                except Exception as e:
                    print(f"⚠️ ロボット切断時にエラー: {e}")


@draccus.wrap()
def main(cfg: MotionPlayerConfig):
    """メイン関数"""
    init_logging()

    # モーションディレクトリ作成
    Path(cfg.motion_dir).mkdir(exist_ok=True)

    player = MotionPlayer(cfg)
    player.run()


if __name__ == "__main__":
    main()
