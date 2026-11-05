import rp2
import time

def is_bootsel_pressed():
    # BOOTSELボタンの状態を取得 (押されていると1、離すと0)
    # 内部的にフラッシュメモリのCSラインをチェックしています
    return rp2.bootsel_button() == 1

print("Wait for BOOTSEL...")

# メインループの前にボタンを待つ
while True:
    if is_bootsel_pressed():
        print("BOOTSEL pressed.")
        break
    time.sleep(0.1)

# ここからメインの処理
print("メインプログラム実行中...")