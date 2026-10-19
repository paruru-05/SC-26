import machine,framebuf
from ssd1306 import SSD1306_I2C

class SSD1306(SSD1306_I2C):
    def __init__(self, i2c: machine.I2C, width: int = 128, height: int = 64):
        # 親クラス(SSD1306_I2C)を初期化
        super().__init__(width, height, i2c)
        
        # テストコード側の ssd.oled 呼び出しとの互換性を保持
        self.oled = self
        
        self.y = 0
        self.reset()
        
        # 初期タイトル表示
        self.text("==== Pico2W ====")
        self.update()

    def update(self):
        self.show()

    def text(self, txt: str):
        # 親クラスの text(string, x, y, col) を呼び出し
        super().text(txt, 0, self.y, 1)
        self.y += 20
        # 画面をはみ出す場合はy座標を上に戻す
        if self.y >= self.height:
            self.y = 0

    def reset(self):
        self.fill(0)
        self.y = 0

    def line(self, x1: int, y1: int, x2: int, y2: int, width: int = 1):
        if width <= 1:
            super().line(x1, y1, x2, y2, 1)
            return

        dx, dy = abs(x2 - x1), abs(y2 - y1)
        half = width // 2
        if dx >= dy:
            for i in range(-half, half + 1):
                super().line(x1, y1 + i, x2, y2 + i, 1)
        else:
            for i in range(-half, half + 1):
                super().line(x1 + i, y1, x2 + i, y2, 1)

    def rect(self, x1: int, y1: int, x2: int, y2: int, fill: bool = False):
        w = abs(x1 - x2) + 1
        h = abs(y1 - y2) + 1
        x = min(x1, x2)
        y = min(y1, y2)
        if fill:
            self.fill_rect(x, y, w, h, 1)
        else:
            super().rect(x, y, w, h, 1)
            
    def draw_center_text(self, text: str, char_width: int = 8):
        """
        ディスプレイの中央にテキストを描画するクラスメソッド
        (SSD1306クラスの中にそのままコピペして使用できます)
        
        :param text: 描画する文字列
        :param y: 描画するY座標
        :param char_width: 1文字あたりの幅 (MicroPython標準フォントは8px)
        """
        # 文字列全体のピクセル幅を計算
        text_width = len(text) * char_width
        
        # self.width (画面幅: 128等) から中央揃えになるX座標を計算
        # 画面からはみ出る場合は0で固定
        width = getattr(self, 'width', 128)
        x = max(0, (width - text_width) // 2)
        
        super().text(text, x, self.y, 1)
        self.y += 20
        # 画面をはみ出す場合はy座標を上に戻す
        if self.y >= self.height:
            self.y = 0
            
    def draw_bitmap(self, bitmap_data: bytes, width: int, height: int, x: int, y: int, key: int = -1, format=framebuf.MONO_HLSB):
        """
        指定したバイト配列データ(ドット絵)を FrameBuffer 経由でディスプレイに描画する関数
        
        :param display: SSD1306等のFrameBuffer継承/保有オブジェクト
        :param bitmap_data: ドット絵のバイトデータ (bytes や bytearray)
        :param width: ドット絵の幅 (ピクセル)
        :param height: ドット絵の高さ (ピクセル)
        :param x: 描画先のX座標
        :param y: 描画先のY座標
        :param key: 透過色 (-1で透過なし、0で黒を透過、1で白を透過)
        :param format: フォーマット (デフォルト: framebuf.MONO_HLSB)
        """
        # バイトデータから一時的な FrameBuffer オブジェクトを作成
        fb = framebuf.FrameBuffer(bytearray(bitmap_data), width, height, format)
        
        # blit メソッドでメインディスプレイのバッファに転送
        if key >= 0:
            self.blit(fb, x, y, key)  # 透過色指定あり
        else:
            self.blit(fb, x, y)       # 上書き転送
