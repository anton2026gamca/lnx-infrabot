from time import sleep
from gpiozero import Button

from PIL import Image, ImageDraw, ImageFont

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

# ---------------- GPIO ----------------

button = Button(25, pull_up=True)

# GPIO 22: Up
# GPIO 23: Down
# GPIO 24: Enter/Select
# GPIO 25: Back/Exit

# ---------------- DISPLAY ----------------

LINE_MARGIN = 3
LINE_HEIGHT = 7 + LINE_MARGIN

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=32)

font = ImageFont.truetype("font.ttf", 8)

LIGATURES = [
    "==",
    "===",
    "!=",
    "!==",
    "<=",
    ">=",
    "<-",
    "->",
    "=>",
    "<=>",
    "<!--",
    "==>",
    "<==",
    "-->",
    "<--",
    # "TODO:",
    # "NOTE:",
]

def draw_mono_text(draw, pos, text, font, fill=255, spacing=1):
    x, y = pos

    normal_width = font.getbbox("A")[2]

    i = 0
    while i < len(text):
        matched = False

        for lig in sorted(LIGATURES, key=len, reverse=True):
            if text.startswith(lig, i):
                draw.text((x, y), lig, font=font, fill=fill)

                x += (normal_width + spacing) * len(lig)
                i += len(lig)
                matched = True
                break

        if not matched:
            char = text[i]

            draw.text((x, y), char, font=font, fill=fill)

            x += normal_width + spacing
            i += 1

# ---------------- MAIN LOOP ----------------

last_state = None

try:
    while True:
        pressed = button.is_pressed

        if pressed != last_state:
            print(f"Button is {'pressed' if pressed else 'released'}")

            image = Image.new("1", (128, 32))
            draw = ImageDraw.Draw(image)

            if not pressed:
                draw_mono_text(
                    draw,
                    (0, LINE_MARGIN),
                    "LNX InfraBot",
                    font
                )

                draw_mono_text(
                    draw,
                    (0, LINE_HEIGHT * 1 + LINE_MARGIN),
                    "abcdefghijklm |  |",
                    font
                )

                draw_mono_text(
                    draw,
                    (0, LINE_HEIGHT * 2 + LINE_MARGIN),
                    "TODO: ===JKLM |->|- > ->",
                    font
                )

                draw_mono_text(
                    draw,
                    (0, LINE_HEIGHT * 3 + LINE_MARGIN),
                    "NOTE: != != != != != != != != != !=",
                    font
                )

            device.display(image)

            last_state = pressed

        sleep(0.01)

except KeyboardInterrupt:
    pass
