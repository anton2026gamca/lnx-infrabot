from __future__ import annotations

from PIL import ImageDraw, ImageFont

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


def draw_mono_text(
    draw: ImageDraw.ImageDraw,
    pos: tuple[int, int],
    text: str,
    font: ImageFont.FreeTypeFont,
    fill: int = 255,
    spacing: int = 1,
) -> None:
    x, y = pos

    normal_width = 5 #font.getbbox("A")[2]

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
