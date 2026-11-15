from pathlib import Path

from PIL import Image, ImageOps


SOURCE = Path(r"C:\Users\14449\Desktop\梦之队队徽.jpg")
OUTPUT = Path(r"E:\Engineering_Training_Project\Core\Src\dream_team_logo.c")
SIZE = 180


def main() -> None:
    image = Image.open(SOURCE).convert("L")
    image = image.resize((SIZE, SIZE), Image.Resampling.LANCZOS)
    image = ImageOps.autocontrast(image)
    monochrome = image.point(lambda value: 255 if value >= 145 else 0, mode="1")
    pixels = list(monochrome.getdata())

    lines = []
    for row in range(SIZE):
        row_bits = pixels[row * SIZE:(row + 1) * SIZE]
        row_bytes = bytearray((SIZE + 7) // 8)
        for column, value in enumerate(row_bits):
            if value:
                row_bytes[column // 8] |= 0x80 >> (column % 8)
        rows = [f"0x{byte:02X}U" for byte in row_bytes]
        lines.append("    " + ", ".join(rows) + ",")

    body = "\n".join(lines).rstrip(",")
    OUTPUT.write_text(
        "#include \"dream_team_logo.h\"\n\n"
        "const uint8_t DreamTeamLogoBits[] = {\n"
        f"{body}\n"
        "};\n",
        encoding="ascii",
    )


if __name__ == "__main__":
    main()
