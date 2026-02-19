# make_c_array.py
data = open("imitation_model_float32.tflite", "rb").read()

with open("model_data.h", "w", encoding="utf-8") as f:
    f.write("#pragma once\n")
    f.write("#include <cstdint>\n\n")
    f.write("alignas(16) const unsigned char g_model[] = {\n")

    for i, b in enumerate(data):
        if i % 12 == 0:
            f.write("  ")
        f.write(f"0x{b:02x}, ")
        if i % 12 == 11:
            f.write("\n")

    f.write("\n};\n")
    f.write(f"const unsigned int g_model_len = {len(data)};\n")

print(f"Wrote model_data.h with {len(data)} bytes")
