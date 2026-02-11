import struct
import math

def atomic_scope(input_raw, output_txt, start, end, width=80):
    blocks = [" ", "▏", "▎", "▍", "▌", "▋", "▊", "▉"]
    
    try:
        with open(input_raw, "rb") as f:
            f.seek(start * 2)
            num_samples = (end - start) + 1
            raw_data = f.read(num_samples * 2)
            samples = struct.unpack(f'<{num_samples}h', raw_data)

        with open(output_txt, "w", encoding="utf-8") as out:
            out.write(f"D-50 ATOMIC SCOPE | Tartomány: {start} - {end}\n")
            out.write(f"LOG-DYNAMIC SCALE: A legkisebb különbség is látszik!\n")
            out.write("-" * (width * 2 + 30) + "\n")

            for i, val in enumerate(samples):
                idx = start + i
                abs_val = abs(val)
                
                if abs_val == 0:
                    bar_str = ""
                else:
                    # Logaritmikus/Dinamikus sűrítés, hogy a 10 és a 100 ne legyen ugyanaz
                    # A math.log használatával a kis értékek "helyet kapnak" a skálán
                    scaled = math.sqrt(abs_val / 32768.0) * (width * 8)
                    total_subpixels = max(1, int(scaled))
                    
                    full_chars = total_subpixels // 8
                    fraction = total_subpixels % 8
                    bar_str = "█" * full_chars + blocks[fraction]
                
                if val == 0:
                    line = " " * width + "║ (CSEND)"
                elif val > 0:
                    line = " " * width + "║" + bar_str
                else:
                    line = " " * (width - len(bar_str)) + bar_str + "║"

                out.write(f"{idx:<7} | {val:>7} | {line}\n")

        print(f"Az Atom-Szkóp elkészült: {output_txt}")

    except Exception as e:
        print(f"Hiba: {e}")

# Futtasd le a kritikus 2750-2850 szakaszon!
atomic_scope("D50_MASTER_EXPAND.raw", "atomic_view.txt", 118000, 200000)
