import struct
import random

def reconstruct_highend_filtered(input_bin, output_raw, power=4.0, dither_amt=0.002):
    with open(input_bin, "rb") as f:
        data = f.read()

    raw_data = bytearray()
    MAX_IN = 4095.0
    MAX_OUT = 32767.0
    
    prev_sample = 0 # A szűrőhöz

    for i in range(0, len(data) - 1, 2):
        msb = data[i]
        lsb = data[i+1]
        
        is_neg = bool((msb >> 7) & 1)
        
        if not (lsb & 0x40): 
            final_sample = 0
        else:
            # Eredeti logika
            linear_val = ((msb & 0x7F) << 5) | ((lsb >> 1) & 0x1F)
            norm = linear_val / MAX_IN
            
            # 1. DITHERING: pici zaj hozzáadása a normalizált értékhez
            # Ez segít "kitölteni" a huplikat
            dither = random.uniform(-dither_amt, dither_amt)
            norm = max(0, min(1.0, norm + dither))
            
            # ... a ciklus belsejében ...
            expanded = pow(norm, 3.0) # Finomabb görbe
            
            magnitude = expanded * MAX_OUT
            val = -magnitude if is_neg else magnitude
            
            # Puhább simítás a mélyeknek
            smoothed_val = (val * 0.7) + (prev_sample * 0.3)
            prev_sample = smoothed_val
            
            final_sample = int(round(smoothed_val))
            final_sample = max(-32768, min(32767, final_sample))
            
        raw_data.extend(struct.pack('<h', final_sample))

    with open(output_raw, "wb") as f:
        f.write(raw_data)
    
    print(f"SZŰRT ÉS DITHERELT KÉSZ | Power: {power}")

# Próbáld ki ezzel!
reconstruct_highend_filtered("Roland-D50-PCM-ROM-A-IC30.bin", "D50_PRO_CLEAN2.raw", power=4.0)