import struct

def make_my_array(name, start, end, input_file="D50_PRO_CLEAN2.raw"):
    try:
        with open(input_file, "rb") as f:
            # Kiszámoljuk a pozíciókat (16-bit = 2 bájt mintánként)
            f.seek(start * 2)
            num_samples = (end - start) + 1
            raw_data = f.read(num_samples * 2)
            
            # Kicsomagoljuk a nyers bájtokat 16-bites számokká
            samples = struct.unpack(f'<{num_samples}h', raw_data)
            
            # Összefűzzük egyetlen hosszú, vesszővel elválasztott sorrá
            data_string = ", ".join(map(str, samples))
            
            # Elmentjük egy névvel ellátott fájlba
            filename = f"{name}_array.txt"
            with open(filename, "w") as out:
                out.write(f"const int16_t {name}[{num_samples}] = {{ {data_string} }};")
            
            print(f"--- SIKER ---")
            print(f"Hangszer: {name}")
            print(f"Tartomány: {start} - {end} ({num_samples} minta)")
            print(f"Fájl mentve: {filename}")
            print("-" * 15)

    except Exception as e:
        print(f"Hiba történt: {e}")

# ==========================================================
# ITT ADD MEG A PARAMÉTEREKET: (Név, Alsó index, Felső index)
# ==========================================================

# Példa: a te Marimbád a RAW fájlban lévő valódi indexek alapján
 
#make_my_array("bells", 40960, 44965 )
#make_my_array("nylonstrings", 94208, 98033 )
#make_my_array("electgitar1", 98304, 101919 )
#make_my_array("electgitar2", 102403, 106301 )
#make_my_array("electgitar2", 102403, 106301 )
#make_my_array("dirtygitar", 106496, 110452 )
#make_my_array("pickbass", 110592 , 114630 )
#make_my_array("popbass", 114688 , 118756 )
make_my_array("thump", 118784   , 122849 ) 
make_my_array("uprightbass", 122881 ,126965) 
make_my_array("klarinet", 126977   , 131028) 
make_my_array("breath", 131072  , 135167  ) 
make_my_array("steamer", 135168   , 138852   ) 
make_my_array("hightflute",139265 , 143211  ) 
make_my_array("lowflute",143360  , 146788   ) 





# Ha megvan a Vibraphone és a Xylophone pontos vége a RAW-ban, 
# csak írd be alájuk és futtasd újra:
# make_my_array("vibraphone", 0, 4097)
