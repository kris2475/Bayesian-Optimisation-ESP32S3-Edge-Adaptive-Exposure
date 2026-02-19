import matplotlib.pyplot as plt
import numpy as np
import os

def display_pgm():
    folder_name = 'pgm_files'
    
    if not os.path.exists(folder_name):
        print(f"Error: Folder '{folder_name}' not found.")
        return

    # List files and sort them so they appear in the order you generated them
    files = sorted([f for f in os.listdir(folder_name) if f.lower().endswith('.pgm')])
    
    if not files:
        print("No files found.")
        return

    for i, filename in enumerate(files, start=1):
        print(f"{i}. {filename}")
    
    try:
        selection = input("\nEnter file number to view (or 'q' to quit): ")
        if selection.lower() == 'q': return
        
        selected_file = files[int(selection) - 1]
        path = os.path.join(folder_name, selected_file)
        file_size = os.path.getsize(path)
        
        with open(path, 'rb') as f:
            # Look at the first two bytes for a PGM Magic Number
            magic = f.read(2)
            f.seek(0) # Reset pointer
            
            # CASE 1: Standard PGM Header found (P5 or P2)
            if magic in [b'P5', b'P2']:
                print(f"Standard {magic.decode()} header detected.")
                # We'll use matplotlib's native reader as a backup for standard files
                pixels = plt.imread(path)
                
            # CASE 2: No Header (Raw Sensor Dump - 320x240)
            elif file_size == 76800:
                print(f"No header found. Interpreting as Raw 320x240 grayscale...")
                raw_data = f.read()
                pixels = np.frombuffer(raw_data, dtype=np.uint8).reshape((240, 320))
            
            else:
                print(f"Unknown format. Size is {file_size} bytes (not 76800).")
                return

        # Visualization
        plt.figure(figsize=(10, 6))
        # Use 'gray' colormap and force the scale 0-255 for Bayesian Opt exposure analysis
        plt.imshow(pixels, cmap='gray', vmin=0, vmax=255)
        plt.title(f"Viewing: {selected_file}")
        plt.colorbar(label='Pixel Intensity (0-255)')
        plt.axis('off')
        plt.show()

    except Exception as e:
        print(f"Script Error: {e}")

if __name__ == "__main__":
    display_pgm()
