import pydicom
import os

# Paths to input compressed file and output decompressed file
input_dir = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/data/CQ500/decomp"
output_dir = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/data/CQ500/decomp"

for subfol in os.listdir(input_dir):
    subfolder = os.path.join(input_dir,subfol)
    print(subfolder)
    
    for file in os.listdir(subfolder):
        filepath = os.path.join(subfolder, file)
        
        if os.path.getsize(filepath) < 350000:
            if file.endswith(".dcm"):  # Process only DICOM files
                input_path = os.path.join(subfolder, file)
                
                # Read the DICOM file
                ds = pydicom.dcmread(input_path)

                # Check if the file is already decompressed, if not, decompress it
                try:
                    ds.decompress()  # Uses gdcm or Pillow if available for specific compression formats
                    ds.save_as(input_path)
                    print(f"Decompressed and saved: {input_path}")
                except Exception as e:
                    print(f"Failed to decompress {input_path}: {e}")
        else:
            break