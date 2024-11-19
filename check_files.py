import os

# Set the paths to your folders
xml_folder = r"C:\Users\user\Desktop\dataset"
jpg_folder = r"C:\Users\user\Desktop\jpg"

# Get a list of all XML and JPG filenames without extensions
xml_files = {os.path.splitext(file)[0] for file in os.listdir(xml_folder) if file.endswith('.xml')}
jpg_files = {os.path.splitext(file)[0] for file in os.listdir(jpg_folder) if file.endswith('.jpg')}

# Find XML files without corresponding JPG files
missing_jpg_files = xml_files - jpg_files

# Output the result
if missing_jpg_files:
    print(f"Missing JPG files for the following XML files: {missing_jpg_files}")
else:
    print("All XML files have corresponding JPG files.")
