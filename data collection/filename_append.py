import os

# Set the directory containing the files
directory = "D:\\camera0_pcb2_CFmicrowire_2025-03-12"

# Set the string to append
append_str = "_camera0_pcb2_CFmicrowire_2025-03-13"

# Iterate over all files in the directory
for filename in os.listdir(directory):
    old_path = os.path.join(directory, filename)

    # Ensure it's a file and not a directory
    if os.path.isfile(old_path):
        # Split filename and extension
        name, ext = os.path.splitext(filename)
        
        # Create the new filename
        new_filename = f"{name}{append_str}{ext}"
        new_path = os.path.join(directory, new_filename)

        # Rename the file
        os.rename(old_path, new_path)
        print(f"Renamed: {filename} -> {new_filename}")

print("Done renaming files.")
