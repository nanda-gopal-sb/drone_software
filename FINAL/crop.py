import os
from PIL import Image

def crop_images_from_top_bottom(input_dir, output_dir_name="cropped", pixels_to_crop=100):
    output_dir = os.path.join(input_dir, output_dir_name)
    os.makedirs(output_dir, exist_ok=True)

    for filename in os.listdir(input_dir):
        file_path = os.path.join(input_dir, filename)

        if os.path.isfile(file_path):
            try:
                with Image.open(file_path) as img:
                    width, height = img.size

                    if height <= 2 * pixels_to_crop:
                        print(f"Warning: Image '{filename}' is too small ({height}px) to crop {pixels_to_crop*2}px. Skipping.")
                        continue

                    crop_box = (0, pixels_to_crop, width, height - pixels_to_crop)

                    cropped_img = img.crop(crop_box)

                    save_path = os.path.join(output_dir, filename)
                    cropped_img.save(save_path)
                    print(f"Successfully cropped and saved: {filename}")

            except Exception as e:
                print(f"Could not process '{filename}'. Error: {e}")
                print(f"Skipping '{filename}'. It might not be a valid image or is corrupted.")

if __name__ == "__main__":
    input_directory = "MAPPING" 

    output_folder_name = "CROPPED"
    pixels_to_remove = 100
    
    if not os.path.isdir(input_directory):
        print(f"Error: The specified input directory does not exist: {input_directory}")
        print("Please update the 'input_directory' variable in the script to your actual image folder path.")
    else:
        print(f"Starting image cropping from: {input_directory}")
        crop_images_from_top_bottom(input_directory, output_folder_name, pixels_to_remove)
        print("Image cropping process complete.")