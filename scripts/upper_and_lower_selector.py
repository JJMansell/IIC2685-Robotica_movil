import cv2
import numpy as np
import os

def main():
    # 1. Load your image (replace with your actual image path)
    image_path = os.path.join("image.png")
    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Could not load image from '{image_path}'. Please check the path.")
        return

    # 2. Convert the image from BGR to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 3. Open an interactive window to select the region
    print("Instructions:")
    print(" - Click and drag your mouse over the target area to select it.")
    print(" - Press ENTER or SPACE to confirm the selection.")
    print(" - Press 'c' to cancel.")
    
    roi = cv2.selectROI("Select HSV Zone", image, fromCenter=False, showCrosshair=True)
    cv2.destroyAllWindows()

    # Unpack the bounding box coordinates (x, y, width, height)
    x, y, w, h = roi

    # Check if a valid selection was made
    if w > 0 and h > 0:
        # Crop the HSV image to the selected zone
        hsv_roi = hsv_image[y:y+h, x:x+w]

        # Find minimum and maximum HSV values across the region
        hsv_min = np.min(hsv_roi, axis=(0, 1))
        hsv_max = np.max(hsv_roi, axis=(0, 1))

        # Output the results ready to use in your masking code
        print("\n" + "="*30)
        print(" SUCCESS! FOUND HSV RANGE:")
        print("="*30)
        print(f"Lower Bound: [{hsv_min[0]}, {hsv_min[1]}, {hsv_min[2]}]")
        print(f"Upper Bound: [{hsv_max[0]}, {hsv_max[1]}, {hsv_max[2]}]")
        print("\nPaste this directly into your mask code:")
        print(f"lower_range = np.array([{hsv_min[0]}, {hsv_min[1]}, {hsv_min[2]}])")
        print(f"upper_range = np.array([{hsv_max[0]}, {hsv_max[1]}, {hsv_max[2]}])")
        print("="*30)
    else:
        print("Selection was cancelled or no area was selected.")

if __name__ == "__main__":
    main()
