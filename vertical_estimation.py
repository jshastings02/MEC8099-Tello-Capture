import cv2
import os
import pandas as pd

# === CONFIGURATION ===
FRAME_FOLDER = 'frames'  # Folder containing extracted frames
OUTPUT_EXCEL = 'net_height_data.xlsx'
KNOWN_WIDTH_METERS = 0.17  # Real width of drone (in meters)
FOCAL_LENGTH_PIXELS = 640  # Focal length (adjust if needed)

# Display configuration
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720

# === GLOBAL VARIABLES ===
clicks = []
scale = 1.0

# === MOUSE CALLBACK ===
def click_event(event, x, y, flags, param):
    global clicks, scale
    if event == cv2.EVENT_LBUTTONDOWN:
        true_x = int(x / scale)
        true_y = int(y / scale)
        clicks.append((true_x, true_y))
        print(f"Click {len(clicks)} at ({true_x}, {true_y})")

# === MAIN PROGRAM ===
if __name__ == '__main__':
    frames = sorted([f for f in os.listdir(FRAME_FOLDER) if f.endswith(('.png', '.jpg', '.jpeg'))])

    if not frames:
        print("No frames found!")
        exit(1)

    if os.path.exists(OUTPUT_EXCEL):
        df = pd.read_excel(OUTPUT_EXCEL)
        labeled_frames = set(df['Frame'])
        print(f"Resuming labeling. {len(labeled_frames)} frames already labeled.")
    else:
        df = pd.DataFrame(columns=[
            'Frame', 'Net_Center_X', 'Net_Center_Y', 'Net_Lowest_X', 'Net_Lowest_Y',
            'Drone_Center_X', 'Drone_Center_Y', 'Pixel_Width', 'Meters_per_Pixel',
            'Vertical_Pixel_Diff_Center', 'Height_Diff_Center_Meters',
            'Vertical_Pixel_Diff_Lowest', 'Height_Diff_Lowest_Meters'
        ])
        labeled_frames = set()

    for frame_name in frames:
        if frame_name in labeled_frames:
            continue

        frame_path = os.path.join(FRAME_FOLDER, frame_name)
        frame = cv2.imread(frame_path)

        if frame is None:
            print(f"Failed to load frame {frame_name}")
            continue

        clicks = []
        skip_frame = False

        # Resize for display
        h, w = frame.shape[:2]
        scale_w = SCREEN_WIDTH / w
        scale_h = SCREEN_HEIGHT / h
        scale = min(scale_w, scale_h, 1.0)
        if scale < 1.0:
            frame_display = cv2.resize(frame, (int(w * scale), int(h * scale)))
        else:
            frame_display = frame.copy()

        cv2.imshow("Frame", frame_display)
        cv2.setMouseCallback("Frame", click_event)

        print(f"\nFrame: {frame_name}")
        print("Click 1: Net center")
        print("Click 2: Lowest point of net")
        print("Click 3: Drone left edge")
        print("Click 4: Drone right edge")
        print("Press 'u' to undo last click, 'Esc' to skip this frame.")

        while len(clicks) < 4:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('u') and clicks:
                print("Undo last click!")
                clicks.pop()
            if key == 27:  # ESC key
                print("Skipping frame!")
                skip_frame = True
                break

        if skip_frame:
            cv2.destroyAllWindows()
            continue

        net_x, net_y = clicks[0]
        lowest_net_x, lowest_net_y = clicks[1]
        drone_left_x, drone_left_y = clicks[2]
        drone_right_x, drone_right_y = clicks[3]

        drone_center_x = (drone_left_x + drone_right_x) / 2
        drone_center_y = (drone_left_y + drone_right_y) / 2
        pixel_width = abs(drone_right_x - drone_left_x)

        if pixel_width == 0:
            print("Error: Drone pixel width is 0!")
            cv2.destroyAllWindows()
            continue

        meters_per_pixel = KNOWN_WIDTH_METERS / pixel_width
        vertical_pixel_diff_center = net_y - drone_center_y
        vertical_pixel_diff_lowest = lowest_net_y - drone_center_y
        height_diff_center_meters = vertical_pixel_diff_center * meters_per_pixel
        height_diff_lowest_meters = vertical_pixel_diff_lowest * meters_per_pixel

        df = pd.concat([df, pd.DataFrame([{
            'Frame': frame_name,
            'Net_Center_X': net_x,
            'Net_Center_Y': net_y,
            'Net_Lowest_X': lowest_net_x,
            'Net_Lowest_Y': lowest_net_y,
            'Drone_Center_X': drone_center_x,
            'Drone_Center_Y': drone_center_y,
            'Pixel_Width': pixel_width,
            'Meters_per_Pixel': meters_per_pixel,
            'Vertical_Pixel_Diff_Center': vertical_pixel_diff_center,
            'Height_Diff_Center_Meters': height_diff_center_meters,
            'Vertical_Pixel_Diff_Lowest': vertical_pixel_diff_lowest,
            'Height_Diff_Lowest_Meters': height_diff_lowest_meters
        }])], ignore_index=True)

        df.to_excel(OUTPUT_EXCEL, index=False)
        print(f"Saved Frame {frame_name}")
        cv2.destroyAllWindows()

    print(f"\n✅ Done! All data saved to {OUTPUT_EXCEL}")
