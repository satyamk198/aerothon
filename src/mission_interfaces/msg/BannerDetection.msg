std_msgs/Header header
bool detected
float32[4] bbox_xyxy        # [x1, y1, x2, y2] pixel coordinates
float32 center_x            # pixel x of bbox center
float32 center_y            # pixel y of bbox center
float32 confidence           # detection confidence / green ratio
uint32 image_width
uint32 image_height
string text                  # recognized text (e.g. "AEROTHON 2026")
