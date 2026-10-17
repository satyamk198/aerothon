# Result of running the QR detect-and-match pipeline on one image.
# Fields ported from qr_match_interfaces/QRResult (Shiva007-RAndom/QR_Node).
# The header is copied from the incoming image so results stay
# time-synchronised with the source frame.
std_msgs/Header header

# Size of the processed image, in pixels
uint32 image_width
uint32 image_height

# True if at least one QR code was found in the image
bool qr_detected

# Number of QR codes found
uint16 num_detections

# True if at least one detected QR matched the reference
# (i.e. best_score >= match_threshold)
bool match_found

# Highest score across all detections in this frame, 0.0 if none
float32 best_score

# Threshold that was applied to produce match_found
float32 match_threshold

# Per-QR details, ordered by descending score
QrCode[] detections

# Wall-clock processing time for this frame, in milliseconds
float32 processing_time_ms
