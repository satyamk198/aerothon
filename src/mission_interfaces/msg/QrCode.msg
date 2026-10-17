# One QR code detected in an image.
# Fields ported from qr_match_interfaces/QRDetection (Shiva007-RAndom/QR_Node).

# Axis-aligned bounding box in pixels: [x1, y1, x2, y2]
int32[4] bbox_xyxy

# Detector (YOLO) confidence for this box, in [0, 1]
float32 detector_confidence

# Similarity against the reference QR, in [0, 1] (1.0 == identical module grid)
float32 score

# True if score >= match_threshold
bool matched

# Decoded payload, or "" if this QR could not be decoded
string decoded_text

# True if the score came from an exact decoded-text comparison
# rather than from module-grid similarity
bool score_from_decoded_text
