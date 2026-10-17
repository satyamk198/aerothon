# Red patches (restricted zones) found in one image.
# The header is copied from the incoming image.
std_msgs/Header header

# Patches with area >= min_area, in contour order
RedPatch[] patches

# Wall-clock processing time for this frame, in milliseconds
float32 processing_time_ms
