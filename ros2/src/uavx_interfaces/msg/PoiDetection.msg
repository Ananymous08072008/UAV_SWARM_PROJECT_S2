# A PoI target found in the nadir camera and geolocated on the ground.
std_msgs/Header header        # sim time of the image
string uav_id
float64 east_m                # estimated target position, world ENU (origin = GCS)
float64 north_m
float32 pixel_area            # target size in the image (pixels)
float32 altitude_m            # camera height used for the projection
string image_path             # snapshot saved for the GCS (imagery deliverable)
