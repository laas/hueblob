# A box defines a zone in a 2d image.
#
#  (0, 0) --------------------->
#   |
#   |   (x, y)----------(x + w, y)
#   |     |                   |
#   |     |     B O X         |
#   |     |                   |
#   |     |                   |
#   |   (x, y + h)------(x + w, y + h)
#  \/
Header header
sensor_msgs/RegionOfInterest roi