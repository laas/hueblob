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
int16 x
int16 y
int16 width
int16 height
