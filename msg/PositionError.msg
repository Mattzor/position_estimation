# simple x and y coordinates. These might not always be set, depending on the situation. Always check for a null-value. Will most likely be set to 0 if null,
# so take this into consideration.
int16 x
int16 y


# lane_error is a float holding the error from the centerline.
# the error is simply a pixel-count, and a negative value indicates the truck is too far to the left, and a positive value indicate the truck is too far to the right.
# One should aspire to keep this value as close to 0 as possible.
float32 lane_error
