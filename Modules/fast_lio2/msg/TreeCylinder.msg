# One tracked trunk, expressed in the LIO world frame.
uint32 id
float32 x               # trunk axis at the track base, X
float32 y               # trunk axis at the track base, Y
float32 z_base          # bottom of the observed trunk
float32 height          # observed height
float32 radius          # fitted cylinder radius
geometry_msgs/Vector3 axis
float32 coverage        # observed circumference (rad)
uint16 observations     # number of batches that supported this track
