"""Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable"""

__version__ = "2023.09.26"


#Number of bricks in the jig
jig_num_bricks = len(jig_bricks_brep)

# Number of bricks in the wall
wall_num_bricks = len(wall_bricks)
#print wall_num_bricks

# Multiplication (lower_bound index of the chunk list
wall_lower_bound = mult_factor * jig_num_bricks

# Remaining bricks
remaining_bricks = wall_num_bricks - wall_lower_bound

# Minimum of remaining_bricks and jig_num_bricks
min_val = min(remaining_bricks, jig_num_bricks)
print(wall_lower_bound)
print(remaining_bricks)
print(min_val)
# Finding the upper bound index of the chunk list
wall_upper_bound = wall_lower_bound + min_val
# Wall Bricks Chunk list (chunk of bricks)
wall_bricks_chunk = wall_bricks[wall_lower_bound: wall_upper_bound]
bricks_chunk = wall_bricks_chunk

# Jig Bricks Chunk list
jig_lower_bound = 0
jig_upper_bound = min_val
jig_bricks_chunk = jig_bricks_brep[jig_lower_bound: jig_upper_bound]
print(jig_bricks_chunk)