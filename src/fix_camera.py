import os
import re

# Check both potential workspace paths
paths_to_try = [
    os.path.expanduser("~/barn_ws/src/the-barn-challenge/jackal_helper/worlds/BARN"),
    os.path.expanduser("~/barn_ws_rl/src/the-barn-challenge/jackal_helper/worlds/BARN")
]

worlds_dir = None
for p in paths_to_try:
    if os.path.exists(p):
        worlds_dir = p
        break

if not worlds_dir:
    print("ERROR: Could not find the BARN worlds folder.")
    exit()

# The exact coordinates you want to swap in
new_pose = "2 1.5 3 0 0.4 2.35619"

# This regex finds the <camera name='user_camera'> block, grabs the <pose> tag, 
# and prepares to replace whatever numbers are inside it.
pattern = re.compile(r"(<camera name=['\"]user_camera['\"]>\s*<pose[^>]*>)[^<]+(</pose>)")

modified_count = 0

# Loop through all 300 worlds
for i in range(300):
    filepath = os.path.join(worlds_dir, f"world_{i}.world")
    
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            content = f.read()
        
        # Swap the old numbers with the new pose
        new_content, count = pattern.subn(rf"\g<1>{new_pose}\g<2>", content)
        
        # If a change was actually made, save the file
        if count > 0 and new_content != content:
            with open(filepath, 'w') as f:
                f.write(new_content)
            modified_count += 1

print(f"Found folder at: {worlds_dir}")
print(f"Successfully updated the camera pose in {modified_count} world files!")