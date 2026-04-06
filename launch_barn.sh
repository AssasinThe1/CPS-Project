#!/bin/bash

# If no argument is passed, default to 0.
WORLD_IDX=${1:-0}
HEURISTIC_VAL=$2
RUN=${3:-0}

# Determine the directory and argument based on the heuristic value
if [ ! -z "$HEURISTIC_VAL" ]; then
    VIDEO_DIR="videos/heuristic_${HEURISTIC_VAL}"
    HEURISTIC_ARG="--heuristic $HEURISTIC_VAL"
    RUN_ARG="--run $RUN"
else
    VIDEO_DIR="videos/no_heuristic"
    HEURISTIC_ARG=""
    RUN_ARG=""
fi

# Create the folder if it doesn't exist yet (-p prevents errors if it already exists)
mkdir -p "$VIDEO_DIR"

# Update the video file path to be inside the new directory
VIDEO_FILE="${VIDEO_DIR}/barn_run_${WORLD_IDX}_${RUN}.mp4"

echo "Starting BARN Challenge nodes for World Index: $WORLD_IDX"
if [ ! -z "$HEURISTIC_VAL" ]; then
    echo "Using Heuristic: $HEURISTIC_VAL"
fi
echo "Video will be saved as: ${VIDEO_FILE}"

if [ -f "$VIDEO_FILE" ]; then
    echo "Overwriting existing video file: $VIDEO_FILE"
    rm -f "$VIDEO_FILE"
fi

# Launch 1: run.py (Removed '; exec bash' so the window closes when killed)
gnome-terminal --title="Run Script" -- bash -ic "source devel/setup.bash; python3 src/the-barn-challenge/run.py --world_idx ${WORLD_IDX} ${HEURISTIC_ARG} ${RUN_ARG}"
sleep 1

# Launch 2: Nav-VFH.py
gnome-terminal --title="Navigation Node" -- bash -ic "python3 ~/barn_ws/src/the-barn-challenge/navigation_pkg/scripts/Nav-VFH.py"
sleep 1

# Launch 3: Gazebo
gnome-terminal --title="Gazebo" -- bash -ic "rosrun gazebo_ros gazebo"

echo "Waiting for Gazebo window to appear..."
for i in $(seq 1 30); do
    GAZEBO_WID=$(xdotool search --onlyvisible --name "Gazebo" 2>/dev/null | tail -1)
    if [ ! -z "$GAZEBO_WID" ]; then
        echo "Gazebo GUI found. Waiting for it to fully render..."
        sleep 3  # Give the 3D environment time to draw on screen
        break
    fi
    sleep 1
done

# Minimize all Terminal windows (suppressing errors if it fails)
xdotool search --class "gnome-terminal" 2>/dev/null | xargs -I{} xdotool windowminimize {} 2>/dev/null

# Bring Gazebo to front safely
# Bring Gazebo to front safely
if [ ! -z "$GAZEBO_WID" ]; then
    xdotool windowraise $GAZEBO_WID 2>/dev/null
    xdotool windowfocus $GAZEBO_WID 2>/dev/null
    
    echo "Adjusting Gazebo camera view..."
    # Move the default GUI camera to look straight down at the center of the map
    # -x, -y: Center of the map (adjust these if your map is larger than 10x10)
    # -z: Height/Zoom (increase to zoom out more)
    # -P 1.5708: Pitch 90 degrees straight down
    gz camera -c user_camera -x 5.0 -y 5.0 -z 15.0 -R 0.0 -P 1.5708 -Y 0.0
fi

sleep 2

# Launch 4: Video Recorder
# Make sure to put $VIDEO_FILE in quotes in case the path ever has spaces
gnome-terminal --title="Screen Recorder" -- bash -ic "xhost +local: && ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i \$DISPLAY -preset ultrafast \"${VIDEO_FILE}\""

# Minimize the recorder terminal too
sleep 1
xdotool search --name "Screen Recorder" | xargs -I{} xdotool windowminimize {}
echo "All nodes launched. Monitoring Navigation Node..."

sleep 3

# Wait until Nav-VFH.py is no longer running
while pgrep -f "run.py --world_idx" > /dev/null; do
    sleep 1
done

echo "Navigation finished or stopped! Saving video..."

# Send SIGINT (Ctrl+C) specifically to ffmpeg so it compiles the mp4 safely
pkill -SIGINT -f "ffmpeg"

echo "Video saved. Waiting 3 seconds before shutting down the rest of the environment..."
sleep 3

echo "Stopping remaining nodes and closing terminals..."

# Forcefully kill the navigation node
pkill -f "Nav-VFH.py"

# Forcefully kill the run script
pkill -f "run.py --world_idx"

# Kill Gazebo backend and frontend
killall -9 gzserver gzclient roscore rosmaster 2>/dev/null

echo "Run complete and environment cleaned up successfully!"