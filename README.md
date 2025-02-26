# Lane Centering Accuracy Metric using ArUco Marker based Local Positioning System (LPS) in small rover environment 

See the documentation for this project on my LinkedIn Page www.linkedin.com/in/salvatore-pernice - Autonomous System Engineering (ASE) Bachelor Project on my projects section. You will also be able to see the live DEMO for this on my LinkedIn Page as github does not support large files.

## Files and Descriptions

1. **ProcessedTrackImage.py**
   - **Description**: This script processes an image of the B-Circuit. The processed image is essential for the `LivePositioningErrorDetector.py` script as it uses it as input to calculate the distance to the lanes
   - **Usage**: Run this script to generate the processed track image before running the positioning error detector.

2. **processed_track.jpg**
   - **Description**: This image is an example of the processed B-Circuit that will be used by the positioning error dtector script.
   - **Usage**: Use this as a reference to understand the type of processed image required for accurate error detection. You can see how no black noise pixels should be present inside the lane boundaries.

3. **LivePositioningErrorDetector.py**
   - **Description**: This script should be run while the rover is operational. Connects to the ceiling mounted camera and it uses the processed track image to detect and measure deviation errors from the center of the lane.
   - **Usage**: make sure you have obtained the processed track image before running this script to monitor the rover’s centering positioning errors in real-time. When ending the script by pressing 'q' the output will give you the avarage positioning error and standard deviation, as well as provide you with a visualization of all recorded track positions in `Trajectory Analysis.jpg`.

4. **DEMO - PositioningErrorDetector.avi**
   - **Description**: A demonstration video showcasing the positioning error detector in action. The video is recorded using the ceiling-mounted camera while the rover navigates using the dynamic lookahead method at 25% speed.
   - **Usage**: you can watch this demo to see how the positioning error detector functions
 
