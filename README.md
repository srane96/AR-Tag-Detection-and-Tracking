# AR-Tag-Detection-and-Tracking
Detect and read the AR tags in the given video. Calculate it's ID and orientation and superimpose template image on it.

## Project execution flowchart:
<p>
  <img src="images/algo.PNG">
</p>

## Tag Detection
<p>Since AR tag has only black and white pixels, it is easier to detect AR tag in a given frame using general thresholding process. Hence in this code, after getting the current frame from the video, it is first converted into a grayscale image to make it a single channel image. Then binary thresholding is applied to this grayscale image with a threshold value equal to 200.
As a result, white paper containing the AR tag is clearly distinguished from the background making it easier to detect the corners.
</p>
<p>
  <img src="images/tag1.PNG">
</p>

## Reading Tag Orientation
<p>In order to detect the orientation of the tag, it is necessary to detect its corners first. For this purpose, the contour detection algorithm in OpenCV is used. Initially, after running the contour detection, contours were thresholded according to their areas. Also, another condition is added which filters out all the contours that do not have the 4 corners. Once a contour is detected, coordinates of its corners are found out in order top_left, top_right, bottom_right, and bottom_left. Using these coordinates, a homography matrix is calculated that maps the tag image from world frame to the camera frame as shown below. </p>
<p>
  <img src="images/tag2.PNG">
</p>
<p>Once the tag image is mapped into the camera frame, it is divided into a 8X8 grid of squares. As shown in the figure below, the outer squares that hold the information about the orientation are located at (3,3), (6,6), (6,6), (3,6) with respect to the grid and the orientation of the tag is determined based on which outer square is white. To figure out which one of these squares is white; the average of all the pixel values is calculated for each square. Since it is known that only one of the squares is white and the other three are black, the orientation of the tag image is represented by the location of the square with the highest average pixel value (as the square with the white background will have the highest pixel value average). </p>
<p>
  <img src="images/tag_reading.PNG">
</p>

## Reading the tag ID
<p>To read the tag ID, the same logic is used which was used for determining the orientation of the tag. Tag ID is determined by the value of the squares at the location (4,5), (5,5), (5,4), (4,4). The location of the MSB and LSB squares is determined based on the orientation of the tag. To determine the ID, the average value of all the pixels is calculated for each square. If the average is greater than 180, then it is considered as a white square or else it is black square. Then finally ‘1’ is stored for white square and ‘0’ is stored for the black pixel.</p>

## Superimposing an image onto the tag
<p>In this part of the section, the Lena.png image is used as a template. Using the orientation of the tag, a new homography matrix H is calculated between the rotated coordinates of the tag in the world frame and the coordinates of the template in the camera frame. Then to map the template image onto the tag image in the world frame, inverse of this newly calculated H matrix is used. This inverse of H is utilized in warp function with frame size as input, the template image is given is warped into its projection on world frame. Now utilizing this new frame to merge the existing camera frame, an appropriate mask is created to fit the template into the camera frame. Then the merging is done using bitwise_and() and add() function. Then it is shown as output.</p>
<p>
  <img src="images/tag3.PNG">
</p>

## Placing a virtual cube on the tag.
<p>After finding the Homography matrix and placing Lena on the tag, the next task was to place a 3D cube on the tag. We compute the Projection Matrix, from the Homography matrix with the help of a calibration matrix and determine the rotational and the translation components needed to determine the camera pose. We then pass these values along with a 3D axis to an OpenCV function cv2.projectPoints() to find the points of the projection and draw lines along those points as shown in the figure below. </p>
<p>
  <img src="images/tag4.PNG">
</p>

##  Multiple Tags
<p>When it comes to multiple tags, Identification of the Tag ID was done successfully, due to blurry image initially there was miscalculation yet as soon as the tag comes in better focus the tag IDs get updated to their actual values. But Image template faced some issues with simultaneous superimposition on each tag, rather it keeps jumping from tag to tag, yet the orientation is captured successfully. 3D Cubes also faced some initial problems as the camera moves closer to the tags, the algorithm works effectively.</p>
<p>
  <img src="images/tag5.PNG">
</p>
