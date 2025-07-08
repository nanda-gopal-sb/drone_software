### This is the software code for Team SINGULARIT's SUAS 2025 mission. 

## Mission
* Cover 12 points at a specified altitude
* Make a map of the area
* Drop 4 payloads, each payload in every lap. 
* Laps once started cannot be broken in between. 
* Detect and find the location of 4 objects in the airdrop boundary. 
* Only one payload can be deployed in each lap.

## Software Systems
* Get video stream from telemetry port. (RTSP Stream)
    * Fix latency issues (From 750ms to 560ms)
    * Split frames, on every second
    * Run a cropping algorithm to remove the unwanted parts of the frame(Camera details such as flash indicators)
* Open Drone Map
    * Feed the split frames into Open Drone Mapping algorithm and pass in the parameters to get fast and efficient map.
* YOLO
    * Run detection model on either video stream or video frames. Went with frames as inference rates are faster (1-2ms)
    * If any object (Just object detection is needed) is within the frame, note the geolocation of that frame.
    * Find the closest object to the center of frame and make sure that it is writeen into a seperate file.
* Payload Deployment
    * Trigger the servo pins (AUX pins 9 - 12) to drop each paylod
* Surveying Motion
    * Make the drone move in a coordinated lawnmover action with the speed of the drone set at 3m/s (Later realized it would have worked with 7m/s as well) Judging by the height and width of each recived frame.
