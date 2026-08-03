The visiongps python files send the car id, monotonic time[time counting starts from the time laptop turned ON], x,y and yaw serially to the bridge ESP32.
The bridge ESP32 gets this data and broadcasts it to car ESP32's via ESPNOW.

The car esp recieves the broadcast, checks the recieved message ID and its own car ID, if it matches then it uses the coordinates, if ID doesnt match it just throws it out.

Way to run the codes:
1. Adjust the COM Ports in the Vision_GPS3.py and run the code.
2. Flash the Bridge.ino code to esp32 connected to laptop[BRIDGE].
3. Connect the webcam and the BRIDGE to laptop and run the code.
4. Adjust the CarID [make it unique for each car] and then flash ACTUAL_COMM.ino code to esp32 in the cars.
