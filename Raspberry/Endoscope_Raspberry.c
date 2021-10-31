// https://raspberrypi-guide.github.io/electronics/using-usb-webcams

// Ins
// sudo apt-get install libopencv-dev


#include <stdio.h>
#include <string.h>
#include <stdlib.h>     // System call

int main() { 
    int status = system("fswebcam -r 1600x1200 --no-banner image1.jpg");        // Take Picture
    int status = system("ffmpeg -t 5 -f v4l2 -framerate 50 -video_size 1600x1200 -i /dev/video0 output.mjpg");        // Take Video
    return 0;
} 


