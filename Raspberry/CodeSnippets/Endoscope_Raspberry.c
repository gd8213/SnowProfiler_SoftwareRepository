// https://raspberrypi-guide.github.io/electronics/using-usb-webcams

// Installation
// sudo apt install fswebcam
// sudo apt install ffmpeg


#include <stdio.h>
#include <string.h>
#include <stdlib.h>     // System call

int main() { 
	// Take Picture
    int status = system("fswebcam -r 1600x1200 --no-banner image1.jpg");        
	
	// Take Video
	// & ... run in background
	// -y ... overwrite existing file
	// -t ... duration of video
    status = system("ffmpeg -t 5 -f v4l2 -framerate 50 -video_size 1600x1200 -y -i /dev/video0 output.mjpg &"); 

// 					 ffmpeg -f v4l2 -s 1600x1200 -y -i /dev/video0 -t 5 output.avi
//					 ffmpeg -f v4l2 -r 50 -s 1600x1200 -y -i /dev/video0 -t 5 output.avi
//					 ffmpeg -i /dev/video0 -c:v copy -c:a copy -y -t 5 output.avi				// Better FPS


	
    return 0;
} 


