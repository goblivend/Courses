#include <iostream>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

pid_t libcamera_pid = -1;

// Signal handler to kill libcamera before exiting
void sighandler(int signum)
{
    std::cout << "Received SIGINT. Exiting gracefully..." << std::endl;
    kill(libcamera_pid, SIGINT);
    exit(0);
}

int main(int argc, char *argv[]) {
  pid_t pid = fork();

  if (pid < 0) {
    std::cerr << "Failed to fork process" << std::endl;
    return -1;
  }

  const char *frame_name = "frame.jpeg";
  const char *tmp_frame_name = "tmp_frame.jpeg";

  // Child process => use to run libcamera in background
  if (pid == 0) {
    // Close stdout and stderr to keep it for the parent process
    // close(STDOUT_FILENO);
    // close(STDERR_FILENO);

    // Exec libcamera
    execlp("libcamera-still", "libcamera-still", "-n", "--verbose", "0",
          "--timeout", "0", "--width", "640", "--height", "480", "--mode",
          "3280:2464", "-o", tmp_frame_name, "--signal", NULL);
    return 1;
  }

  // Parent process => use to process frame

  // Signal handler => kill libcamera if receive SIGINT (ctrl + C) signal
  libcamera_pid = pid;
  signal(SIGINT, sighandler);

  // Send SIGUSR1 for the first frame.
  usleep(3000000); // 500 ms (waiting for execvp ready)
  kill(libcamera_pid, SIGUSR1);
  usleep(3000000); // 1 s

  cv::VideoWriter out;
  cv::Size capSize(640, 480);
  out.open("videoFlip.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, capSize, true);
  if (!out.isOpened()) {
      std::cerr << "Could not open the output video file for write\n";
      return -1;
  }

  // usleep(1000000); // 100 ms
  // kill(libcamera_pid, SIGUSR1);
  // usleep(1000000); // 100 ms

  cv::Mat frame;
  cv::Mat frameFlip;
  int nbFrames = 200;
  while (nbFrames--) {
    std::cout << "Processing frame " << nbFrames << std::endl;
    int mvPid = fork();
    if (mvPid == 0) {
      execlp("mv", "mv", tmp_frame_name, frame_name, NULL);
      return 1;
    }
    kill(libcamera_pid, SIGUSR1);
    usleep(400000); // 100 ms

    frame = cv::imread(frame_name);
    if (frame.empty())
        break;
    cv::flip(frame, frameFlip, 0);
    out.write(frameFlip);
  }

  kill(libcamera_pid, SIGKILL);
}
