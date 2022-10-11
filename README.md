# Yildiz-Rocket-Ground-Station-GUI

A Python Tkitnter-based GUI App prepared for rocket and satellite flight missions of the Yıldız Rocket Team. Here are the some features included in the project.

- The application is designed with Figma, a professional design tool. The design was transferred to the application with the help of an open-source [tool](https://github.com/ParthJadhav/Tkinter-Designer) . The output created by the tool is completely coordinate-based and non-scalable. The design was made with representative pictures (including the background and container design) and the coordinates were obtained in the output in which our models were positioned. For easier use, a class has been created with the inheritance method and the output objects have been placed here.

- Receive and save telemetry data, command sending, and [file transfer](https://github.com/doguknY/UART-FILE-TRANSFER) to STM32-based computer (tested with 921600 bauds) via serial port with pySerial library. 

- Capturing live video with OpenCV-python library and placing the video to Tkinter interface and saving it to disk at the same time. Webcam sources (including HDMI sources transferred using a converter) or a video source on the local network can be used for the live video source.

- Opening the .obj file and integrating it into the Tkinter interface as a 3D model with PyOpenGL, pyopengltk, and pywavefront library. The model is oriented according to live-angle data with OpenGL functions. This feature does not work on macOS due to the lack of OpenGL support.

- Matplotlib library is used for live plotting of data. The embedding of Matplotlib plots in the Tkinter interface has caused serious performance problems in the entire application, even with a single graph. The problem has been solved with blitting and thus 4 graphs can be used at the same time with a high refresh rate.

- Tracking with coordinates on the map is done with Pillow, which is the imaging library, instead of a whole 3D map. Since it is done by placing the coordinate values on the picture, the bird's eye view of the target area should be taken and the 2 edges coordinates of the image should be known. According to these new corner values, the incoming latitude and longitude data are added to a separate .csv file and these data are scaled to the picture continuously. With this method, more than one object can be followed by drawing a line with the data.