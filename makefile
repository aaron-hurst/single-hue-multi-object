helpmake: shmo.cpp
	g++ -o shmo shmo.cpp -L/opt/vc/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util
