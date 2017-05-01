#include <ctime>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <raspicam/raspicam_cv.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <unistd.h>
#include <time.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "opencv2/bgsegm.hpp"
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
using namespace std;
using namespace cv;
//////////////////////////////////CLASSES///////////////////////////////
 class Object
 {
 public:
 	Object()
 	{
 		setType("Object");
 	}
 	~Object(void)
 	{
 	}

 	int getXPos()
 	{
 		return xPos;
 	}
 	void setXPos(int x)
 	{
 		xPos = x;
 	}

 	int getYPos()
 	{
 		return yPos;
 	}
 	void setYPos(int y)
 	{
 		yPos = y;
 	}

 	string getType()
 	{
 		return type;
 	}
 	void setType(string t)
 	{
 		type = t;
 	}

 private:
 	int xPos, yPos;
 	string type;
 };

//////////////////////////////////////////////////////////////////////VARIABLES//////////////////////////////////////////////////////////////////////
//Mats:
cv::Mat src;
cv::Mat cameraFeed;
cv::Mat theBackground;
cv::Mat clean;
cv::Mat fgMaskMOG2; //background subtraction
cv::Ptr<BackgroundSubtractor> pMOG2;
bool backgroundSet = false;
//Resolution:
const int FRAME_WIDTH = 2560;//640;
const int FRAME_HEIGHT = 1920;//480;
//Points of the adjusted ROI
int P1_X = 0;
int P1_Y = 0;
int P2_X = 0;
int P2_Y = FRAME_HEIGHT;
int P3_X = FRAME_WIDTH;
int P3_Y = 0;
int P4_X = FRAME_WIDTH;
int P4_Y = FRAME_HEIGHT;
//NORTH-WEST point
Point P1(P1_X, P1_Y);
//SOUTH-WEST point
Point P2(P2_X, P2_Y);
//NORTH-EAST point
Point P3(P3_X, P3_Y);
//SOUTH-EAST point
Point P4(P4_X, P4_Y);

//Point for grid layout:
Point M1(0, 0);
Point M2(0, 0);
Point M3(0, 0);
Point M4(0, 0);
Point C1(FRAME_WIDTH/2,FRAME_HEIGHT/2);

//timing our api calls, weather and detection
bool checkWeather = false;
bool checkDetection = false;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA =  2 * 2;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
const int myThreshold = 80;
Mat cropped;

//Commands to send the drone to different quadrants:
//location of geese, 0000=no geese, 1000=geese on q1, 0100=geese on q1 etc
char cmdStay[5] = "0000";
char cmdQ1[5] = "1000";
char cmdQ2[5] = "0100";
char cmdQ3[5] = "0010";
char cmdQ4[5] = "0001";
char theCMD[5] = "0000";

//other global variables for the server:
unsigned int sockfd, newsockfd, portno, clilen;
char buffer[5];
struct sockaddr_in serv_addr, cli_addr;
int n;


raspicam::RaspiCam_Cv capture;
//////////////////////////////////////////////////////////////////////FUNCTIONS//////////////////////////////////////////////////////////////////////
//function to initialize the server:
void serverInit()
{
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) cout << ("ERROR opening socket") << endl;
	bzero((char *) &serv_addr, sizeof(serv_addr));
	portno = 9876;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) cout << ("ERROR on binding") << endl;
}

//function to establish the server client connection:
void connectionEst()
{
	//establish connection:
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if (newsockfd < 0) cout << ("ERROR on accept") << endl;
}

//int to string converter for convenience:
string intToString(int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}

//Draw line:
void MyLine(Mat img, Point start, Point end)
{
	int thickness = 10;
	cv::line(img,start,end,Scalar(0, 0, 256),thickness, 4);
}

//log objects detected in console
void logObject(vector<Object> myObjects)
{
	cout << myObjects.size() << " objects detected!" << endl;
	for (int i = 0; i < myObjects.size(); i++){
		cout << "Object detected at (" + intToString(myObjects.at(i).getXPos()) + " , " + intToString(myObjects.at(i).getYPos()) + ")" << endl;
	}




		int allX = 0;
		int allY = 0;
		//calculate the average location of the geese:
		for (int i = 0; i < myObjects.size(); i++)
		{
			allX = allX + myObjects.at(i).getXPos();
			allY = allY + myObjects.at(i).getYPos();
		}
		allX = allX/myObjects.size();
		allY = allY/myObjects.size();
		cout << "X-average is: " << allX << " and Y-average is: " << allY << endl;

		if(allX<M2.x)
		{
			if(allY<M1.y)
			{
				cout << "Object detected on Q1!" << endl;
				theCMD[0] = '1';
			}
			else
			{
				cout << "Object detected on Q3!" << endl;
				theCMD[2] = '1';
			}
		}
		else
		{
			if(allY<M1.y)
			{
				cout << "Object detected on Q2!" << endl;
				theCMD[1] = '1';
				//write(newsockfd,cmdQ2,4);
			}
			else
			{
				cout << "Object detected on Q4!" << endl;
				theCMD[3] = '1';
				//write(newsockfd,cmdQ4,4);
			}
		}

}

//Splitting the field into quadrants
void drawGrid(Mat img, Point P1, Point P2, Point P3, Point P4)
{
	//P1-----M2-----P3
	//			   //
	//M1-----C1-----M3
	//		     	  //
	//P2-----M4-----P4

	M1.x = ((P2.x-P1.x)/2)+P1.x;
	M1.y = ((P2.y-P1.y)/2)+P1.y;
	M2.x = ((P3.x-P1.x)/2)+P1.x;
	M2.y = ((P3.y-P1.y)/2)+P1.y;
	M3.x = ((P4.x-P3.x)/2)+P3.x;
	M3.y = ((P4.y-P3.y)/2)+P3.y;
	M4.x = ((P4.x-P2.x)/2)+P2.x;
	M4.y = ((P4.y-P2.y)/2)+P2.y;

	MyLine(img, M1, M3);
	MyLine(img,M2,M4);

}

//User input to select the ROI (shift+left click = NW, ctrl+lef click = SW, shift + right click = NE, ctrl + right click = SE, alt + right click = submit new ROI):
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (flags == (EVENT_FLAG_SHIFTKEY + EVENT_FLAG_LBUTTON))
	{
		cout << "Higher left point - position (" << x << ", " << y << ")" << endl;
		P1 = Point(x, y);
	}
	else if (flags == (EVENT_FLAG_RBUTTON + EVENT_FLAG_SHIFTKEY))
	{
		cout << "Higher right point - position (" << x << ", " << y << ")" << endl;
		P3 = Point(x, y);
		MyLine(src, P1, P3);
	}
	else if (flags == (EVENT_FLAG_LBUTTON + EVENT_FLAG_CTRLKEY))
	{
		cout << "Lower left point - position (" << x << ", " << y << ")" << endl;
		P2 = Point(x, y);
		MyLine(src, P1, P2);
	}
	else if (flags == (EVENT_FLAG_RBUTTON + EVENT_FLAG_CTRLKEY))
	{
		cout << "Lower right point - position (" << x << ", " << y << ")" << endl;
		P4 = Point(x, y);
		MyLine(src, P3, P4);
		MyLine(src, P2, P4);
	}
	else if (flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_SHIFTKEY))
	{
		cout << "New ROI specified!" << endl;
		src = cv::Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
		cameraFeed = cv::Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
		Rect roi(P1.x*3,P1.y*3, (P3.x-P1.x)*3, (P2.y-P1.y)*3);
		cropped = clean(roi);
		//imwrite("cropper.jpg",cropped);

	}
	else if (flags == (EVENT_FLAG_RBUTTON + EVENT_FLAG_LBUTTON))
	{

		drawGrid(src,P1,P2,P3,P4);
		theBackground = cameraFeed;
		pMOG2->apply(cameraFeed, fgMaskMOG2,0);
		//////////////////////////////////find and list contours//////////////////////////////
		vector <Object> objects;
		std::vector<std::vector<cv::Point> > contours;
		vector<Vec4i> hierarchy;
    		cv::Mat contourOutput = fgMaskMOG2.clone();
    		cv::findContours( contourOutput, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
    		//trying to locate each seperate object:
    		bool objectFound = false;
    		for(int i = 0; i < 4; i++)
		{
			theCMD[i]='0';
		}
		if (hierarchy.size() > 0)
		{
			int numObjects = hierarchy.size();
			//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
			if (numObjects<MAX_NUM_OBJECTS)
			{
				for (int index = 0; index >= 0; index = hierarchy[index][0])
				{
					Moments moment = moments((cv::Mat)contours[index]);
					double area = moment.m00;
					//if the area is less than 20 px by 20px then it is probably just noise
					//if the area is the same as the 3/2 of the image size, probably just a bad filter
					//we only want the object with the largest area so we safe a reference area each
					//iteration and compare it to the area in the next iteration.
					if (area>MIN_OBJECT_AREA)
					{
						Object object;

						object.setXPos(moment.m10 / area);
						object.setYPos(moment.m01 / area);

						objects.push_back(object);

						objectFound = true;

					}

				}
				//let user know you found an object
				if ((objectFound == true)&&backgroundSet)
				{

					logObject(objects);
				}
				else if((objectFound == false)&&backgroundSet)
				{
						cout << "No objects detected!" << endl;
						for(int i = 0; i < 4; i++)
						{
							theCMD[i]='0';
						}

				}

			}

			else
			{
				cout << "too much noise!" << endl;
			}

		}

			imwrite("test1.jpg",clean);

			imwrite("cropper.jpg",cropped);
			backgroundSet = true;

	}
	else if (flags == (EVENT_FLAG_MBUTTON))
	{

			printf("Sending command: %s\n",theCMD);
			connectionEst();
			write(newsockfd,theCMD,4);

	}
}


int main ( int argc,char **argv ) {

	//initialize server:
	serverInit();


	//Background subtraction:
	pMOG2 = createBackgroundSubtractorMOG2(500,myThreshold,false); //MOG approach testing background subtraction


	cv::Mat img;


    	//set resolution of capture
    	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);



   //check if input is working as expected:
    if (!capture.open()) {cerr<<"Error opening the camera"<<endl;return -1;}

    waitKey(1000);
    while(1)
    {




		capture.grab();
		capture.retrieve (img);
		capture.retrieve (clean);
	   	int rows = img.rows;
		int cols = img.cols;

		resize(img, img, Size(cols/3,rows/3));

	   //return -1 if there is a problem reading frames:
	   	if(!img.data)
	   	{
	   		return -1;
	   	}

	  	 //view original camera feed:
	   	string windowName = "Original Feed: " + intToString(cols) + " x " + intToString(rows);
	   	circle(img,C1,1,Scalar(0, 256, 0),5,0);
	   	//imshow(windowName, img);

		//Adjust RIO (move this to one function for better organized code):
		Mat black(img.rows, img.cols, img.type(), cv::Scalar::all(0));
		Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
		vector< vector<Point> >  co_ordinates;
		co_ordinates.push_back(vector<Point>());
		co_ordinates[0].push_back(P1);
		co_ordinates[0].push_back(P3);
		co_ordinates[0].push_back(P4);
		co_ordinates[0].push_back(P2);
		drawContours(mask, co_ordinates, 0, Scalar(255), CV_FILLED, 8);
		img.copyTo(src, mask);
		img.copyTo(cameraFeed, mask);

		//circle(src,C1,1,Scalar(0, 256, 0),5,0);
		namedWindow("Adjust RIO",CV_WINDOW_AUTOSIZE);
		imshow("Adjust RIO", src);
		setMouseCallback("Adjust RIO", CallBackFunc, NULL);
		//imshow("Area to detect",cameraFeed);

		//show background subtraction in action, shows the outlines of the object on the field:
		if(backgroundSet)
		{
			imshow("FG Mask MOG 2", fgMaskMOG2);
		}






		waitKey(30);
 	}
}

//compile command: g++ the_main.cpp -o  finalMain -I/usr/local/include/ -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_bgsegm -lopencv_video
