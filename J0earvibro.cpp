/*
Code to display Bscans obtained using J0-null method in FD-OCT
		based on mehtod (a) subtraction in the Fourier domain

Author: Denny
		August 2021

the keys and their purposes are given below   
4 - increments the number of averages by 5 in Average mode
3 - decrements the number of averages by 5 in Average mode
] - increases threshold for display by 3dB
[ - decreases threshold for display by 3dB
f - increases the Fudge factor by 0.005
g - decreases the fudGe factor by 0.005
s - Saves the current Bscan to disk
'+' - increases exposure time by 100 microseconds
'-' - decreases exposure time by 100 microseconds
'a' - to enter accumulate mode
'l' - to enter live mode
Esc or x - quits the program

i - to display ROI
the arrow keys can be used to control the position and size of ROI
By default, arrow keys allow to control position of ROI
Press 'r' to change size of ROI using arrow keys
Press 't' to go back to changing position of ROI
Press 'p' to display ROI stats for X1+X2
Press 'm' to display ROI stats for X2-X1

*/

/*
// Date: August 2021

// Protocol for vibration detection
// Camera runs in non-triggered mode
// Speaker always on
// Piezo always on with amplitude V (such that m_bias is between 0 and 2.303), say V = 4.0

// step 1: 5 frames are taken -- bscans computed and averaged to X1
// step 2: Program sends a char 'P' (phase shift signal) to the Arduino
// step 3: Arduino receives P and sends HIGH through pin10 (Port B2)
// step 4: High on Pin 10 causes the phase shift of 180
// step 5: the program discards the next frame from the camera
// step 6: 5 frames are taken -- bscans computed and averaged to X2
// step 7: displays X2 - X1
// step 8: go back to step1

Calculation of vibration amplitude

For lambdacentre = 886.1 nm, deltalambda = 35.56 nm

	First zero of J0(x) occurs at x = M = 2.405
		2 * k * A = M
	==> 2 * 2*pi/lambdac * A = 2.405 
	==> A = 2.405 * lambdac / (4*pi)
		  = 2.405 * 886.1 / (4*pi) nm
		  = 169.6 nm is the vibration amplitude correpsonding to J0 Null

Vibration amplitude, V = (X1-X2)/(X1+X2) * A

*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include "FDOCT.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <fcntl.h> // Contains file controls like O_RDWR
#include <sys/time.h> // gettimeofday()
#include <sys/stat.h> // this is for mkdir
#include <array>

#define BAUDRATE B115200
#define ARDUINO "/dev/ttyACM0"
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE; 

using namespace cv;
using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;

// Function declarations
void setCamera(CameraPtr pCam);
inline void makeonlypositive(Mat& src, Mat& dst);
void matwrite(const string& filename, const Mat& mat);
inline void savematasbin(char* p, char* d, char* f, Mat m);
inline void savematasimage(char* p, char* d, char* f, Mat m);

int main()
{
	int result = 0;
	bool bgframestaken = false, accummode = false;
	bool expchanged = false, skeypressed = false, doneflag = false;
	bool dir_created = false;
	double minval, maxval, minvalsum, maxvalsum, meanvalsum, minvaldiff, maxvaldiff, meanvaldiff, bscanthreshold = 72.0;
	float fudgefactor = 1.000;
	int dt, key;
	double fps = 0.0, framecount = 0.0;
	double V = 0.0, A = 0.0; // vibration amplitude
	A = 169.6; // vibration amplitude in nm coresponding to J0 Null for lambda = 886.1 nm
	system("clear");
	// Print application build information
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

	SystemPtr system = System::GetInstance();
    
	// Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
    
	unsigned int numCameras = camList.GetSize();
    cout << "Number of cameras detected: " << numCameras << endl << endl;
	if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        cout << "Camera not detected. " << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
	CameraPtr pCam = nullptr;
	pCam = camList.GetByIndex(0);

	Mat statusimg = Mat::zeros(cv::Size(600, 300), CV_64F);
	Mat firstrowofstatusimg = statusimg(Rect(0, 0, 600, 50)); // x,y,width,height
	Mat secrowofstatusimg = statusimg(Rect(0, 50, 600, 50));
	Mat secrowofstatusimgRHS = statusimg(Rect(300, 50, 300, 50));
	Mat thirdrowofstatusimg = statusimg(Rect(0, 100, 600, 50));
	Mat fourthrowofstatusimg = statusimg(Rect(0, 150, 600, 50));
	char textbuffer[80];

	namedWindow("Interferogram", 0); // 0 = WINDOW_NORMAL
	moveWindow("Interferogram", 0, 0);

	namedWindow("X1+X2", 0); // 0 = WINDOW_NORMAL
	moveWindow("X1+X2", 900, 0);

	namedWindow("X2-X1", 0); // 0 = WINDOW_NORMAL
	moveWindow("X2-X1", 900, 400);
	
	namedWindow("X2", 0); // 0 = WINDOW_NORMAL
	moveWindow("X2", 500, 0);

	namedWindow("Status", 0); // 0 = WINDOW_NORMAL
	moveWindow("Status", 0, 600);

	ifstream infile("spint.ini");
	string tempstring;

	int fd, res;	
	struct termios oldtio, newtio;
    char buf[255];

	fd = open(ARDUINO, O_RDWR | O_NOCTTY ); 
    if (fd < 0) 
	{	
		cout << "Arduino not detected..";
		perror(ARDUINO);
		exit(-1); 
	}

	tcgetattr(fd,&oldtio); /* save current port settings */
	
	bzero(&newtio, sizeof(newtio));

	newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

	newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 char is received */
	
	tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);

	char dirdescr[60];
	sprintf(dirdescr, "_");
	char dirname[80];
	char filename[20];
	char filenamec[20];
	unsigned int indexi = 0;
	char pathname[140];

	char lambdamaxstr[40];
	char lambdaminstr[40];
	double lambdamin, lambdamax;
	unsigned int averagescount, numdisplaypoints, fftpointsmultiplier;
	char thresholdstr[40];
	bool ROIflag = false;
	unsigned int ROIstartrow, ROIendrow, ROIstartcol, ROIendcol, heightROI, widthROI, ROIstepsize = 1;
	double ROImeanVal, ROImaxVal;
	unsigned int binvaluex = 1, binvaluey = 1;

	// inputs from ini file
	if (infile.is_open())
	{
		// skip the first 22 lines - they contain camera settings
		for ( int i = 0; i < 22; i++)
		{
			infile >> tempstring;
		}

		infile >> tempstring;
		infile >> lambdaminstr;
		infile >> tempstring;
		infile >> lambdamaxstr;
		infile >> tempstring;
		infile >> averagescount;
		infile >> tempstring;
		infile >> numdisplaypoints;
		infile >> tempstring;
		infile >> fftpointsmultiplier;
		infile >> tempstring;
		infile >> thresholdstr;
		infile >> tempstring;
		infile >> dirdescr;
		infile >> tempstring;
		infile >> ROIstartrow;
		infile >> tempstring;
		infile >> ROIendrow;
		infile >> tempstring;
		infile >> ROIstartcol;
		infile >> tempstring;
		infile >> ROIendcol;
		infile >> tempstring;
		infile >> binvaluex;
		infile >> tempstring;
		infile >> binvaluey;
		
		infile.close();
		lambdamin = atof(lambdaminstr);
		lambdamax = atof(lambdamaxstr);
		bscanthreshold = atof(thresholdstr);
		cout << "lambdamin set to " << lambdamin << " ..." <<endl;
		cout << "lambdamax set to " << lambdamax << " ..." << endl;
	}

	try 
	{
		// Initialize camera
		pCam->Init();
		setCamera(pCam); // set the camera for asynchronous mode
		
		unsigned int w, h, opw, oph, camtime;
		w = pCam->Width.GetValue();
		h = pCam->Height.GetValue();
		opw = w / binvaluex;
		oph = h / binvaluey;
		cout << "binvaluex set to " << binvaluex << " ..." <<endl;
		cout << "binvaluey set to " << binvaluey << " ..." << endl;
		cout << "width set to " << opw << " ..." <<endl;
		cout << "height set to " << oph << " ..." << endl;
		camtime = pCam->ExposureTime.GetValue();

		FDOCT oct;
		oct.setWidthHeight(opw,oph);
		oct.setLambdaMinMax(lambdamin,lambdamax);
		oct.setfftpointsmultiplier(fftpointsmultiplier);
		oct.setnumdisplaypoints(numdisplaypoints);
		oct.initialCompute();		

		cout << "Acquiring images " << endl;
		pCam->BeginAcquisition();
		ImagePtr pResultImage;
		ImagePtr convertedImage;

		unsigned long time_start, time_end;	
		struct timeval tv;
		gettimeofday(&tv,NULL);	
		time_start = 1000000 * tv.tv_sec + tv.tv_usec;	
		
		Mat m, opm, Sk, bscantemp, mvector, bvector;
		Mat data_y, data_yb;
		Mat B, X1, X2, x1, x2;
		X1 = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);			// Size(cols,rows)
		X2 = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);
		Sk = Mat::ones(Size(opw,oph),CV_16UC1);
		Mat jdiff, positivediff, bscanlog, bscandb, tempmat1, bscandisp, cmagI, tempmat2;
		Mat ROI, tempmatsum, tempmatdiff;	
		// ROI parameters
		Point ROItopleft(ROIstartcol,ROIstartrow), ROIbottright(ROIendcol,ROIendrow);
		enum ROIoption{position=1,size};
		enum displayROI{plus=1,minus, vib};
		int selectedROIoption, displayROI;
		selectedROIoption = position;
		displayROI = minus; // display ROI stats for X1-X2
	
		resizeWindow("X1+X2", oph, numdisplaypoints);		// (width,height)
		resizeWindow("X2-X1", oph, numdisplaypoints);		// (width,height)
		resizeWindow("X2", oph, numdisplaypoints);		// (width,height)
		int ret;
		unsigned int ii; // index

		while (1)	//camera frames acquisition loop, which is inside the try
		{
			if(bgframestaken == false)
			{
				B = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);
				// read a set of background images from the camera,
				// 		compute bscan and accumulate to Mat J
				res = 0;
			
				for(ii = 1; ii <= averagescount; ii++)
				{			
					ret = 0;
					// save one image to Mat m
					while(ret == 0)
					{
						pResultImage = pCam->GetNextImage();
						if (pResultImage->IsIncomplete())
						{
							ret = 0;
						}
						else
						{
							ret = 1;
							convertedImage = pResultImage;
							m = Mat(h, w, CV_16UC1, convertedImage->GetData(), convertedImage->GetStride());
							// binning (averaging)
							resize(m, opm, Size(), 1.0 / binvaluex, 1.0 / binvaluey, INTER_AREA);
							opm.convertTo(data_yb, CV_64F);
						}
					}

					// pResultImage has to be released to avoid buffer filling up
					pResultImage->Release();
					//cout << "Read  bg image " << ii << " from the camera" << endl;	
					imshow("Interferogram", opm);

					if(ret == 1)
					{
						oct.readInterferogram(data_yb);
						oct.dividebySpectrum(Sk);
						bscantemp = oct.computeBscan();	

						// accumulate the set of background bscans to B
						accumulate(bscantemp, B);

					} // end of if ret == 1 block 
				} // end of for loop ii <= averagescount

				bgframestaken = true;				
			} // end of if(bgframestaken == false)

			// write 'I' to Arduino - I for in phase
			write(fd, "I", sizeof("I"));
			// read a set of images from the camera,  
			// compute bscan and accumulate to Mat X1
			for(ii = 1; ii <= averagescount; ii++)
			{			
				ret = 0;
				// save one image to Mat m
				while(ret == 0)
				{
					pResultImage = pCam->GetNextImage();
					if (pResultImage->IsIncomplete())
					{
						ret = 0;
					}
					else
					{
						ret = 1;
						convertedImage = pResultImage;
						m = Mat(h, w, CV_16UC1, convertedImage->GetData(), convertedImage->GetStride());
						// binning (averaging)
						resize(m, opm, Size(), 1.0 / binvaluex, 1.0 / binvaluey, INTER_AREA);
						opm.convertTo(data_y, CV_64F);
					}
				}

				// pResultImage has to be released to avoid buffer filling up
				pResultImage->Release();

				tcflush(fd, TCIFLUSH);
				imshow("Interferogram", opm);

				if(ret == 1)
				{
					oct.readInterferogram(data_y);
					oct.dividebySpectrum(Sk);
					bscantemp = oct.computeBscan();	

					// accumulate the bscans to X1
					accumulate(bscantemp, X1);
					framecount ++;
					fps++;
				} // end of if ret == 1 block 

			} // end of for loop ii <= averagescount

			// write 'O' to Arduino - O for out of phase
			write(fd, "O", sizeof("O"));
			
			// delay of 10 ms
			//usleep(1000);

			// read a set of images from the camera,  
			// compute bscan and accumulate to Mat X2
			for(ii = 1; ii <= averagescount; ii++)
			{			
				ret = 0;
				// save one image to Mat m
				while(ret == 0)
				{
					pResultImage = pCam->GetNextImage();
					if (pResultImage->IsIncomplete())
					{
						ret = 0;
					}
					else
					{
						ret = 1;
						convertedImage = pResultImage;
						m = Mat(h, w, CV_16UC1, convertedImage->GetData(), convertedImage->GetStride());
						// binning (averaging)
						resize(m, opm, Size(), 1.0 / binvaluex , 1.0 / binvaluey, INTER_AREA);
						opm.convertTo(data_y, CV_64F);
					}
				}

				// pResultImage has to be released to avoid buffer filling up
				pResultImage->Release();

				imshow("Interferogram", opm);

				if(ret == 1)
				{
					oct.readInterferogram(data_y);
					oct.dividebySpectrum(Sk);
					bscantemp = oct.computeBscan();	

					// accumulate the bscans to X2
					accumulate(bscantemp, X2);
					fps++;
				} // end of if ret == 1 block 

			} // end of for loop ii <= averagescount

			tcflush(fd, TCIFLUSH);
			jdiff = X2 - B;
			jdiff.copyTo(positivediff);		// just to initialize the Mat
			makeonlypositive(jdiff, positivediff);
			if (accummode == true)
				positivediff = positivediff / (1.0 * framecount);
			else
				positivediff = positivediff / (1.0 * averagescount);	
			positivediff += 0.000000001;			// to avoid log(0)
			log(positivediff, bscanlog);				// switch to logarithmic scale
			bscandb = 20.0 * bscanlog / 2.303;
			bscandb.row(4).copyTo(bscandb.row(1));	// masking out the DC in the display
			bscandb.row(4).copyTo(bscandb.row(0));
			tempmat1 = bscandb.colRange(0, numdisplaypoints);
			if(binvaluex > 1 || binvaluey > 1)
			{
				resize(tempmat1,tempmat2,Size(),binvaluex,binvaluey,INTER_AREA);
			}
			else
			{
				tempmat1.copyTo(tempmat2);
			}
			tempmat2.copyTo(bscandisp);
			bscandisp = max(bscandisp, bscanthreshold);
			normalize(bscandisp, bscandisp, 0, 1, NORM_MINMAX);	// normalize the log plot for display
			bscandisp.convertTo(bscandisp, CV_8UC1, 255.0);
			applyColorMap(bscandisp, cmagI, COLORMAP_JET);
			imshow("X2", cmagI);
			//cout << "framecount = " << framecount << endl;
			// absolute diff --- uncomment the next two lines for absolute diff 	
			//absdiff(X1,X2,jdiff);
			//positivediff = jdiff / (1.0 * averagescount);	
							
// only positive --- comment the next four lines if negative values should NOT be set to zero
			jdiff = (X2-X1);
			jdiff.copyTo(positivediff);
			//makeonlypositive(jdiff, positivediff);
			if (accummode == true)
				positivediff = positivediff / (1.0 * framecount);
			else
				positivediff = positivediff / (1.0 * averagescount);	
			//positivediff += 0.000000001;			// to avoid log(0)
			// comment the below two lines for linear scale
			//log(positivediff, bscanlog);				// switch to logarithmic scale
			//bscandb = 20.0 * bscanlog / 2.303;
			// uncomment the below line for linear scale
			//positivediff.copyTo(bscandb);
			//bscandb.row(4).copyTo(bscandb.row(1));	// masking out the DC in the display
			//bscandb.row(4).copyTo(bscandb.row(0));
			//tempmat1 = bscandb.colRange(0, numdisplaypoints);
			tempmat1 = positivediff.colRange(0, numdisplaypoints);
			if(binvaluex > 1 || binvaluey > 1)
			{
				resize(tempmat1,tempmatdiff,Size(),binvaluex,binvaluey,INTER_AREA);
			}
			else
			{
				tempmat1.copyTo(tempmatdiff);
			}
			tempmatdiff.copyTo(bscandisp);
			bscandisp = max(bscandisp, bscanthreshold);
			normalize(bscandisp, bscandisp, 0, 1, NORM_MINMAX);	// normalize the log plot for display
			bscandisp.convertTo(bscandisp, CV_8UC1, 255.0);
			applyColorMap(bscandisp, cmagI, COLORMAP_JET);

			if (ROIflag == true)	
				rectangle(cmagI,ROItopleft,ROIbottright,Scalar(0,255,0),1, LINE_8);
			imshow("X2-X1", cmagI);

			jdiff = (X1 + X2 - 2*B); // sum
			jdiff.copyTo(positivediff);		// just to initialize the Mat
			makeonlypositive(jdiff, positivediff);
			if (accummode == true)
				positivediff = positivediff / (1.0 * framecount);
			else
				positivediff = positivediff / (1.0 * averagescount);	
			positivediff += 0.000000001;			// to avoid log(0)
			// comment the below two lines for linear scale
			log(positivediff, bscanlog);				// switch to logarithmic scale
			bscandb = 20.0 * bscanlog / 2.303;
			// uncomment the below line for linear scale
			jdiff.copyTo(bscandb);
			bscandb.row(4).copyTo(bscandb.row(1));	// masking out the DC in the display
			bscandb.row(4).copyTo(bscandb.row(0));
			tempmat1 = bscandb.colRange(0, numdisplaypoints);
			if(binvaluex > 1 || binvaluey > 1)
			{
				resize(tempmat1,tempmatsum,Size(),binvaluex,binvaluey,INTER_AREA);
			}
			else
			{
				tempmat1.copyTo(tempmatsum);
			}
			tempmatsum.copyTo(bscandisp);
			bscandisp = max(bscandisp, bscanthreshold);
			normalize(bscandisp, bscandisp, 0, 1, NORM_MINMAX);	// normalize the log plot for display
			bscandisp.convertTo(bscandisp, CV_8UC1, 255.0);
			applyColorMap(bscandisp, cmagI, COLORMAP_JET);
			if (ROIflag == true)	
				rectangle(cmagI,ROItopleft,ROIbottright,Scalar(0,255,0),1, LINE_8);
			imshow("X1+X2", cmagI);

			if (skeypressed == true)
			{
				if(dir_created == false)
				{
					// create a directory with time stamp
					struct tm *timenow;
					time_t now = time(NULL);
					timenow = localtime(&now);
					strftime(dirname, sizeof(dirname), "%Y-%m-%d_%H_%M_%S-", timenow);
					strcat(dirname, dirdescr);
					mkdir(dirname, 0755);
					strcpy(pathname, dirname);
					strcat(pathname, "/");
					dir_created = true;
				}
				indexi++;
				sprintf(filename, "bscan%03d", indexi);
				savematasbin(pathname, dirname, filename, bscandb);
				savematasimage(pathname, dirname, filename, bscandisp);
				sprintf(filenamec, "bscanc%03d", indexi);
				savematasimage(pathname, dirname, filenamec, cmagI);

				skeypressed = false;
			}			

			if(accummode == false)
			{
				X1 = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);
				X2 = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);
				framecount = 0.0;
			}
			gettimeofday(&tv,NULL);	
			time_end = 1000000 * tv.tv_sec + tv.tv_usec;	
			// update the image windows
			dt = time_end - time_start;

			if(dt > 1000000) // 1 second in microseconds 
			{
				m.copyTo(mvector);
				mvector.reshape(0, 1);	//make it into a row array
				minMaxLoc(mvector, &minval, &maxval);
				sprintf(textbuffer, "fps = %d  Max val = %d", int(round(fps/dt*1e6)), int(floor(maxval)));
				firstrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				if(accummode == true)
					sprintf(textbuffer, "Accum mode");
				else if (averagescount == 1)	
					sprintf(textbuffer, "Live mode");
				else
					sprintf(textbuffer, "Avg mode N = %d",averagescount);

				secrowofstatusimgRHS = Mat::zeros(cv::Size(300, 50), CV_64F);
				putText(statusimg, textbuffer, Point(300, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				
				if(ROIflag == true)
				{
					// display max val of Bscan
					heightROI = ROIendrow - ROIstartrow;
					widthROI = ROIendcol - ROIstartcol;

					tempmatdiff(Rect(ROIstartcol,ROIstartrow,widthROI,heightROI)).copyTo(ROI);
					ROI.reshape(0, 1);	//make it into a row array
					minMaxLoc(ROI, &minvaldiff, &maxvaldiff);
					meanvaldiff = mean(ROI)(0);

					tempmatsum(Rect(ROIstartcol,ROIstartrow,widthROI,heightROI)).copyTo(ROI);
					ROI.reshape(0, 1);	//make it into a row array
					minMaxLoc(ROI, &minvalsum, &maxvalsum);
					meanvalsum = mean(ROI)(0);
					
					V = maxvaldiff / maxvalsum * A;
					if(selectedROIoption == position)
					{
						if(displayROI == plus)
							sprintf(textbuffer, "r 4size; '+': max=%d mean=%d", int(round(maxvalsum)), int(round(meanvalsum)));
						if(displayROI == minus)
							sprintf(textbuffer, "r 4size; '-': max=%d mean=%d", int(round(maxvaldiff)), int(round(meanvaldiff)));
						if(displayROI == vib)
							sprintf(textbuffer, "r 4size; Vmax = %d nm", int(round(V)));
					}
					
					if(selectedROIoption == size)
					{
						if(displayROI == plus)
							sprintf(textbuffer, "t 4pos; '+': max=%d mean=%d", int(round(maxvalsum)), int(round(meanvalsum)));
						if(displayROI == minus)
							sprintf(textbuffer, "t 4pos; '-': max=%d mean=%d", int(round(maxvaldiff)), int(round(meanvaldiff)));
						if(displayROI == vib)
							sprintf(textbuffer, "t 4pos; V = %d nm", int(round(V)));
					}
				}
				else
				{
					sprintf(textbuffer, " Press i for ROI ");
				}
				fourthrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0,190), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
	

				resizeWindow("Status", 600, 300);
				imshow("Status", statusimg);
			
				fps = 0;
				gettimeofday(&tv,NULL);	
				time_start = 1000000 * tv.tv_sec + tv.tv_usec;	
			}		

			//key = waitKey(3); // wait for keypress
			key = waitKey(3); // wait for keypress
			switch (key)
			{

			case 27: //ESC key
			case 'x':
			case 'X':
				doneflag = true;
				break;
			
			case '+':
			case '=':
				camtime = camtime + 100;
				expchanged = true;
				break;

			case '-':
			case '_':
				if (camtime < 8)	// spinnaker has a min of 8 microsec
				{
					camtime = 8;
					break;
				}
				camtime = camtime - 100;
				expchanged = true;
				break;
		
			case 'f':
				fudgefactor += 0.005;
				sprintf(textbuffer, "Fudge = %4.3f", fudgefactor);
				thirdrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 130), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				imshow("Status", statusimg);
				break;
	
			case 'g':
				fudgefactor -= 0.005;
				sprintf(textbuffer, "Fudge = %4.3f", fudgefactor);
				thirdrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 130), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				imshow("Status", statusimg);
				break;

			case ']':
				bscanthreshold += 3;  // for log scale
				//bscanthreshold += 200;  // for linear scale
				sprintf(textbuffer, "Threshold = %3.1f", bscanthreshold);
				thirdrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 130), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				imshow("Status", statusimg);
				break;

			case '[':
				bscanthreshold -= 3;  // for log scale
				//bscanthreshold -= 200;  // for linear scale
				sprintf(textbuffer, "Threshold = %3.1f", bscanthreshold);
				thirdrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 130), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				imshow("Status", statusimg);
				break;
		
			case '4':
				if (accummode == true)
					break;
				if (averagescount == 1)
					averagescount = 5; 
				else 
					averagescount += 5;
				bgframestaken = false;
				break;

			case '3':
				if (accummode == true)
					break;
				// decrement number of averages by 5
				if(averagescount >= 10)
					averagescount -= 5;
				else if(averagescount == 5)
					averagescount = 1;
				bgframestaken = false;
				break;

			case 's':			
				// save bscan			
				skeypressed = true;
				break;

			case 'b':
				// new bscan background			
				bgframestaken = false;
				break;

			case 'i':
				// toggle ROI display
				ROIflag = !ROIflag;
				break;
		
			case 'a':
				// accumulate mode
				accummode = true;
				averagescount = 1;
				//bgframestaken = false;
				break;

			case 'l':
				// live mode
				accummode = false;
				averagescount = 1;
				//bgframestaken = false;
				break;

	
				// keys to change ROI

			case 't':
				if (ROIflag == false)
					break;
				// select the ROI option to change position
				selectedROIoption = 1; // position
				break; 			 

			case 'r':
				if (ROIflag == false)
					break;
				// select the ROI option to change size
				selectedROIoption = 2; // size
				break; 
			 
			case 'p':
				if (ROIflag == false)
					break;
				// display the ROI stats for X1+X2
				displayROI = 1; // plus 
				break; 			 

			case 'm':
				if (ROIflag == false)
					break;
				// display the ROI stats for X2-X1
				displayROI = 2; // minus  
				break; 			 

			case 'v':
				if (ROIflag == false)
					break;
				// display the ROI stats for the vibration amplitude, V
				displayROI = 3; // V  
				break; 			 

			case 82: 							// up arrow key = R ?
				if (ROIflag == false)
					break;
				if(selectedROIoption == position)
				{
					// move ROI up
					if(ROIstartrow > 5)
					{
						ROIstartrow -= 1;
						ROIendrow -= 1;
					}
					ROItopleft.y = ROIstartrow;
					ROIbottright.y = ROIendrow;
				}
				if(selectedROIoption == size)
				{
					// blow up the ROI vertically
					if(ROIstartrow > 5) 
						ROIstartrow -= 1;
					if(ROIendrow < (oph*binvaluey-5))
						ROIendrow += 1;
					ROItopleft.y = ROIstartrow;
					ROIbottright.y = ROIendrow; 
				}
				break;

			case 84: 						// down arrow key = T ?
				if (ROIflag == false)
					break;
				if(selectedROIoption == position)
				{
					// move ROI down
					if(ROIendrow < (oph*binvaluey-5))
					{
						ROIstartrow += 1;
						ROIendrow += 1;
					}
					ROItopleft.y = ROIstartrow;
					ROIbottright.y = ROIendrow;
				}
				if(selectedROIoption == size)
				{
					// shrink the ROI vertically
					if(ROIendrow-ROIstartrow <= 3)
					{
						ROIstartrow -= 1;
						ROIendrow += 1;
					}
					else
					{
						ROIstartrow += 1;
						ROIendrow -= 1;
					}
					ROItopleft.y = ROIstartrow;
					ROIbottright.y = ROIendrow; 
				}
				break;

			case 81:						// left arrow key = Q ?
				if (ROIflag == false)
					break;
				if(selectedROIoption == position)
				{ 
					// move ROI left
					if(ROIstartcol > 5)
					{
						ROIstartcol -= 1;
						ROIendcol -= 1;
					}
					ROItopleft.x = ROIstartcol;
					ROIbottright.x = ROIendcol;
				}
				if(selectedROIoption == size)
				{
					// blow up the ROI horizontally
					if(ROIstartcol > 5) 
						ROIstartcol -= 1;
					if(ROIendcol < (numdisplaypoints*binvaluey-5))
						ROIendcol += 1;
					ROItopleft.x = ROIstartcol;
					ROIbottright.x = ROIendcol; 
				}
				break;

			case 83:						// right arrow key = S ?
				if (ROIflag == false)
					break;
				if(selectedROIoption == position)
				{ 
					// move ROI right
					if(ROIendcol < (numdisplaypoints*binvaluex-5))
					{
						ROIstartcol += 1;
						ROIendcol += 1;
					}
					ROItopleft.x = ROIstartcol;
					ROIbottright.x = ROIendcol;
				}
				if(selectedROIoption == size)
				{
					// shrink the ROI horizontally
					if(ROIendcol-ROIstartcol <= 3)
					{
						ROIstartcol -= 1;
						ROIendcol += 1;
					}
					else
					{
						ROIstartcol += 1;
						ROIendcol -= 1;
					}
					ROItopleft.x = ROIstartcol;
					ROIbottright.x = ROIendcol; 
				}
				break;

			default:
				break;
			}

			if (doneflag == 1)
			{
				break;
			}

			if (expchanged == true)
			{
				//Set exp with QuickSpin
				ret = 0;
				if (IsReadable(pCam->ExposureTime) && IsWritable(pCam->ExposureTime))
				{
					pCam->ExposureTime.SetValue(camtime);
					ret = 1;
				}
				if (ret == 1)
				{
					sprintf(textbuffer, "Exp time = %d ", camtime);
					secrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
					putText(statusimg, textbuffer, Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
					imshow("Status", statusimg);

				}
				else
				{
					sprintf(textbuffer, "CONTROL_EXPOSURE failed");
					secrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
					putText(statusimg, textbuffer, Point(0, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
					imshow("Status", statusimg);
				}

			} // end of if expchanged

		}
		
		pCam->EndAcquisition();
		pCam->DeInit();
		pCam = nullptr;

		// Clear camera list before releasing system
    	camList.Clear();
    	
		// Release system
    	system->ReleaseInstance();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	tcsetattr(fd,TCSANOW,&oldtio);

    return result;
}

// Function definitions
void setCamera(CameraPtr pCam)
{
	int result = 0;    
	unsigned int w, h, camspeed, camtime, camgain = 1, bpp;
	unsigned int offsetx = 0, offsety = 0;
	unsigned int cambinx, cambiny;
	
	ifstream infile("spint.ini");
	string tempstring;
	
	// inputs from ini file
	if (infile.is_open())
	{
		infile >> tempstring;
		infile >> tempstring;
		infile >> tempstring;
		// first three lines of ini file are comments
		infile >> camgain;
		infile >> tempstring;
		infile >> camtime;
		infile >> tempstring;
		infile >> bpp;
		infile >> tempstring;
		infile >> w;
		infile >> tempstring;
		infile >> h;
		infile >> tempstring;
		infile >> offsetx;
		infile >> tempstring;
		infile >> offsety;
		infile >> tempstring;
		infile >> camspeed;
		infile >> tempstring;
		infile >> cambinx;
		infile >> tempstring;
		infile >> cambiny;

		infile.close();
	}

	cout << "Initialising Camera settings ..." << endl;
	
	pCam->TLStream.StreamBufferHandlingMode.SetValue(StreamBufferHandlingMode_NewestOnly);
	pCam->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		
	// gain
	pCam->GainAuto.SetValue(GainAuto_Off);	
	pCam->Gain.SetValue(camgain);
	cout << "Gain set to " << pCam->Gain.GetValue() << " dB ..." << endl;

	// exposure time
	pCam->ExposureAuto.SetValue(ExposureAuto_Off);
	pCam->ExposureMode.SetValue(ExposureMode_Timed);
	pCam->ExposureTime.SetValue(camtime);
	cout << "Exp set to " << pCam->ExposureTime.GetValue() << " microsec ..." << endl;

	// bpp or cambitdepth 
	if (bpp == 16)
	{
		pCam->PixelFormat.SetValue(PixelFormat_Mono16);
		cout << "Pixel format set to " << pCam->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..." << endl;
	}
	
	// cambinx
	pCam->BinningHorizontal.SetValue(cambinx);
	cout << "BinningHorizontal set to " << pCam->BinningHorizontal.GetValue() << "..." << endl;

	// cambiny
	pCam->BinningVertical.SetValue(cambiny);
	cout << "BinningVertical set to " << pCam->BinningVertical.GetValue() << "..." << endl;
	
	// width 
	if (IsReadable(pCam->Width) && IsWritable(pCam->Width))
	{
		pCam->Width.SetValue(w);
	}
	else
	{
		cout << "Width not available..." << endl;
	}
	
	// height 
	if (IsReadable(pCam->Height) && IsWritable(pCam->Height))
	{
		pCam->Height.SetValue(h);
	}
	else
	{
		cout << "Height not available..." << endl;
	}

	// offsetx
	if (IsReadable(pCam->OffsetX) && IsWritable(pCam->OffsetX))
	{
		pCam->OffsetX.SetValue(offsetx);
	}
	else
	{
		cout << "Offset X not available..." << endl;
	}
	
	// offsety
	if (IsReadable(pCam->OffsetY) && IsWritable(pCam->OffsetY))
	{
		pCam->OffsetY.SetValue(offsety);
	}
	else
	{
		cout << "Offset Y not available..." << endl;
	}

	// frame rate
	pCam->AcquisitionFrameRateEnable.SetValue(1);
	pCam->AcquisitionFrameRate.SetValue(camspeed);
	cout << "Frame rate set to " << camspeed << endl;

	// set the hardware trigger	     
	pCam->TriggerMode.SetValue(TriggerMode_Off);
	cout << "Camera set to trigger mode OFF "<< endl;

}

inline void makeonlypositive(Mat& src, Mat& dst)
{
	// from https://stackoverflow.com/questions/48313249/opencv-convert-all-negative-values-to-zero
	max(src, 0, dst);

}

inline void savematasbin(char* p, char* d, char* f, Mat m)
{
	// saves a Mat m by writing to a binary file  f appending .ocv, both windows and unix versions
	// p=pathname, d=dirname, f=filename

#ifdef __unix__
	strcpy(p, d);
	strcat(p, "/");
	strcat(p, f);
	strcat(p, ".ocv");
	matwrite(p, m);
#else

	strcpy(p, d);
	strcat(p, "\\");		// imwrite needs path with \\ separators, not /, on windows
	strcat(p, f);
	strcat(p, ".ocv");
	matwrite(p, m);
#endif	

}

inline void savematasimage(char* p, char* d, char* f, Mat m)
{
	// saves a Mat m using imwrite as filename f appending .png, both windows and unix versions
	// p=pathname, d=dirname, f=filename

#ifdef __unix__
	strcpy(p, d);
	strcat(p, "/");
	strcat(p, f);
	strcat(p, ".png");
	imwrite(p, m);
#else

	strcpy(p, d);
	strcat(p, "\\");		// imwrite needs path with \\ separators, not /, on windows
	strcat(p, f);
	strcat(p, ".png");
	imwrite(p, m);
#endif	

}

// from http://stackoverflow.com/a/32357875/5294258
void matwrite(const string& filename, const Mat& mat)
{
	ofstream fs(filename, fstream::binary);

	// Header
	int type = mat.type();
	int channels = mat.channels();
	fs.write((char*)&mat.rows, sizeof(int));    // rows
	fs.write((char*)&mat.cols, sizeof(int));    // cols
	fs.write((char*)&type, sizeof(int));        // type
	fs.write((char*)&channels, sizeof(int));    // channels

												// Data
	if (mat.isContinuous())
	{
		fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
	}
	else
	{
		int rowsz = CV_ELEM_SIZE(type) * mat.cols;
		for (int r = 0; r < mat.rows; ++r)
		{
			fs.write(mat.ptr<char>(r), rowsz);
		}
	}
}
