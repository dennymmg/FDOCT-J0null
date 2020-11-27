/*
Code to display Bscans obtained using J0-null method in FD-OCT
		based on mehtod (a) subtraction in the Fourier domain

Author: Denny
		Nov 2020
the keys and their purposes are given below   
A - enables Accumulate mode
L - enables Live mode
] - increases threshold for display by 3dB
[ - decreases threshold for display by 3dB
f - increases the Fudge factor by 0.005
g - decreases the fudGe factor by 0.005
b - displays the non-subtracted Bscan (toggle key)
s - Saves the current Bscan to disk
d - save the current background frame S(k) 
	so as to Divide by the interferograms by S(k) for subsequent bscans 
'+' - increases exposure time by 100 microseconds
'-' - decreases exposure time by 100 microseconds
Esc or x - quit the program
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
	bool expchanged = false, accummode = false, refreshkeypressed = false, skeypressed = false, doneflag = false, dkeypressed = false;
	bool dir_created = false;
	float fudgefactor = 1.000;
	double minVal, maxVal, bscanthreshold = 72.0;
	int fps, dt, key;
	double spectcount = 0, spectnumavgs = 1000;	

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
	char textbuffer[80];

	namedWindow("Bscan", 0); // 0 = WINDOW_NORMAL
	moveWindow("Bscan", 500, 0);

	namedWindow("Status", 0); // 0 = WINDOW_NORMAL
	moveWindow("Status", 0, 600);

	ifstream infile("spin.ini");
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

	// inputs from ini file
	if (infile.is_open())
	{
		// skip the first 26 lines - they contain camera settings
		for ( int i = 0; i < 26; i++)
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
		setCamera(pCam); // set for trigger mode
		
		unsigned int w, h, camtime;
	
		w = pCam->Width.GetValue();
		h = pCam->Height.GetValue();
		camtime = pCam->ExposureTime.GetValue();

		FDOCT oct;
		oct.setWidthHeight(w,h);
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
		
		Mat m, Sk, mbg, bscantemp, mvector;
		Mat data_y, data_yb;
		Mat J, S, bscan, jscan;
		J = Mat::zeros(Size(numdisplaypoints, h), CV_64F);			// Size(cols,rows)
		S = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
		Sk = Mat::ones(Size(w,h),CV_16UC1);
		Mat jdiff, positivediff, bscanlog, bscandb, tempmat, bscandisp, cmagI;
	
		resizeWindow("Bscan", h, numdisplaypoints);		// (width,height)
		int ret;

		while (1)	//camera frames acquisition loop, which is inside the try
		{
			res = 0;
			STOP = FALSE;	
		
			// write J to Arduino
			write(fd, "J", sizeof("J"));

			// poll to read a character back from Arduino 
			while (STOP==FALSE) 
			{
			  /* loop for input */
			  res = read(fd,buf,255);   /* returns after 1 char has been input */
			  buf[res]=0;               /* so we can printf... */
			  if (res > 0) STOP=TRUE;
			}
			
			// now read a J0Null background image from the camera,
			// 		compute bscan and accumulate to Mat J
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
					m.convertTo(data_yb, CV_64F);
				}
			}

			// pResultImage has to be released to avoid buffer filling up
			pResultImage->Release();

			if(ret == 1)
			{
				oct.readInterferogram(data_yb);
				oct.dividebySpectrum(Sk);
				bscantemp = oct.computeBscan();	

				// accumulate the J0Null background bscans to J
				accumulate(bscantemp, J);
				fps++; 

			} // end of if ret == 1 block 
	

			res = 0;
			STOP = FALSE;
			tcflush(fd, TCIFLUSH);

			// write K to Arduino
			write(fd, "K", sizeof("K"));

			// poll to read a character back from Arduino 
			while (STOP==FALSE) 
			{
			  /* loop for input */
			  res = read(fd,buf,255);   /* returns after 1 char has been input */
			  buf[res]=0;               /* so we can printf... */
			  // cout << endl << "Received from Arduino: " << buf <<"  - " << res << "byte" << endl;
			  if (res > 0) STOP=TRUE;
			}

			// now read a image from the camera,  
			// compute bscan and accumulate to Mat S

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
					m.convertTo(data_y, CV_64F);
				}
			}

			// pResultImage has to be released to avoid buffer filling up
			pResultImage->Release();

			if(ret == 1)
			{
				oct.readInterferogram(data_y);
				oct.dividebySpectrum(Sk);
				bscantemp = oct.computeBscan();	

				// accumulate the Signal bscans to S
				accumulate(bscantemp, S);

			} // end of if ret == 1 block 
	

			tcflush(fd, TCIFLUSH);

			transpose(J, jscan);

			transpose(S, bscan);

			jdiff = bscan - fudgefactor*jscan;	// these are in linear scale
			
			jdiff.copyTo(positivediff);		// just to initialize the Mat
			makeonlypositive(jdiff, positivediff);
			positivediff += 0.000001;			// to avoid log(0)
			log(positivediff, bscanlog);				// switch to logarithmic scale
			bscandb = 20.0 * bscanlog / 2.303;
			bscandb.row(4).copyTo(bscandb.row(1));	// masking out the DC in the display
			bscandb.row(4).copyTo(bscandb.row(0));
			tempmat = bscandb.rowRange(0, numdisplaypoints);
			tempmat.copyTo(bscandisp);
			bscandisp = max(bscandisp, bscanthreshold);
			normalize(bscandisp, bscandisp, 0, 1, NORM_MINMAX);	// normalize the log plot for display
			bscandisp.convertTo(bscandisp, CV_8UC1, 255.0);
			applyColorMap(bscandisp, cmagI, COLORMAP_JET);
			imshow("Bscan", cmagI);


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

			if (accummode == false)
			{
				J = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
				S = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
			}

			if(dkeypressed == true)
			{
				if(spectcount < spectnumavgs)
				{
					Sk = Sk + mbg;
				}
				if(spectcount == spectnumavgs)
				{
					Sk = Sk / spectcount;
				}
			}
			spectcount ++;
			
			if (accummode == true && refreshkeypressed == true)
			{
				J = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
				S = Mat::zeros(Size(numdisplaypoints, h), CV_64F);
				refreshkeypressed = false;
			}

			gettimeofday(&tv,NULL);	
			time_end = 1000000 * tv.tv_sec + tv.tv_usec;	
			// update the image windows
			dt = time_end - time_start;

			if(dt > 1000000) // 1 second in microseconds 
			{
				m.copyTo(mvector);
				mvector.reshape(0, 1);	//make it into a row array
				minMaxLoc(mvector, &minVal, &maxVal);
				sprintf(textbuffer, "fps = %d  Max I = %d", fps, int(floor(maxVal)));
				firstrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
			
				if (accummode == false)	
					sprintf(textbuffer, " Live mode ");
				else
					sprintf(textbuffer, "Accum. mode ");

				secrowofstatusimgRHS = Mat::zeros(cv::Size(300, 50), CV_64F);
				putText(statusimg, textbuffer, Point(300, 80), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				
				resizeWindow("Status", 600, 300);
				imshow("Status", statusimg);
			
				fps = 0;
				gettimeofday(&tv,NULL);	
				time_start = 1000000 * tv.tv_sec + tv.tv_usec;	
			}		

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
				bscanthreshold += 3;
				sprintf(textbuffer, "Threshold = %3.1f", bscanthreshold);
				thirdrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 130), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				imshow("Status", statusimg);
				break;

			case '[':
				bscanthreshold -= 3;
				sprintf(textbuffer, "Threshold = %3.1f", bscanthreshold);
				thirdrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 130), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				imshow("Status", statusimg);
				break;
		
			case 'a':
			case 'A':
				// accumulate mode
				accummode = true;
				break;

			case 'l':
			case 'L':
				// live mode
				accummode = false;
				break;

			case 'r':
			case 'R':
				// refresh key pressed
				refreshkeypressed = true;
				break;

			case 's':			
			case 'S':
				// save bscan			
				skeypressed = true;
				break;

			case 'd':
			case 'D':
				// save the spectrum
				dkeypressed = true;
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
	unsigned int w, h, camspeed, burstframecount,triggerdelay, camtime, camgain = 1, bpp;
	unsigned int offsetx = 0, offsety = 0;
	unsigned int cambinx, cambiny;
	
	ifstream infile("spin.ini");
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
		infile >> burstframecount;
		infile >> tempstring;
		infile >> triggerdelay;
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
	pCam->TriggerSelector.SetValue(TriggerSelector_FrameBurstStart);
	pCam->AcquisitionBurstFrameCount.SetValue(burstframecount);
	pCam->TriggerSource.SetValue(TriggerSource_Line0);
	pCam->TriggerActivation.SetValue(TriggerActivation_AnyEdge);
	pCam->TriggerMode.SetValue(TriggerMode_On);
	pCam->TriggerDelay.SetValue(triggerdelay);
	cout << "Camera set to trigger mode ON \n\t with trigger source as Line0, \n\t trigger selector as FrameBurstStart and \n\t AcquisitionBurstFrameCount set  to " << burstframecount << "\n\t Trigger delay set to " << triggerdelay<< endl;

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
