/*
Code to display Bscans obtained in FD-OCT
	 using the method of subtraction in the Fourier domain

Author: Denny
		August 2021

The keys and their purposes are given below   

b - takes a fresh set of background frames
4 - increments the number of averages by 5 in Average mode
3 - decrements the number of averages by 5 in Average mode
] - increases threshold for display by 3dB
[ - decreases threshold for display by 3dB
s - Saves the current Bscan to disk
'+' - increases exposure time by 100 microseconds
'-' - decreases exposure time by 100 microseconds
Esc or x - quits the program

i - to display ROI
ROI control
t - set the top-left corner as the selected point
r - set the right-bottom corner as the selected point
up arrow key - move the selected point up by 5 pixels 
down arrow key - move the selected point down by 5 pixels
left arrow key - move the selected point to the left by 5 pixels
right arrow key - move the selected point to the right by 5 pixels
 
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include "FDOCT.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <sys/time.h> // gettimeofday()
#include <sys/stat.h> // this is for mkdir

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
	bool bgframestaken = false;
	bool expchanged = false, skeypressed = false, doneflag = false;
	bool dir_created = false;
	double minVal, maxVal, meanVal, bscanthreshold = 72.0;
	int dt, key;
	double fps = 0.0;	

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

	namedWindow("Bscan", 0); // 0 = WINDOW_NORMAL
	moveWindow("Bscan", 500, 0);

	namedWindow("Status", 0); // 0 = WINDOW_NORMAL
	moveWindow("Status", 0, 600);

	ifstream infile("spint.ini");
	string tempstring;

	int fd, res;	
    char buf[255];

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
	unsigned int ROIstartrow, ROIendrow, ROIstartcol, ROIendcol, heightROI, widthROI;
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
		Mat J, S, bscan, jscan;
		J = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);			// Size(cols,rows)
		S = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);
		Sk = Mat::ones(Size(opw,oph),CV_16UC1);
		Mat jdiff, positivediff, bscanlog, bscandb, tempmat, bscandisp, cmagI;
		Mat ROI;	
		// ROI parameters
		Point ROItopleft(ROIstartcol,ROIstartrow), ROIbottright(ROIendcol,ROIendrow);
		enum corner{topleft=1,bottomright};
		int selectedpoint;
		selectedpoint = topleft;

		resizeWindow("Bscan", oph, numdisplaypoints);		// (width,height)
		//resizeWindow("Interferogram", opw, oph);		// (width,height)
		int ret;
		unsigned int ii; // index

		while (1)	//camera frames acquisition loop, which is inside the try
		{
			if(bgframestaken == false)
			{
				J = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);
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

						// accumulate the set of background bscans to J
						accumulate(bscantemp, J);

					} // end of if ret == 1 block 
				} // end of for loop ii <= averagescount

				bgframestaken = true;				
			} // end of if(bgframestaken == false)

			// now read a set of images from the camera,  
			// compute bscan and accumulate to Mat S

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
				//cout << "Read  signal image " << ii << " from the camera" << endl;	

				imshow("Interferogram", opm);

				if(ret == 1)
				{
					oct.readInterferogram(data_y);
					oct.dividebySpectrum(Sk);
					bscantemp = oct.computeBscan();	

					// accumulate the Signal bscans to S
					accumulate(bscantemp, S);
					fps++;
				} // end of if ret == 1 block 

			} // end of for loop ii <= averagescount

			//transpose(J, jscan);
			//transpose(S, bscan);
			J.copyTo(jscan);
			S.copyTo(bscan);
			
// absolute diff --- uncomment the next two lines for absolute diff 	
			//absdiff(bscan,jscan,jdiff);
			//positivediff = jdiff / (1.0 * averagescount);	

// only positive --- comment the next four lines if negative values should NOT be set to zero
			jdiff = bscan - jscan;
			jdiff.copyTo(positivediff);		// just to initialize the Mat
			makeonlypositive(jdiff, positivediff);
			positivediff = positivediff / (1.0 * averagescount);	

			positivediff += 0.000000001;			// to avoid log(0)
			
			// uncomment the below two lines for log scale
			log(positivediff, bscanlog);				// switch to logarithmic scale
			bscandb = 20.0 * bscanlog / 2.303;
			
			// uncomment the below line for linear scale
			//positivediff.copyTo(bscandb);

			bscandb.row(4).copyTo(bscandb.row(1));	// masking out the DC in the display
			bscandb.row(4).copyTo(bscandb.row(0));
			tempmat = bscandb.colRange(0, numdisplaypoints);
			tempmat.copyTo(bscandisp);
			bscandisp = max(bscandisp, bscanthreshold);
			normalize(bscandisp, bscandisp, 0, 1, NORM_MINMAX);	// normalize the log plot for display
			bscandisp.convertTo(bscandisp, CV_8UC1, 255.0);
			applyColorMap(bscandisp, cmagI, COLORMAP_JET);

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
			if (ROIflag == true)	
				rectangle(cmagI,ROItopleft,ROIbottright,Scalar(0,255,0),1, LINE_8);
			imshow("Bscan", cmagI);
			S = Mat::zeros(Size(numdisplaypoints, oph), CV_64F);

			gettimeofday(&tv,NULL);	
			time_end = 1000000 * tv.tv_sec + tv.tv_usec;	
			// update the image windows
			dt = time_end - time_start;

			if(dt > 1000000) // 1 second in microseconds 
			{
				m.copyTo(mvector);
				mvector.reshape(0, 1);	//make it into a row array
				minMaxLoc(mvector, &minVal, &maxVal);
				sprintf(textbuffer, "fps = %d  Max val = %d", int(round(fps/dt*1e6)), int(floor(maxVal)));
				sprintf(textbuffer, "dt = %d  Max I = %d", dt, int(floor(maxVal)));
				firstrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);
				if (averagescount == 1)	
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
				tempmat(Rect(ROIstartcol,ROIstartrow,widthROI,heightROI)).copyTo(ROI);
				ROI.reshape(0, 1);	//make it into a row array
				minMaxLoc(ROI, &minVal, &maxVal);
				meanVal = mean(ROI)(0);
				//sprintf(textbuffer, "Max Bscan val = %d dB", int(round(maxVal)));
				if (selectedpoint == topleft)
					sprintf(textbuffer, "ROI T(%d,%d) max=%d mean=%d", ROIstartcol, ROIstartrow, int(round(maxVal)), int(round(meanVal)));
				if (selectedpoint == bottomright)
					sprintf(textbuffer, "ROI R(%d,%d) max=%d mean=%d", ROIendcol, ROIendrow, int(round(maxVal)), int(round(meanVal)));
				fourthrowofstatusimg = Mat::zeros(cv::Size(600, 50), CV_64F);
				putText(statusimg, textbuffer, Point(0,190), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 3, 1);

				}
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
				if (averagescount == 1)
					averagescount = 5; 
				else 
					averagescount += 5;
				bgframestaken = false;
				break;

			case '3':
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
			
				// keys to change ROI

			case 't':
				if (ROIflag == false)
					break;
				// select the topleft corner point
				selectedpoint = 1; //topleft
				break; 			 

			case 'r':
				if (ROIflag == false)
					break;
				// select the rightbottom corner point
				selectedpoint = 2; //rightbottom
				break; 
			 
			case 82: 							// up arrow key = R ?
				if (ROIflag == false)
					break;
				if(selectedpoint == topleft)
				{
					// move up the topleft corner of ROI by 5 pixels
					if(ROIstartrow > 5)
						ROIstartrow -= 5;
					ROItopleft.y = ROIstartrow;
				}
				if(selectedpoint == bottomright)
				{
					// move up the bottomright corner of ROI by 5 pixels
					if(ROIendrow > 5)
						ROIendrow -= 5;
					if(ROIendrow-ROIstartrow <= 0)
						ROIendrow += 5;
					ROIbottright.y = ROIendrow;
				}
				break;

			case 84: 						// down arrow key = T ?
				if (ROIflag == false)
					break;
				if(selectedpoint == topleft)
				{
					// move down the topleft corner of ROI by 5 pixels
					if(ROIstartrow < (oph-5) )
						ROIstartrow += 5;
					if(ROIendrow-ROIstartrow <= 0)
						ROIstartrow -= 5;
					ROItopleft.y = ROIstartrow;
				}
				if(selectedpoint == bottomright)
				{
					// move down the bottomright corner of ROI by 5 pixels
					if(ROIendrow < (oph-5) )
						ROIendrow += 5;
					ROIbottright.y = ROIendrow;

				}
				break;

			case 81:						// left arrow key = Q ?
				if (ROIflag == false)
					break;
				if(selectedpoint == topleft)
				{ 
					// move left the topleft corner of the ROI by 5 pixels
					if(ROIstartcol > 5)
						ROIstartcol -= 5;
					ROItopleft.x = ROIstartcol;
				}
				if(selectedpoint == bottomright)
				{
					// move left the bottomright corner of the ROI by 5 pixels
					if(ROIendcol > 5)
						ROIendcol -= 5;
					if(ROIendcol-ROIstartcol <= 0)
						ROIendcol += 5;
					ROIbottright.x = ROIendcol;
				}
				break;

			case 83:						// right arrow key = S ?
				if (ROIflag == false)
					break;
				if(selectedpoint == topleft)
				{ 
					// move right the topleft corner of the ROI by 5 pixels
					if(ROIstartcol < (numdisplaypoints-5))
						ROIstartcol += 5;
					if(ROIendcol-ROIstartcol <= 0)
						ROIstartcol -= 5;
					ROItopleft.x = ROIstartcol;
				}
				if(selectedpoint == bottomright)
				{
					// move right the bottomright corner of the ROI by 5 pixels
					if(ROIendcol < (numdisplaypoints-5))
						ROIendcol += 5;
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
