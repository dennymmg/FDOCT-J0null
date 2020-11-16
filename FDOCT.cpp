#include "FDOCT.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp>

using namespace cv;

// member functions of FDOCT class

void FDOCT::setWidthHeight(unsigned int w, unsigned int h)
{
	width = w;
	height = h;
}

void FDOCT::setLambdaMinMax(double lmin, double lmax)
{
	lambdamin = lmin;
	lambdamax = lmax;
}

void FDOCT::setfftpointsmultiplier(unsigned int mul)
{
	fftpointsmultiplier = mul;
}

void FDOCT::setnumdisplaypoints(unsigned int num)
{
	numdisplaypoints = num;
}

void FDOCT::initialCompute()
{	
	const double pi = 3.141592653589793;

	data_y = Mat(height, width, CV_64F);	// the Mat constructor Mat(rows,columns,type);
	barthannwin	= Mat(1, width, CV_64F);
	
	numfftpoints = fftpointsmultiplier * width;  

	double deltalambda = (lambdamax - lambdamin) / data_y.cols;
	
	// create modified Bartlett-Hann window
	for (unsigned int p = 0; p<(width); p++)
	{
		// https://in.mathworks.com/help/signal/ref/barthannwin.html
		float nn = p;
		float NN = width - 1;
		barthannwin.at<double>(0, p) = 0.62 - 0.48*abs(nn / NN - 0.5) + 0.38*cos(2 * pi*(nn / NN - 0.5));
	}

	Mat klinear = Mat::zeros(cv::Size(1, numfftpoints), CV_64F);
	fractionalk = Mat::zeros(cv::Size(1, numfftpoints), CV_64F);
	nearestkindex = Mat::zeros(cv::Size(1, numfftpoints), CV_32S);
	
	Mat lambdas = Mat::zeros(cv::Size(1, numfftpoints), CV_64F);		//Size(cols,rows)
	Mat diffk = Mat::zeros(cv::Size(1, numfftpoints), CV_64F);

	unsigned int indextemp;

	// compute lambdas
	for (indextemp = 0; indextemp< (data_y.cols); indextemp++)
	{
		lambdas.at<double>(0, indextemp) = lambdamin + indextemp * deltalambda / fftpointsmultiplier;
	}

	Mat k = 2 * pi / lambdas;
	double kmin = 2 * pi / (lambdamax - deltalambda);
	double kmax = 2 * pi / lambdamin;
	double deltak = (kmax - kmin) / numfftpoints;

	// compute klinear
	for (indextemp = 0; indextemp < numfftpoints; indextemp++)
	{
		klinear.at<double>(0, indextemp) = kmin + (indextemp + 1)*deltak;
	}

	// find the diff of the non-linear ks
	for (indextemp = 1; indextemp < numfftpoints; indextemp++)
	{
		// since this is a decreasing series, RHS is (i-1) - (i)
		diffk.at<double>(0, indextemp) = k.at<double>(0, indextemp - 1) - k.at<double>(0, indextemp);
	}
	// and initializing the first point separately
	diffk.at<double>(0, 0) = diffk.at<double>(0, 1);

	// find the index of the nearest k value, less than the linear k
	for (int f = 0; f < numfftpoints; f++)
	{
		for (indextemp = 0; indextemp < numfftpoints; indextemp++)
		{
			if (k.at<double>(0, indextemp) < klinear.at<double>(0, f))
			{
				nearestkindex.at<int>(0, f) = indextemp;
				break;
			}// end if
		}//end indextemp loop
	}// end f loop

	// now find the fractional amount by which the linearized k value is greater than the next lowest k
	for (int f = 0; f < numfftpoints; f++)
	{
		fractionalk.at<double>(0, f) = (klinear.at<double>(0, f) - k.at<double>(0, nearestkindex.at<int>(0, f))) / diffk.at<double>(0, nearestkindex.at<int>(0, f));
	}

}

void FDOCT::readInterferogram(Mat m)
{
	Mat m_64F;
	m.convertTo(m_64F, CV_64F);
	data_y = m_64F; 
}

void FDOCT::dividebySpectrum(Mat spectrum)
{
	Mat spectrum_64F;
	spectrum.convertTo(spectrum_64F, CV_64F);
	data_y = data_y / spectrum_64F; 
}

Mat FDOCT::computeBscan()
{
	// DC removal and windowing
	for (int p = 0; p<(data_y.rows); p++)
	{
		Scalar meanval = mean(data_y.row(p));
		data_y.row(p) = data_y.row(p) - meanval(0);	// Only the first value of the scalar is useful for us
		
		//multiply(data_yb.row(p), 1.0/sqrt((meanval(0))), data_yb.row(p));
		multiply(data_y.row(p), barthannwin, data_y.row(p));
	}

	//increasing number of points by zero padding
	if (fftpointsmultiplier > 1)
		data_y = zeropadrowwise(data_y, fftpointsmultiplier);

	Mat data_ylin(height, numfftpoints, CV_64F);
	Mat slopes = Mat::zeros(cv::Size(data_y.rows, numfftpoints), CV_64F);

	// interpolate to linear k space
	for (int p = 0; p<(data_y.rows); p++)
	{
		for (int q = 1; q<(data_y.cols); q++)
		{
			//find the slope of the data_y at each of the non-linear ks
			slopes.at<double>(p, q) = data_y.at<double>(p, q) - data_y.at<double>(p, q - 1);
			// in the .at notation, it is <double>(y,x)
		}
		// initialize the first slope separately
		slopes.at<double>(p, 0) = slopes.at<double>(p, 1);

		for (int q = 1; q<(data_ylin.cols - 1); q++)
		{
			//find the value of the data_ylin at each of the klinear points
			// data_ylin = data_y(nearestkindex) + fractionalk(nearestkindex)*slopes(nearestkindex)
			data_ylin.at<double>(p, q) = data_y.at<double>(p, nearestkindex.at<int>(0, q))
				+ fractionalk.at<double>(nearestkindex.at<int>(0, q))
				* slopes.at<double>(p, nearestkindex.at<int>(0, q));
		}
	}


	// InvFFT - compute the bscan
	Mat planes[] = { Mat_<float>(data_ylin), Mat::zeros(data_ylin.size(), CV_32F) };
	Mat complexI;
	merge(planes, 2, complexI);       // Add to the expanded another plane with zeros
	dft(complexI, complexI, DFT_ROWS | DFT_INVERSE);
	split(complexI, planes);          // planes[0] = Re(DFT(I)), planes[1] = Im(DFT(I))
	Mat magI;
	magnitude(planes[0], planes[1], magI);
	Mat bscan_32f, bscan_64f;
	bscan_32f = magI.colRange(0, numdisplaypoints);
	bscan_32f.convertTo(bscan_64f, CV_64F);
	bscan = bscan_64f;
	return bscan;
}

Mat FDOCT::zeropadrowwise(Mat sm, int sn)
{
	// increase fft points sn times 
	// newnumcols = numcols*sn;
	// by fft, zero padding, and then inv fft

	// returns CV_64F

	// guided by https://stackoverflow.com/questions/10269456/inverse-fourier-transformation-in-opencv
	// inspired by Drexler & Fujimoto 2nd ed Section 5.1.10

	// needs fftshift implementation for the zero pad to work correctly if done on borders.
	// or else adding zeros directly to the higher frequencies. 

	// freqcomplex=fftshift(fft(signal));
	// zp2=4*ifft(ifftshift(zpfreqcomplex));

	// result of this way of zero padding in the fourier domain is to resample the same min / max range
	// at a higher sampling rate in the initial domain.
	// So this improves the k linear interpolation.

	Mat origimage;
	Mat fouriertransform, fouriertransformzp;
	Mat inversefouriertransform;

	int numrows = sm.rows;
	int numcols = sm.cols;
	int newnumcols = numcols * sn;

	sm.convertTo(origimage, CV_32F);

	dft(origimage, fouriertransform, DFT_SCALE | DFT_COMPLEX_OUTPUT | DFT_ROWS);

	// implementing fftshift row-wise
	// like https://docs.opencv.org/2.4/doc/tutorials/core/discrete_fourier_transform/discrete_fourier_transform.html
	int cx = fouriertransform.cols / 2;

	// here we assume fouriertransform.cols is even

	Mat LHS(fouriertransform, Rect(0, 0, cx, fouriertransform.rows));   // Create a ROI per half
	Mat RHS(fouriertransform, Rect(cx, 0, cx, fouriertransform.rows)); //  Rect(topleftx, toplefty, w, h), 
																	   // OpenCV typically assumes that the top and left boundary of the rectangle are inclusive, while the right and bottom boundaries are not. 
																	   // https://docs.opencv.org/3.2.0/d2/d44/classcv_1_1Rect__.html

	Mat tmp;                           // swap LHS & RHS
	LHS.copyTo(tmp);
	RHS.copyTo(LHS);
	tmp.copyTo(RHS);

	copyMakeBorder(fouriertransform, fouriertransformzp, 0, 0, floor((newnumcols - numcols) / 2), floor((newnumcols - numcols) / 2), BORDER_CONSTANT, 0.0);
	// this does the zero pad - copyMakeBorder(src, dest, top, bottom, left, right, borderType, value)

	// Now we do the ifftshift before ifft
	cx = fouriertransformzp.cols / 2;
	Mat LHSzp(fouriertransformzp, Rect(0, 0, cx, fouriertransformzp.rows));   // Create a ROI per half
	Mat RHSzp(fouriertransformzp, Rect(cx, 0, cx, fouriertransformzp.rows)); //  Rect(topleftx, toplefty, w, h)

	LHSzp.copyTo(tmp);
	RHSzp.copyTo(LHSzp);
	tmp.copyTo(RHSzp);

	dft(fouriertransformzp, inversefouriertransform, DFT_INVERSE | DFT_REAL_OUTPUT | DFT_ROWS);
	inversefouriertransform.convertTo(inversefouriertransform, CV_64F);

	return inversefouriertransform;
}
