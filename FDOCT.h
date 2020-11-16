#ifndef FDOCT_H
#define FDOCT_H
#include <opencv2/opencv.hpp>

class FDOCT {

public:

void setWidthHeight(unsigned int w, unsigned int h);
void setLambdaMinMax(double lmin, double lmax);
void setfftpointsmultiplier(unsigned int mul);
void setnumdisplaypoints(unsigned int num);
void initialCompute();
void readInterferogram(cv::Mat m);
void dividebySpectrum(cv::Mat spectrum);
cv::Mat computeBscan();

private:

unsigned int width;    // of the interferogram
unsigned int height;
double lambdamin;
double lambdamax;
cv::Mat data_y;
cv::Mat bscan;
unsigned int fftpointsmultiplier;
unsigned int numdisplaypoints;

unsigned int numfftpoints;
cv::Mat barthannwin;
cv::Mat fractionalk;
cv::Mat nearestkindex;
 
cv::Mat zeropadrowwise(cv::Mat sm, int sn);

};

#endif
