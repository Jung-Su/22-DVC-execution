#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Core/array.h>
#include <Geo/geo.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/cameraview.h>
#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

#include <RealSense/RealSenseThread.h>
//#include <RealSense/RealSenseCalibration.h>

int CHECKERBOARD[2]{7,9};

#include "cameraCalibration.cpp"
#include "takePictures.cpp"
#include "runSolution.cpp"

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);
  //  calibrateCamera(3, true);
    //  takePictures(5, true);

//  testRealSense();
//  visualizeChessBoard();

//  moveToPoses_cali();
  checkCalibrationResult();

//  moveToPoses();

//  runSolution(5.);
//  runSolutionHO();

//  LOG(0) <<" === bye bye ===\n used parameters:\n" << rai::getParameters()() <<'\n';

  return 0;
}
