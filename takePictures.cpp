#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <BotOp/bot.h>
//#include "cameraCalibration.cpp"

//===========================================================================

void moveToPoses(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("myWorld.g");

  arr points=rai::getParameter<arr>("points");
  points.reshape(-1,3);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

//  //-- load cam calibration
//  ifstream fil("z.calib.dat");
//  arr camPose, K;
//  for(uint t=0;;t++){
//    rai::skip(fil);
//    if(!fil.good()) break;
//    fil >>PARSE("qCam") >>  camPose >>PARSE("intrinsic") >>K;
//    }
//  fil.close();

  cv::Mat K, R, T, distCoeffs;

  cv::FileStorage fs("calib_result.json", cv::FileStorage::READ);
  fs["K"] >> K;
  fs["R"] >> R;
  fs["T"] >> T;
  fs["distCoeffs"] >> distCoeffs;


  arr P(4,4);
  P.setZero();
  P(0,0) = R.at<double>(0,0);
  P(0,1) = R.at<double>(0,1);
  P(0,2) = R.at<double>(0,2);
  P(1,0) = R.at<double>(1,0);
  P(1,1) = R.at<double>(1,1);
  P(1,2) = R.at<double>(1,2);
  P(2,0) = R.at<double>(2,0);
  P(2,1) = R.at<double>(2,1);
  P(2,2) = R.at<double>(2,2);
  P(0,3) = T.at<double>(0);
  P(1,3) = T.at<double>(1);
  P(2,3) = T.at<double>(2);
  P(3,3) = 1.;


  rai::Transformation camPose;
  camPose.setAffineMatrix(inverse(P).p);

  rai::Frame* cameraFrame;
  cameraFrame = C.addFrame("rs_camera", "r_robotiq_optitrackMarker");
  cameraFrame->setRelativePosition(camPose.pos.getArr()).setRelativeQuaternion(camPose.rot.getArr4d()).setShape(rai::ST_marker, {.2});
  cameraFrame->addRad(RAI_PI, 1, 0, 0); //opencv => gl camera!

  //-- prepare image files
  RealSenseThread RS({}, {});
  cv::Mat rgb, rgb_undistorted, bgr;
  ofstream fil2("/home/jung-su/GoogleDrive/DVC_experiment/cameraData.json");
  fil2 <<" { \"intrinsic\": " << K << "," << endl;
  fil2 <<" \"pose\": ["  <<endl;

  //-- loop through poses
  arr q_last=bot.get_q();
  for(uint l=0;l<points.d0;l++){

    //compute pose
    arr q_target = getShootingPose(C, points[l], bot.qHome);
    if(!q_target.N){
      cout <<" === pose infeasible === " <<endl;
      continue;
    }

    //compute path
    C.setJointState(q_last);
    arr path = getStartGoalPath(C, q_target, bot.qHome);
    if(!path.N){
      cout <<" === path infeasible === " <<endl;
      continue;
    }
    q_last = q_target;
    cout <<" === path feasible  === " <<endl;

    cout <<" === -> executing === " <<endl;
    bot.moveAutoTimed(path, 1., 1.);

    while(bot.step(C)){
        rgb = CV(RS.color.get()).clone();
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        cv::imshow("rgb", bgr);
        cv::waitKey(1);
      }
    if(bot.keypressed=='q' || bot.keypressed==27) break;

    rai::wait(.2);

    //-- take image
    rgb = CV(RS.color.get()).clone();
//    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    cv::undistort(rgb, rgb_undistorted, K, distCoeffs);
    cv::cvtColor(rgb_undistorted, bgr, cv::COLOR_RGB2BGR);
    cv::imwrite(STRING("/home/jung-su/GoogleDrive/DVC_experiment/img_"<<l<<".png").p, bgr);
    if(l>0) fil2 << "," <<endl;
    fil2 << C["rs_camera"]->getPose() <<endl;

    rai::wait(.2);
  }
  fil2 <<  "]}" <<endl;
  fil2.close();

  bot.home(C);
  rai::wait();
}

//===========================================================================

void takePictures(uint numImages, bool onGripper=false){

  rai::Configuration C;
  C.addFile("../botop/rai-robotModels/scenarios/pandasTable-calibrated.g");

  ifstream fil("z.calib.dat");
  arr camPose, K;
  for(uint t=0;;t++){
    rai::skip(fil);
    if(!fil.good()) break;
    fil >>PARSE("qCam") >>  camPose >>PARSE("intrinsic") >>K;
    }
  fil.close();


  rai::Frame* cameraFrame;
  if(onGripper){
    cameraFrame = C.addFrame("rs_camera", "r_robotiq_optitrackMarker");
    cameraFrame->setRelativePosition(camPose.sub(0,2)).setRelativeQuaternion(camPose.sub(3,6)).setShape(rai::ST_marker, {.2});
  }else{
    cameraFrame = C.addFrame("rs_camera");
    cameraFrame->setPosition(camPose.sub(0,2)).setQuaternion(camPose.sub(3,6)).setShape(rai::ST_marker, {.2});
  }
  cameraFrame->addRad(RAI_PI, 1, 0, 0); //opencv => gl camera!

  C.watch(true);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  bot.hold(true, false);


  // launch camera
  RealSenseThread RS({}, {});
  cv::Mat rgb, bgr;


  ofstream fil2("/home/jung-su/GoogleDrive/DVC_experiment/camData.json");
  fil2 <<" { \"intrinsic\": " << K << "," << endl;
  fil2 <<" \"pose\": ["  <<endl;

  for(uint i=0;i<numImages;i++){
    while(true){
      bot.step(C);
      RS.color.waitForNextRevision();
      rgb = CV(RS.color.get()).clone();
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

      cv::imshow("rgb", bgr);
      int key = cv::waitKey(1);
      if((key&0xff)=='q') break;
    }
    cv::imwrite(STRING("/home/jung-su/GoogleDrive/DVC_experiment/img_"<<i<<".png").p, bgr);
    if(i > 0) fil2 << "," <<endl;
    fil2 << C["rs_camera"]->getPose() <<endl;
  }
  fil2 <<  "]}" <<endl;
  fil2.close();
}
