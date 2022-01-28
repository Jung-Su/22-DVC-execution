#include <BotOp/bot.h>


void cvConvertShow(const byteA rgb, const std::string& name){
  cv::Mat bgr;
  cv::cvtColor(CV(rgb), bgr, cv::COLOR_RGB2BGR);
  cv::imshow(name, bgr);
}
//===========================================================================
void moveToPoses_cali(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("world.g");

  arr orig = {0.0215, .01, 0.0415};
  for(uint i=0; i<CHECKERBOARD[0]; i++){
    for(uint j=0; j<CHECKERBOARD[1]; j++){
      arr relPos = arr{(CHECKERBOARD[0]-1-i)*0.025, 0, j*0.025} + orig;
      rai::Frame& f = C.addFrame(STRING("CBP"<<i<<"_"<<j), "ot_cameraCalibrationPlate")->setRelativePosition(relPos).setShape(rai::ST_sphere, {0.01});
      if(i==0 && j==0) f.setColor({.9, .6, .6});
    }
  }

  arr points=rai::getParameter<arr>("points");
  points.reshape(-1,3);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  //-- prepare image files
  RealSenseThread RS({}, {});

  std::vector<cv::Point3f> objpoints;
  std::vector<cv::Point2f> imgpoints;

  cv::Mat rgb, gray, bgr;
  bool success;


  //-- loop through poses
  arr q_last=bot.get_q();
  uint L = points.d0;
  for(uint l=0;l<=L;l++){
    arr q_target;

    //compute pose
    if(l<L){
      KOMO komo;
      komo.setModel(C, true);
      komo.setTiming(1, 1, 3., 1);
      komo.add_qControlObjective({}, 1, 1e-1);
      komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, bot.qHome);
      komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

      arr pt = points[l];
      komo.addObjective({}, FS_positionDiff, {"r_gripper", "table_base"}, OT_sos, {1e2}, pt);
      komo.addObjective({}, FS_positionRel, {"viewCenter", "r_gripper"}, OT_sos, {{2,3},{1e1,0,0,0,1e1,0}});

      komo.optimize();

      //is feasible?
      bool feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;

      if(!feasible){
        cout <<" === pose infeasible ===\n" <<points[l] <<endl;
        continue;
      }

      q_target = komo.x;
      cout <<" === pose feasible  === " <<endl;
    }else{
      q_target = bot.qHome;
    }

    //C.setJointState(q_target);  C.watch(true, "pose");

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
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    rai::Transformation TgripperInv;
    TgripperInv.setInverse(C["r_robotiq_optitrackMarker"]->get_X());

    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    std::vector<cv::Point2f> corner_pts;
    success = cv::findChessboardCorners(gray,
                                        cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
                                        corner_pts,
                                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);


    cout << C["ot_cameraCalibrationPlate"]->getPosition() << endl;
    if(success){
      cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS, 30, 0.001);
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1), criteria);
      cv::drawChessboardCorners(bgr, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

      cv::imshow("image", bgr);
      int key = cv::waitKey(0);
      cv::destroyWindow("image");
      arr pos;
      rai::String dir = (key&0xff)=='f'?"forward":"backward";
      cout << dir  << endl;
      for(int j{0}; j<CHECKERBOARD[1]; j++){
        for(int i{0}; i<CHECKERBOARD[0]; i++){
          rai::String fname = (key&0xff)=='f'?STRING("CBP"<<i<<"_"<<j):STRING("CBP"<<CHECKERBOARD[0]-1-i<<"_"<<CHECKERBOARD[1]-1-j);
          pos = C[fname]->getPosition();
          TgripperInv.applyOnPoint(pos);
          objpoints.push_back(cv::Point3f(pos(0),pos(1),pos(2)));
        }
      }

      cout<<"corner"<<endl;
      for(auto& pts: corner_pts){
        imgpoints.push_back(pts);
      }
    }

    rai::wait(.2);
  }
  cv::destroyAllWindows();
  cv::Mat cameraMatrix(3,3, CV_64F),distCoeffs,R,T;

  arr Fxypxy = RS.fxypxy;
  cameraMatrix.at<double>(0,0)=Fxypxy.elem(0);
  cameraMatrix.at<double>(0,1)=0.;
  cameraMatrix.at<double>(0,2)=Fxypxy.elem(2);
  cameraMatrix.at<double>(1,0)=0.;
  cameraMatrix.at<double>(1,1)=Fxypxy.elem(1);
  cameraMatrix.at<double>(1,2)=Fxypxy.elem(3);
  cameraMatrix.at<double>(2,0)=0.;
  cameraMatrix.at<double>(2,1)=0.;
  cameraMatrix.at<double>(2,2)=1.;
  cout << "cameraMatrixInit : " << cameraMatrix << endl;

  double ret = cv::calibrateCamera(std::vector<std::vector<cv::Point3f>>{objpoints},
                      std::vector<std::vector<cv::Point2f>>{imgpoints},
                      cv::Size(gray.rows,gray.cols),
                      cameraMatrix,
                      distCoeffs, R, T,
                      cv::CALIB_USE_INTRINSIC_GUESS);
  cout << "cameraMatrix : " << cameraMatrix << endl;
  cout << "distCoeffs : " << distCoeffs << endl;
  cout << "Rotation vector : " << R << endl;
  cout << "Translation vector : " << T << endl;

  cout << "error : " << ret << endl;

  cv::Mat Rmat;
  cv::Rodrigues(R, Rmat);

  arr P(4,4);
  P.setZero();
  P(0,0) = Rmat.at<double>(0,0);
  P(0,1) = Rmat.at<double>(0,1);
  P(0,2) = Rmat.at<double>(0,2);
  P(1,0) = Rmat.at<double>(1,0);
  P(1,1) = Rmat.at<double>(1,1);
  P(1,2) = Rmat.at<double>(1,2);
  P(2,0) = Rmat.at<double>(2,0);
  P(2,1) = Rmat.at<double>(2,1);
  P(2,2) = Rmat.at<double>(2,2);
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

  cout << cameraFrame->getRelativePosition() << endl;
  C.watch(true);

  rai::CameraView V(C);
  V.addSensor("rs_camera", "rs_camera", 1920, 1080, cameraMatrix.at<double>(0,0)/1080);

  byteA _image;
  floatA dummy;
  V.computeImageAndDepth(_image, dummy);
  cv::imshow("realsense", bgr);
  cvConvertShow(_image, "rai");
  cv::waitKey(0);

  ofstream fil("z.calib.dat");
  fil <<" qCam " << camPose.getArr7d() <<" intrinsic "<< cameraMatrix <<endl; // <<" poseTable " <<optiTable->ensure_X() <<endl;
  fil.close();

  rai::wait();
}

  //===========================================================================



void addCBPoints(rai::Configuration& C){
  arr orig = {0.0215, .01, 0.0415};
  for(uint i=0; i<CHECKERBOARD[0]; i++){
    for(uint j=0; j<CHECKERBOARD[1]; j++){
      arr relPos = arr{i*0.025, 0, j*0.025} + orig;
      C.addFrame(STRING("CBP"<<i<<"_"<<j), "ot_cameraCalibrationPlate")->setRelativePosition(relPos).setShape(rai::ST_sphere, {0.01});
    }
  }
}

void visualizeChessBoard(){
  rai::Configuration C("world.g");
  addCBPoints(C);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.hold(true, false);
  while(bot.step(C));
}

void testRealSense(){
  Var<byteA> rgb;
  RealSenseThread RS(rgb, {});
  while(true){
    RS.color.waitForNextRevision();
    cvConvertShow(rgb.get(), "rgb");
    int key = cv::waitKey(1);
    if((key&0xff)=='q') break;
  }
}

void calibrateCamera(uint numImages, bool onGripper=false){

  rai::Configuration C;
  C.addFile("world.g");

  arr orig = {0.0215, .01, 0.0415};
  for(uint i=0; i<CHECKERBOARD[0]; i++){
    for(uint j=0; j<CHECKERBOARD[1]; j++){
      arr relPos = arr{(CHECKERBOARD[0]-1-i)*0.025, 0, j*0.025} + orig;
      rai::Frame& f = C.addFrame(STRING("CBP"<<i<<"_"<<j), "ot_cameraCalibrationPlate")->setRelativePosition(relPos).setShape(rai::ST_sphere, {0.01});
      if(i==0 && j==0) f.setColor({.9, .6, .6});
    }
  }

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.hold(true, false);


  // launch camera
  RealSenseThread RS({}, {});

  std::vector<cv::Point3f> objpoints;
  std::vector<cv::Point2f> imgpoints;

  cv::Mat rgb, gray, bgr;
  bool success;

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
    rai::Transformation TgripperInv;
    if(onGripper) TgripperInv.setInverse(C["r_robotiq_optitrackMarker"]->get_X());

    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    std::vector<cv::Point2f> corner_pts;
    success = cv::findChessboardCorners(gray,
                                        cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
                                        corner_pts,
                                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    cout << C["ot_cameraCalibrationPlate"]->getPosition() << endl;
    if(success){
      cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS, 30, 0.001);
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1), criteria);
      cv::drawChessboardCorners(bgr, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

      cv::imshow("image", bgr);
      int key = cv::waitKey(0);
      cv::destroyWindow("image");
      arr pos;
      rai::String dir = (key&0xff)=='f'?"forward":"backward";
      cout << dir  << endl;
      for(int j{0}; j<CHECKERBOARD[1]; j++){
        for(int i{0}; i<CHECKERBOARD[0]; i++){
          rai::String fname = (key&0xff)=='f'?STRING("CBP"<<i<<"_"<<j):STRING("CBP"<<CHECKERBOARD[0]-1-i<<"_"<<CHECKERBOARD[1]-1-j);
          pos = C[fname]->getPosition();
          if(onGripper) TgripperInv.applyOnPoint(pos);
          objpoints.push_back(cv::Point3f(pos(0),pos(1),pos(2)));
        }
      }

      cout<<"corner"<<endl;
      for(auto& pts: corner_pts){
        imgpoints.push_back(pts);
      }

    }

  }

  cv::destroyAllWindows();
  cv::Mat cameraMatrix(3,3, CV_64F),distCoeffs,R,T;

  arr Fxypxy = RS.fxypxy;
  cameraMatrix.at<double>(0,0)=Fxypxy.elem(0);
  cameraMatrix.at<double>(0,1)=0.;
  cameraMatrix.at<double>(0,2)=Fxypxy.elem(2);
  cameraMatrix.at<double>(1,0)=0.;
  cameraMatrix.at<double>(1,1)=Fxypxy.elem(1);
  cameraMatrix.at<double>(1,2)=Fxypxy.elem(3);
  cameraMatrix.at<double>(2,0)=0.;
  cameraMatrix.at<double>(2,1)=0.;
  cameraMatrix.at<double>(2,2)=1.;
  cout << "cameraMatrixInit : " << cameraMatrix << endl;

  double ret = cv::calibrateCamera(std::vector<std::vector<cv::Point3f>>{objpoints},
                      std::vector<std::vector<cv::Point2f>>{imgpoints},
                      cv::Size(gray.rows,gray.cols),
                      cameraMatrix,
                      distCoeffs, R, T,
                      cv::CALIB_USE_INTRINSIC_GUESS);
  cout << "cameraMatrix : " << cameraMatrix << endl;
  cout << "distCoeffs : " << distCoeffs << endl;
  cout << "Rotation vector : " << R << endl;
  cout << "Translation vector : " << T << endl;

  cout << "error : " << ret << endl;

  cv::Mat Rmat;
  cv::Rodrigues(R, Rmat);

  arr P(4,4);
  P.setZero();
  P(0,0) = Rmat.at<double>(0,0);
  P(0,1) = Rmat.at<double>(0,1);
  P(0,2) = Rmat.at<double>(0,2);
  P(1,0) = Rmat.at<double>(1,0);
  P(1,1) = Rmat.at<double>(1,1);
  P(1,2) = Rmat.at<double>(1,2);
  P(2,0) = Rmat.at<double>(2,0);
  P(2,1) = Rmat.at<double>(2,1);
  P(2,2) = Rmat.at<double>(2,2);
  P(0,3) = T.at<double>(0);
  P(1,3) = T.at<double>(1);
  P(2,3) = T.at<double>(2);
  P(3,3) = 1.;

  rai::Transformation camPose;
  camPose.setAffineMatrix(inverse(P).p);

  rai::Frame* cameraFrame;
  if(onGripper){
    cameraFrame = C.addFrame("rs_camera", "r_robotiq_optitrackMarker");
    cameraFrame->setRelativePosition(camPose.pos.getArr()).setRelativeQuaternion(camPose.rot.getArr4d()).setShape(rai::ST_marker, {.2});
  }else{
    cameraFrame = C.addFrame("rs_camera");
    cameraFrame->setPose(camPose).setShape(rai::ST_marker, {.2});
  }

  cameraFrame->addRad(RAI_PI, 1, 0, 0); //opencv => gl camera!

  cout << cameraFrame->getRelativePosition() << endl;
  C.watch(true);

  rai::CameraView V(C);
  V.addSensor("rs_camera", "rs_camera", 1920, 1080, cameraMatrix.at<double>(0,0)/1080);

  byteA _image;
  floatA dummy;
  V.computeImageAndDepth(_image, dummy);
  cv::imshow("realsense", bgr);
  cvConvertShow(_image, "rai");
  cv::waitKey(0);

  ofstream fil("z.calib.dat");
  fil <<" qCam " << camPose.getArr7d() <<" intrinsic "<< cameraMatrix <<endl; // <<" poseTable " <<optiTable->ensure_X() <<endl;
  fil.close();
}
