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

  //-- load cam calibration
  ifstream fil("z.calib.dat");
  arr camPose, K;
  for(uint t=0;;t++){
    rai::skip(fil);
    if(!fil.good()) break;
    fil >>PARSE("qCam") >>  camPose >>PARSE("intrinsic") >>K;
    }
  fil.close();

  rai::Frame* cameraFrame;
  cameraFrame = C.addFrame("rs_camera", "r_robotiq_optitrackMarker");
  cameraFrame->setRelativePosition(camPose.sub(0,2)).setRelativeQuaternion(camPose.sub(3,6)).setShape(rai::ST_marker, {.2});
  cameraFrame->addRad(RAI_PI, 1, 0, 0); //opencv => gl camera!

  //-- prepare image files
  RealSenseThread RS({}, {});
  cv::Mat rgb, bgr;
  ofstream fil2("/home/jung-su/GoogleDrive/DVC_experiment/camData.json");
  fil2 <<" { \"intrinsic\": " << K << "," << endl;
  fil2 <<" \"pose\": ["  <<endl;

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
    cv::imwrite(STRING("/home/jung-su/GoogleDrive/DVC_experiment/img_"<<l<<".png").p, bgr);
    if(l>0) fil2 << "," <<endl;
    fil2 << C["rs_camera"]->getPose() <<endl;

    rai::wait(.2);
  }
  fil2 <<  "]}" <<endl;
  fil2.close();

  rai::wait();
}

//===========================================================================

void takePictures(uint numImages, bool onGripper=false){

  rai::Configuration C;
  C.addFile("../../rai-robotModels/scenarios/pandasTable-calibrated.g");

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
