#include <BotOp/bot.h>

void runSolution(float timePerPhase){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("myWorld.g");

  //-- load path
  arrA fullPath;
  fullPath <<FILE("/home/jung-su/GoogleDrive/DVC_experiment/traj.json");
  CHECK_EQ(fullPath.N, 80, "");

  arr path1(40, fullPath.first().N);
  arr path2(40, fullPath.first().N);

  for(uint i=0;i<path1.d0;i++) path1[i] = fullPath(i);
  for(uint i=0;i<path2.d0;i++) path2[i] = fullPath(i+40);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  //grasp
  bot.move(path1, {timePerPhase});
  while(bot.step(C)){}

  bot.gripperR->close(.5, .0, .5); while(!bot.gripperR->isDone()) rai::wait(.1);

  //place
  bot.move(path2, {timePerPhase});
  while(bot.step(C)){}

  bot.gripperR->open(); while(!bot.gripperR->isDone()) rai::wait(.1);

  bot.home(C);
}

void runSolutionHO(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("myWorld.g");

  //-- load path
  arrA fullPath;
  fullPath <<FILE("/home/jung-su/GoogleDrive/DVC_experiment/traj.json");
  CHECK_EQ(fullPath.N, 120, "");

  arr path1(40, fullPath.first().N);
  arr path2(40, fullPath.first().N);
  arr path3(40, fullPath.first().N);

  for(uint i=0;i<path1.d0;i++) path1[i] = fullPath(i);
  for(uint i=0;i<path2.d0;i++) path2[i] = fullPath(i+40);
  for(uint i=0;i<path3.d0;i++) path3[i] = fullPath(i+80);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  //grasp1
  bot.move(path1, {5.});
  while(bot.step(C)){}

  bot.gripperR->close(.5, .0, .5); while(!bot.gripperR->isDone()) rai::wait(.1);

  //grasp2
  bot.move(path2, {5.});
  while(bot.step(C)){}

  bot.gripperL->close(.5, .0, .5); while(!bot.gripperL->isDone()) rai::wait(.1);
  bot.gripperR->open(); while(!bot.gripperR->isDone()) rai::wait(.1);

  //place
  bot.move(path3, {5.});
  while(bot.step(C)){}

  bot.gripperL->open(); while(!bot.gripperL->isDone()) rai::wait(.1);




  bot.home(C);
}
