#include <BotOp/bot.h>

void runSolution(){
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
  bot.move(path1, {10.});
  while(bot.step(C)){}

  bot.gripperR->close(.5, .2, .5); while(!bot.gripperR->isDone()) rai::wait(.1);

  //place
  bot.move(path2, {10.});
  while(bot.step(C)){}

  bot.gripperR->open(); while(!bot.gripperR->isDone()) rai::wait(.1);

  bot.home(C);
}
