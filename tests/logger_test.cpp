#include "../utils/logger.cpp"

int main(){
  Logger logger;
  printf("%s\n",logger.getStr());
  logger.addFloat(234.5678);
  printf("%s\n",logger.getStr());
  logger.addInt(4321);
  printf("%s\n",logger.getStr());
  return 0;
}
