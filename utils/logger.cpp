#include "ftoa.cpp"
#include <math.h>
#include <stdio.h>
#include <string.h>

class Logger
{
  private:
  char str[100];

  void addComma(){
    strcat(str, ", ");
  }

  public:

  Logger()
  {
    strcpy(str, ">");
  }

  void addFloat(float f)
  {
    char res[10];
    ftoa(f, res, 4);
    strcat(str,res);
    addComma();
  }

  void addInt(int i)
  {
    char res[10];
    intToStr(i, res, 1);
    strcat(str,res);
    addComma();
  }

  char * getStr()
  {
    return str;
  }
};
