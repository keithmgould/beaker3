#include "../utils/ftoa.cpp"
#include <stdio.h>
#include <string.h>

int main() {
  char str[80];
  strcpy(str, "these ");
  strcat(str, "strings ");
  strcat(str, "are ");
  strcat(str, "concatenated.");

  printf("%s",str);

  char res[20];
  float n = 233.007;
  ftoa(n, res, 4);
  printf("\n\"%s\"\n", res);
}

