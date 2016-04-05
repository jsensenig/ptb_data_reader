#include <iostream>

void myfunc(char* &answer) {

  //  char* test = new char[50];
  answer = new char[50];
  sprintf(answer,"This is the answer.");
  //  answer = strdup(test);
  // delete [] test;
}

int main() {
  char *answer = NULL;
  myfunc(answer);
  std::cout << "The answer is : " << answer << std::endl;
  delete [] answer;
  //free(answer);
  return 0;
}
