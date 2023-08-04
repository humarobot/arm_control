#include "ArxDriver.h"

int main(int argc, char** argv){
  ArxDriver driver(500);

  while(1){
    driver.Update();
  }
  return 0;
}