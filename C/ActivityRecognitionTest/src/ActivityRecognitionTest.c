/*
 ============================================================================
 Name        : ActivityRecognitionTest.c
 Author      : Madison Blake
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>

int SimpleTest();

int main(void) {

  if(SimpleTest() == 1)
  {
	  printf("Simple Test Failed!\n");
  }

  printf("done\n");
  return EXIT_SUCCESS;
}
