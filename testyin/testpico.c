// clang -g ../pitch/*.c ../pitch/*/*.c testpico.c -o testpico

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>

#include "pico/stdlib.h"

#include "test1.h"
#include "flux_base.h"
#include "_pitch_yin.h"
#include "testdata.h"

static void testYIN();


int main(int argc, char const *argv[]){
	struct timeval tv;
	long t1=0,t2=0;

	stdio_init_all();

	gettimeofday(&tv, NULL);
	t1=tv.tv_sec*1000+tv.tv_usec/1000;

	printf("testpico\n"); fflush(stdin);

	testYIN();

	gettimeofday(&tv, NULL);
	t2=tv.tv_sec*1000+tv.tv_usec/1000;
	printf("cost time is %0.3f\n",(t2-t1)/1000.0);

	return 0;
}

static void testYIN(){
	PitchYINObj pitch=NULL;
	
	int samplate=32000;

	float lowFre=32;
	float highFre=2000;

	// int radix2Exp=12; // 4096
	// int slideLength=(1<<radix2Exp)/4;

	// max radix2Exp is 2048 on Pico
	int radix2Exp=11; // 2048
	int slideLength=(1<<radix2Exp)/4;

	// WindowType winType=Window_Hamm;

	int isContinue=0;

	// int timeLength=0;
	float *freArr=NULL;
	float *arr1;
	float *arr2;
	arr1 = NULL;
	arr2 = NULL;


	// float *dataArr=NULL;
	// int dataLength=0;
	// int offset=0;
	// int autoLength=2048;
	int autoLength=1024;

	// int start = 32000*5;  // start after 5s
	int len = 2048; 

	pitchYINObj_new(&pitch,
				&samplate, &lowFre, &highFre,
				&radix2Exp, &slideLength, &autoLength,
				&isContinue);

	freArr=__vnew(10, NULL);
	
	pitchYINObj_pitch(pitch,testdata,len,freArr, arr1, arr2);
	printf("f: %.1f\n", freArr[0]);


	free(freArr);

	pitchYINObj_free(pitch);
}




