/* Copyright (c) 2018 Gregor Richards
 * Copyright (c) 2017 Mozilla */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include "rnnoise.h"
#include <stdint.h>
#define FRAME_SIZE 480

#include "helper.h"

inline DenoiseState * rnnoise_new(){
    return(rnnoise_create(NULL));
}
// The wrapped of rnnoise's |rnnoise_process_frame| function so as to make sure its input/outpu is |f32| format.
// Note tha the frame size is fixed 480.
float rnnoise_process(DenoiseState* pRnnoise, float* pFrameOut, float* pFrameIn){
    unsigned int n;
    float vadProb;
    float buffer[FRAME_SIZE];

    // Note: Be careful for the format of the input data.
    for (n = 0; n < FRAME_SIZE; ++n){
        buffer[n] = pFrameIn[n] * 32768.0f;
    }

    vadProb = rnnoise_process_frame(pRnnoise, buffer, buffer);
    for (n = 0; n < FRAME_SIZE; ++n){
        pFrameOut[n] = drwav_clamp(buffer[n], -32768, 32767) * (1.0f / 32768.0f);
    }
    return vadProb;
}

int rnnoise_free(DenoiseState* pRnnoise){
    if (!pRnnoise){
        return -1;
    }

    rnnoise_destroy(pRnnoise);
    return 0;
}

int rnnoise_demo(const char* inFile, const char* outFile) {

    drwav* pWavOut = NULL;
    // Step 0: read wav data
    printf("open file read...\n");
    drwav* pWavIn = open_wavfile(inFile);
    if (pWavIn == NULL) {
        printf("Cannot open wav file\n");
        return -1;
    }

    uint16_t channels = pWavIn->channels;
    uint16_t sampleRate = pWavIn->sampleRate;
    uint64_t totalFrameCnt = pWavIn->totalPCMFrameCount;
    if (channels != 1) {
        printf("Only support mono wav file\n");
        uninit_wavfile(pWavIn);
        return -1;
    }

    printf("open file write\n");
	if ( NULL != outFile)
	{
    	pWavOut = init_wavfile(outFile, 1, 48000, 32, DR_WAVE_FORMAT_IEEE_FLOAT);
    	if (!pWavOut) {
        	printf("Cannot open output file\n");
        	drwav_uninit(pWavIn);
        	return -1;
    	}
	}

    // Step 1: create rnnoise state
    printf("Initialize rnnoise\n");
    DenoiseState* pRnnoise = rnnoise_new(NULL);
    if (!pRnnoise) {
        printf("initialized rnnoise error\n");
		uninit_wavfile(pWavIn);
        if (NULL != pWavOut)
		    uninit_wavfile(pWavOut);
        return -1;
    }


    // Step 2: rnnoise frame process and output
    float *frameIn = (float *)malloc(totalFrameCnt*sizeof(float));
    float *frameOut = (float*)malloc(totalFrameCnt * sizeof(float));
    get_frame_f32(pWavIn, frameIn, totalFrameCnt);

    printf("Run...\n");
    double startTime = now();

    for (size_t n = 0; n < totalFrameCnt / FRAME_SIZE; n++) {
        rnnoise_process(pRnnoise, (frameOut + n * FRAME_SIZE), (frameIn + n * FRAME_SIZE));
    }

    double timeInterval = calcElapsed(startTime, now()) * 1000;
    printf("Run end,time interval is\t%lf ms, each frame cost %lf ms / 10ms\n", timeInterval, timeInterval / (totalFrameCnt / FRAME_SIZE));

    if (NULL != pWavOut) {
        write_frames_f32(pWavOut, totalFrameCnt, frameOut);
    }

    free(frameIn);
    free(frameOut);

    // Step 3: uninit wavfile and rnnoise object
	rnnoise_destroy(pRnnoise);
	
    uninit_wavfile(pWavIn);
	if (NULL != pWavOut) {
    	uninit_wavfile(pWavOut);
	}
    
    return 0;
}

int main(int argc, char **argv)
{
    char inFile[256];
    char outFile[256];

#ifdef _WIN32
    // Windows platform
    sprintf(inFile, "audio_in_with_noise_48k.wav");
    sprintf(outFile, "audio_in_with_noise_ns.wav");
    rnnoise_demo(inFile, outFile);
    return 0;
#endif // _WIN32

    // Linux platform
    if (argc == 1) {
        printf("Usage: %s <inFile> <outFile>\n", argv[0]);
        return  0;
    }

    sprintf(inFile, argv[1]);
    sprintf(outFile, argv[2]);

    rnnoise_demo(inFile, outFile);
    return 0;
}

// A  modified official demo for reference, only support for 16-bit PCM file
int rnnoise_demo_official(int argc, char** argv) {
  int i;
  int first = 1;
  float x[FRAME_SIZE];
  FILE *f1, *fout;
  DenoiseState *st;
  st = rnnoise_create(NULL);
  if (argc!=3) {
    fprintf(stderr, "usage: %s <noisy speech> <output denoised>\n", argv[0]);
    return 1;
  }
  f1 = fopen(argv[1], "rb");
  fout = fopen(argv[2], "wb");
  while (1) {
    short tmp[FRAME_SIZE];
    fread(tmp, sizeof(short), FRAME_SIZE, f1);
    if (feof(f1)) break;
    for (i=0;i<FRAME_SIZE;i++) x[i] = tmp[i];
    rnnoise_process_frame(st, x, x);
    for (i=0;i<FRAME_SIZE;i++) tmp[i] = x[i];
    if (!first) fwrite(tmp, sizeof(short), FRAME_SIZE, fout);
    first = 0;
  }
  rnnoise_destroy(st);
  fclose(f1);
  fclose(fout);
  return 0;
}
