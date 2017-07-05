/*
  Copyright (C) 2014 Parrot SA

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.
  * Neither the name of Parrot nor the names
  of its contributors may be used to endorse or promote products
  derived from this software without specific prior written
  permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
  SUCH DAMAGE.
*/
/**
 * @file BebopDroneDecodeStream.c
 * @brief This file contains sources about basic arsdk example decoding video stream from a BebopDrone with ffmpeg
 * @date 08/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#ifdef __cplusplus
extern "C"{
#endif
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#ifdef __cplusplus
}
#endif
#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>

#include "BebopDroneDecodeStream.h"
//後付け
#include <iostream>
#include<fstream>

std::ofstream rocLog;

/*****************************************
 *
 *             define :
 *
 *****************************************/
#define TAG "BebopDroneReceiveStream"
#define BD_IP_ADDRESS "192.168.42.1"
#define BD_DISCOVERY_PORT 44444
#define BD_C2D_PORT 54321 // should be read from Json
#define BD_D2C_PORT 43210

#define BD_NET_CD_NONACK_ID 10
#define BD_NET_CD_ACK_ID 11
#define BD_NET_CD_EMERGENCY_ID 12
#define BD_NET_CD_VIDEO_ACK_ID 13
#define BD_NET_DC_NAVDATA_ID 127
#define BD_NET_DC_EVENT_ID 126
#define BD_NET_DC_VIDEO_DATA_ID 125

#define BD_NET_DC_VIDEO_FRAG_SIZE 65000
#define BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG 4

#define BD_RAW_FRAME_BUFFER_SIZE 50
#define BD_RAW_FRAME_POOL_SIZE 50

#define ERROR_STR_LENGTH 2048


#define FRONT_ROC 0.35 //正面向いてる時のROC
#define SIDE_ROC 0.135 //側面ROC
#define Kpp 0.01
#define Kpi 0.0000
#define Kpd 0.6

#define Kyp 3.0000
#define Kyi 0.0000
#define Kyd 15.0000

#define Kgp	1.0
#define Kgi 0.0000
#define Kgd 7.0000

#define Krp 30

#define Kri 80	//使用していない
#define ROLL_COUNT 5
#define STOP_COUNT 3

enum ContType {
	CT_yaw,
	CT_pitch,
	CT_gaz
};

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"

static char fifo_dir[] = FIFO_DIR_PATTERN;
static char fifo_name[128] = "";

/** Handle older ffmpeg/libav versions */
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#define av_frame_free   avcodec_free_frame
#endif

int getNextDataCallback(uint8_t **data, void *customData);
void* Decode_RunDataThread(void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);
/**
 * @brief Component of a frame.
 */
struct RawFrame_t
{
    uint8_t *data; /**< data buffer*/
    uint32_t size; /**< size of the buffer */
    uint8_t isIframe;
};

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

static ARNETWORK_IOBufferParam_t c2dParams[] = {
    /* Non-acknowledged commands. */
    {
        BD_NET_CD_NONACK_ID,
        ARNETWORKAL_FRAME_TYPE_DATA,
        20,
        ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        2,
        128,
        1,
    },
    /* Acknowledged commands. */
    {
        BD_NET_CD_ACK_ID,
        ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        20,
        500,
        3,
        20,
        128,
        0,
    },
    /* Emergency commands. */
    {
        BD_NET_CD_EMERGENCY_ID,
        ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        10,
        100,
        ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        1,
        128,
        0,
    },
    /* Video ACK (Initialized later) */
    {
        BD_NET_CD_VIDEO_ACK_ID,
        ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
        0,
        0,
        0,
        0,
        0,
        0,
    },
};
static const size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

static ARNETWORK_IOBufferParam_t d2cParams[] = {
    {
        BD_NET_DC_NAVDATA_ID,
        ARNETWORKAL_FRAME_TYPE_DATA,
        20,
        ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        20,
        128,
        0,
    },
    {
        BD_NET_DC_EVENT_ID,
        ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        20,
        500,
        3,
        20,
        128,
        0,
    },
    /* Video data (Initialized later) */
    {
        BD_NET_DC_VIDEO_DATA_ID,
        ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
        0,
        0,
        0,
        0,
        0,
        0,
    },
};
static const size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

static int commandBufferIds[] = {
    BD_NET_DC_NAVDATA_ID,
    BD_NET_DC_EVENT_ID,
};
static const size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

int gIHMRun = 0;
char gErrorStr[ERROR_STR_LENGTH];

// reader thread
static void *readerRun (void* data)
{
    BD_MANAGER_t *deviceManager = NULL;
    int bufferId = 0;
    int failed = 0;

    // Allocate some space for incoming data.
    const size_t maxLength = 128 * 1024;
    void *readData = malloc (maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }

    if (!failed)
    {
        // get thread data.
        if (data != NULL)
        {
            bufferId = ((READER_THREAD_DATA_t *)data)->readerBufferId;
            deviceManager = ((READER_THREAD_DATA_t *)data)->deviceManager;

            if (deviceManager == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        while (deviceManager->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;

            // read data
            netError = ARNETWORK_Manager_ReadDataWithTimeout (deviceManager->netManager, bufferId, static_cast<uint8_t *>(readData), maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNETWORK_Manager_ReadDataWithTimeout () failed : %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }

            if (!skip)
            {
                // Forward data to the CommandsManager
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeBuffer ((uint8_t *)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer ((uint8_t *)readData, length, msg, sizeof(msg));
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARCOMMANDS_Decoder_DecodeBuffer () failed : %d %s", cmdError, msg);
                }
            }
        }
    }

    if (readData != NULL)
    {
        free (readData);
        readData = NULL;
    }

    return NULL;
}

// decoder thread
void* Decode_RunDataThread(void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARCODECS_ERROR error;
    ARCODECS_Manager_Frame_t *decodedFrame = NULL;
    unsigned int height;
    unsigned int width;
    unsigned int compHeight[3];
    unsigned int compWidth[3];

	HOGDescriptor hog = HOGDescriptor();
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

    while (!deviceManager->decodingCanceled)
    {

        ARSAL_Mutex_Lock (&(deviceManager->mutex));
        RawFrame_t *rawFrame = deviceManager->rawFrameFifo[deviceManager->fifoReadIdx];
        ARSAL_Mutex_Unlock (&(deviceManager->mutex));

        if (rawFrame != NULL && rawFrame->isIframe == 1)
        {
            deviceManager->hasReceivedFirstIFrame = 1;
        }

        if (rawFrame != NULL)
        {
            if (deviceManager->hasReceivedFirstIFrame)
            {
                decodedFrame = ARCODECS_Manager_Decode(deviceManager->decoder, &error);
                height = decodedFrame->height;
                width = decodedFrame->width;

                /*
                for(int i = 0;i < 3;i++){
                	compHeight[i] = (decodedFrame->componentArray[i].size)/decodedFrame->componentArray[i].lineSize;
                	compWidth[i] = decodedFrame->componentArray[i].lineSize;
                }
                cout << "Vheight=" << compHeight[2] << " Vwidth=" << compWidth[2];
                */

                //2フレームに一度処理を行う
                /*if(!deviceManager->imageFlag){
                	deviceManager->imageFlag = 1;
                	imageProc(decodedFrame,hog,deviceManager);	//YuvからBGRに変換したMat作成
                }else{
                	deviceManager->imageFlag = 0;
                }*/

            }

            if (decodedFrame != NULL)
            {
                //ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Frame has been decoded ");

                // this part is only needed to copy the YUV frame into the file
                int pic_size = avpicture_get_size(AV_PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height);

                uint8_t *decodedOut = static_cast<uint8_t *>(malloc(pic_size));

                // in case your install of FFMpeg supports av_image_copy_to_buffer, you can replace the AVFrame creation and initialisation by this block
                /*uint8_t *src_data[4];
                src_data[0] = decodedFrame->componentArray[0].data;
                src_data[1] = decodedFrame->componentArray[1].data;
                src_data[2] = decodedFrame->componentArray[2].data;
                src_data[3] = NULL;

                int src_linesize[4];
                src_linesize[0] = decodedFrame->componentArray[0].lineSize;
                src_linesize[1] = decodedFrame->componentArray[1].lineSize;
                src_linesize[2] = decodedFrame->componentArray[2].lineSize;
                src_linesize[3] = 0;

                av_image_copy_to_buffer(decodedOut, pic_size,
                                        (const uint8_t *const *)src_data, src_linesize,
                                        AV_PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, 1);*/

                AVFrame *avFrame = av_frame_alloc();
                if (avFrame != NULL)
                {
                    avFrame->width = decodedFrame->width;
                    avFrame->height = decodedFrame->height;
                    avFrame->format = AV_PIX_FMT_YUV420P;

                    avpicture_fill((AVPicture*)avFrame, NULL, AV_PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height);
                    avFrame->linesize[0] = decodedFrame->componentArray[0].lineSize;
                    avFrame->linesize[1] = decodedFrame->componentArray[1].lineSize;
                    avFrame->linesize[2] = decodedFrame->componentArray[2].lineSize;

                    avFrame->data[0] = decodedFrame->componentArray[0].data;
                    avFrame->data[1] = decodedFrame->componentArray[1].data;
                    avFrame->data[2] = decodedFrame->componentArray[2].data;

                    avpicture_layout((AVPicture*)avFrame, AV_PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, decodedOut, pic_size);
                    av_frame_free(&avFrame);
                }

				if (decodedOut != NULL) {
					fwrite(decodedOut, pic_size, 1, deviceManager->video_out);

//					if (deviceManager->imageFlag == 2) {
//						deviceManager->imageFlag = deviceManager->imageFlag % 2;
//						imageProc2(decodedOut, hog, deviceManager);
//					} else {
//						deviceManager->imageFlag++;
//					}

					imageProc2(decodedOut, hog, deviceManager);
				}

                free (decodedOut);
                decodedOut = NULL;
            }

            ARSAL_Mutex_Lock (&(deviceManager->mutex));
            putRawFrameBackToPool(deviceManager, deviceManager->fifoReadIdx);
            deviceManager->fifoReadIdx = (deviceManager->fifoReadIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
            ARSAL_Mutex_Unlock (&(deviceManager->mutex));
        }
    }

    return (void*)0;
}

// looper thread
static void *looperRun (void* data)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)data;

    if(deviceManager != NULL)
    {
        while (deviceManager->run)
        {

            sendPCMD(deviceManager);

            sendCameraOrientation(deviceManager);
            usleep(50000);
        }
    }

    return NULL;
}

static void signal_handler(int signal)
{
    gIHMRun = 0;
}

int main (int argc, char *argv[])
{

    /* local declarations */
    try{
    	rocLog.open("rocLog.txt",std::ios::out);
    	if(rocLog.fail()){
    		throw "file reservation fail";
    	}

    }catch(const char* e){
    	return 0;
    }

    int failed = 0;
    BD_MANAGER_t *deviceManager = static_cast<BD_MANAGER_t *>(malloc(sizeof(BD_MANAGER_t)));
    pid_t child = 0;
    /* Set signal handlers */
    struct sigaction sig_action = {};
    sig_action.sa_handler = signal_handler;
    int ret = sigaction(SIGINT, &sig_action, NULL);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Unable to set SIGINT handler : %d(%s)",
                    errno, strerror(errno));
        return 1;
    }
    ret = sigaction(SIGPIPE, &sig_action, NULL);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Unable to set SIGPIPE handler : %d(%s)",
                    errno, strerror(errno));
        return 1;
    }


    if (mkdtemp(fifo_dir) == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkdtemp failed.");
        return 1;
    }
    snprintf(fifo_name, sizeof(fifo_name), "%s/%s", fifo_dir, FIFO_NAME);

    if(mkfifo(fifo_name, 0666) < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
        return 1;
    }


    // fork the process to launch ffplay
    if ((child = fork()) == 0)
    {
        // redirect stdout and stderr of mplayer to dev/null to avoid messing with ncurse
        int stdout_fd = open("/dev/null", O_RDONLY);
        if (stdout_fd < 0)
            return 1;
        dup2(stdout_fd, STDOUT_FILENO);
        close(stdout_fd);

        int stderr_fd = open("/dev/null", O_RDONLY);
        if (stderr_fd < 0)
            return 1;
        dup2(stderr_fd, STDERR_FILENO);
        close(stderr_fd);

        execlp("mplayer", "mplayer", fifo_name, "-demuxer", "rawvideo", "-rawvideo", "w=640:h=368:fps=30:format=i420", ">/dev/null", "2>/dev/null", NULL);
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer.");
        return -1;
    }

    if (deviceManager == NULL)
    {
        failed = 1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "deviceManager alloc error !");
    }

	if (!failed) {

		ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Starting --");

		// initialize jsMnager
		deviceManager->alManager = NULL;
		deviceManager->netManager = NULL;
		deviceManager->streamReader = NULL;
		deviceManager->looperThread = NULL;
		deviceManager->rxThread = NULL;
		deviceManager->txThread = NULL;
		deviceManager->videoRxThread = NULL;
		deviceManager->videoTxThread = NULL;
		deviceManager->d2cPort = BD_D2C_PORT;
		deviceManager->c2dPort = BD_C2D_PORT; //deviceManager->c2dPort = 0; // Should be read from json
		deviceManager->arstreamAckDelay = 0; // Should be read from json
		deviceManager->arstreamFragSize = BD_NET_DC_VIDEO_FRAG_SIZE; // Should be read from json
		deviceManager->arstreamFragNb = BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG; // Should be read from json
		deviceManager->video_out = fopen(fifo_name, "w");
		deviceManager->decoder = NULL;
		deviceManager->decodingCanceled = 1;
		deviceManager->decodingThread = NULL;

		deviceManager->ihm = NULL;

		deviceManager->hasReceivedFirstIFrame = 0;

		deviceManager->freeRawFramePool = NULL;
		deviceManager->rawFramePoolCapacity = 0;
		deviceManager->lastRawFrameFreeIdx = 0;

		deviceManager->rawFrameFifo = NULL;
		deviceManager->fifoReadIdx = 0;
		deviceManager->fifoWriteIdx = BD_RAW_FRAME_BUFFER_SIZE - 1;
		deviceManager->run = 1;

		deviceManager->dataPCMD.flag = 0;
		deviceManager->dataPCMD.roll = 0;
		deviceManager->dataPCMD.pitch = 0;
		deviceManager->dataPCMD.yaw = 0;
		deviceManager->dataPCMD.gaz = 0;

		deviceManager->dataCam.tilt = 0;
		deviceManager->dataCam.pan = 0;

		deviceManager->faceCascade.load("haarcascade_frontalface_alt.xml");
		deviceManager->fullbodyCascade.load("haarcascade_fullbody.xml");
		deviceManager->upperbodyCascade.load("haarcascade_upperbody.xml");

		deviceManager->imageFlag = 0;	//画像処理実行フラグ

		deviceManager->pastRoll = 0;
		deviceManager->rollFlag = 0;
		deviceManager->pastPixPerHeight = 0;	//0は未入力という意味
		deviceManager->pastROC = 0.0;
		deviceManager->currentROC = 0.0;
		deviceManager->eigenvectors = vector<vector<int> >(2,
				vector<int>(2, 0));
		deviceManager->contVariable = vector<vector<double> >(3,vector<double>(300,0));	//4行300列の2次元vector初期化
		deviceManager->ROCFlag = false;
		deviceManager->differenceROC = 0.0;
		deviceManager->pastDifferenceROC = 0.0;
		deviceManager->firstEV = 0.0;
		deviceManager->secondEV = 0.0;
		deviceManager->rollControllFlag = false;
		deviceManager->rocCount = 0;
		deviceManager->Ese = 0.0;
		deviceManager->Epe = 0.0;
		deviceManager->Eppe = 0.0;
		deviceManager->Ece = 0.0;

		deviceManager->Mp = 0;
		deviceManager->Mpp = 0;
		deviceManager->Mpd = 0;

		deviceManager->Epx = 0.0;
		deviceManager->Eppx = 0.0;
		deviceManager->Ecx = 0.0;

		deviceManager->My = 0;
		deviceManager->Mpy = 0;
		deviceManager->Myd = 0;

		deviceManager->Epy = 0.0;
		deviceManager->Eppy = 0.0;
		deviceManager->Ecy = 0.0;

		deviceManager->Mg = 0;
		deviceManager->Mpg = 0;
		deviceManager->Mgd = 0;
		deviceManager->plotType = 0;
		deviceManager->Kpg = 0;
		deviceManager->Kppitch = 0;
		deviceManager->Kpy = 0;
		deviceManager->Kig = 0;
		deviceManager->Kip = 0;
		deviceManager->Kiy = 0;
		deviceManager->Kdg = 0;
		deviceManager->Kdp = 0;
		deviceManager->Kdy = 0;
		deviceManager->isGetDifference = 0;
		deviceManager->isNegative = 0;
		deviceManager->Ecr = 0.0;
		deviceManager->Epr = 0.0;
		deviceManager->isStop = 0;
		deviceManager->isFront = 0;
		deviceManager->rollSeved = 0;
		deviceManager->numOfFace = 0;

		for(int i = 0;i < 6;i++){
			deviceManager->rocArray[i] = 0.0;
		}

		deviceManager->gp = popen("gnuplot -persist","w"); //グラフ描画用のパイプを開き、gnuplotを立ち上げる
		//deviceManager->ROC = vector<double>(0.0,0.0);
		//deviceManager->rocCount = -1;
		deviceManager->cameraCount = 0;
		deviceManager->time = 0;
		deviceManager->flyingState =
				ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
	}

    if (!failed)
    {
        failed = ardiscoveryConnect (deviceManager);
    }

    if (!failed)
    {
        ARSTREAM_Reader_InitStreamDataBuffer (&d2cParams[2], BD_NET_DC_VIDEO_DATA_ID, deviceManager->arstreamFragSize, deviceManager->arstreamFragNb);
        ARSTREAM_Reader_InitStreamAckBuffer (&c2dParams[3], BD_NET_CD_VIDEO_ACK_ID);
    }

    if (!failed)
    {
        failed = startNetwork (deviceManager);
    }

    if (!failed)
    {
        failed = startDecoder(deviceManager);
    }

    if (!failed)
    {
        failed = startVideo (deviceManager);
    }

    if (!failed)
    {
        int cmdSend = sendDate(deviceManager);
        failed = !cmdSend;
    }


    if (!failed)
    {
        int cmdSend = sendAllSettings(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        int cmdSend = sendAllStates(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        int cmdSend = sendBeginStream(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        // allocate reader thread array.
        deviceManager->readerThreads = static_cast<void **>(calloc(numOfCommandBufferIds, sizeof(ARSAL_Thread_t)));

        if (deviceManager->readerThreads == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads failed.");
            failed = 1;
        }
    }

    if (!failed)
    {
        // allocate reader thread data array.
        deviceManager->readerThreadsData = static_cast<READER_THREAD_DATA_t*>(calloc(numOfCommandBufferIds, sizeof(READER_THREAD_DATA_t)));

        if (deviceManager->readerThreadsData == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads data failed.");
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start reader threads.
        unsigned int readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
        {
            // initialize reader thread data
            deviceManager->readerThreadsData[readerThreadIndex].deviceManager = deviceManager;
            deviceManager->readerThreadsData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];

            if (ARSAL_Thread_Create(&(deviceManager->readerThreads[readerThreadIndex]), readerRun, &(deviceManager->readerThreadsData[readerThreadIndex])) != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of reader thread failed.");
                failed = 1;
            }
        }
    }

    if (!failed)
    {
        // Create and start looper thread.
        if (ARSAL_Thread_Create(&(deviceManager->looperThread), looperRun, deviceManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of looper thread failed.");
            failed = 1;
        }
    }

    if (!failed)
    {
        deviceManager->ihm = IHM_New (&onInputEvent);
        if (deviceManager->ihm != NULL)
        {
            gErrorStr[0] = '\0';
            ARSAL_Print_SetCallback (customPrintCallback); //use a custom callback to print, for not disturb ncurses IHM

            IHM_PrintHeader(deviceManager->ihm, "-- Bebop Drone Decode Video Stream --");
            registerARCommandsCallbacks (deviceManager);
            IHM_setCustomData(deviceManager->ihm, deviceManager);

            gIHMRun = 1;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of IHM failed.");
            failed = 1;
        }
    }

    if (!failed)
    {
        IHM_PrintInstruction(deviceManager->ihm, "Arrow keys to move ; \n'z' = up ; 's' = down ; 'q' = yaw left ; 'd' = yaw right; \nSpacebar to take off/land ; \n'k' = move camera down ; 'i' = move camera up ; 'j' = move camera left ; 'l' = move camera right ;\n'm' = EMERGENCY\n'esc' to quit");

        while (gIHMRun)
        {
            usleep(50);
        }

        IHM_PrintInfo(deviceManager->ihm, "Disconnecting ...");
    }
    if (deviceManager != NULL)
    {
        deviceManager->run = 0; // break threads loops

        // Stop looper Thread
        if (deviceManager->looperThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->looperThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->looperThread));
            deviceManager->looperThread = NULL;
        }

        if (deviceManager->readerThreads != NULL)
        {
            // Stop reader Threads
            unsigned int readerThreadIndex = 0;
            for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
            {
                if (deviceManager->readerThreads[readerThreadIndex] != NULL)
                {
                    ARSAL_Thread_Join(deviceManager->readerThreads[readerThreadIndex], NULL);
                    ARSAL_Thread_Destroy(&(deviceManager->readerThreads[readerThreadIndex]));
                    deviceManager->readerThreads[readerThreadIndex] = NULL;
                }
            }

            // free reader thread array
            free (deviceManager->readerThreads);
            deviceManager->readerThreads = NULL;
        }

        if (deviceManager->readerThreadsData != NULL)
        {
            // free reader thread data array
            free (deviceManager->readerThreadsData);
            deviceManager->readerThreadsData = NULL;
        }

        stopVideo (deviceManager);
        stopDecoder (deviceManager);
        stopNetwork (deviceManager);
        fclose (deviceManager->video_out);
        fprintf(deviceManager->gp,"exit\n");	//パイプを通してgnuplotを終了させる
        fflush(deviceManager->gp);
        pclose(deviceManager->gp);	//パイプを閉じる
        free (deviceManager);
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

    if (child > 0)
    {
        kill(child, SIGKILL);
    }

    unlink(fifo_name);
    rmdir(fifo_dir);

    return 0;
}

/************************** Connection part **************************/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager)
{
    int failed = 0;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- ARDiscovery Connection");

    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = ARDISCOVERY_Connection_New (ARDISCOVERY_Connection_SendJsonCallback, ARDISCOVERY_Connection_ReceiveJsonCallback, deviceManager, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while creating discoveryData : %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }

    if (!failed)
    {
        eARDISCOVERY_ERROR err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, BD_DISCOVERY_PORT, BD_IP_ADDRESS);
        if (err != ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while opening discovery connection : %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }

    ARDISCOVERY_Connection_Delete(&discoveryData);

    return failed;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataTx != NULL) && (dataTxSize != NULL) && (deviceManager != NULL))
    {
        *dataTxSize = sprintf((char *)dataTx, "{ \"%s\": %d,\n \"%s\": \"%s\",\n \"%s\": \"%s\" }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, deviceManager->d2cPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "toto",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "tata") + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataRx != NULL) && (dataRxSize != 0) && (deviceManager != NULL))
    {
        char *json = static_cast<char*>(malloc(dataRxSize + 1));
        strncpy(json, (char *)dataRx, dataRxSize);
        json[dataRxSize] = '\0';

        //read c2dPort ...

        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "    - ReceiveJson:%s ", json);

        free(json);
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}

/************************** Network part **************************/
int startNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARNetwork");

    // Create the ARNetworkALManager
    deviceManager->alManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        // Initilize the ARNetworkALManager
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(deviceManager->alManager, BD_IP_ADDRESS, BD_C2D_PORT, BD_D2C_PORT, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create the ARNetworkManager.
        deviceManager->netManager = ARNETWORK_Manager_New(deviceManager->alManager, numC2dParams, c2dParams, numD2cParams, d2cParams, pingDelay, onDisconnectNetwork, deviceManager, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->rxThread), ARNETWORK_Manager_ReceivingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->txThread), ARNETWORK_Manager_SendingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Tx thread failed.");
            failed = 1;
        }
    }

    // Print net error
    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWorkAL Error : %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWork Error : %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}

void stopNetwork (BD_MANAGER_t *deviceManager)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARNetwork");

    // ARNetwork cleanup
    if (deviceManager->netManager != NULL)
    {
        ARNETWORK_Manager_Stop(deviceManager->netManager);
        if (deviceManager->rxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->rxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->rxThread));
            deviceManager->rxThread = NULL;
        }

        if (deviceManager->txThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->txThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->txThread));
            deviceManager->txThread = NULL;
        }
    }

    if (deviceManager->alManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(deviceManager->alManager);
    }

    ARNETWORK_Manager_Delete(&(deviceManager->netManager));
    ARNETWORKAL_Manager_Delete(&(deviceManager->alManager));
}

void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "onDisconnectNetwork ...");
    gIHMRun = 0;
}

/************************** Video part **************************/
int startVideo(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARSTREAM_ERROR err = ARSTREAM_OK;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARStream");

    deviceManager->videoFrameSize = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
    deviceManager->videoFrame = static_cast<uint8_t*>(malloc (deviceManager->videoFrameSize));
    if (deviceManager->videoFrame == NULL)
    {
        failed = 1;
    }

    if (! failed)
    {
        deviceManager->streamReader = ARSTREAM_Reader_New (deviceManager->netManager, BD_NET_DC_VIDEO_DATA_ID, BD_NET_CD_VIDEO_ACK_ID, frameCompleteCallback, deviceManager->videoFrame, deviceManager->videoFrameSize, deviceManager->arstreamFragSize, deviceManager->arstreamAckDelay, deviceManager, &err);
        if (err != ARSTREAM_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during ARStream creation : %s", ARSTREAM_Error_ToString(err));
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->videoRxThread), ARSTREAM_Reader_RunDataThread, deviceManager->streamReader) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->videoTxThread), ARSTREAM_Reader_RunAckThread, deviceManager->streamReader) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video Tx thread failed.");
            failed = 1;
        }
    }

    return failed;
}

void stopVideo(BD_MANAGER_t *deviceManager)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARStream");

    if (deviceManager->streamReader)
    {
        ARSTREAM_Reader_StopReader(deviceManager->streamReader);

        // Optionnal, but better for speed:
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        if (deviceManager->videoRxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->videoRxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->videoRxThread));
            deviceManager->videoRxThread = NULL;
        }
        if (deviceManager->videoTxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->videoTxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->videoTxThread));
            deviceManager->videoTxThread = NULL;
        }

        ARSTREAM_Reader_Delete (&(deviceManager->streamReader));
    }
}

uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom)
{
    uint8_t *ret = NULL;
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)custom;

    switch(cause)
    {
    case ARSTREAM_READER_CAUSE_FRAME_COMPLETE:
    {
        //ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Did receive a frame");

        RawFrame_t *freeFrame = getFrameFromData(deviceManager, frame);
        if (freeFrame != NULL)
        {
            freeFrame->data = frame;
            freeFrame->size = frameSize;
            freeFrame->isIframe = isFlushFrame;
        }

        if (isFlushFrame)
        {
            // since we received an iFrame, flush the fifo
            flushFifo(deviceManager);
        }

        if (freeFrame != NULL)
        {
            // push the received frame into the fifo
            freeFrame->data = frame;
            freeFrame->size = frameSize;

            addFreeRawFrameToFifo(deviceManager, freeFrame);
        }

        // get free frame for the next frame
        RawFrame_t *freeRawFrame = getFreeRawFrame(deviceManager);
        ret = freeRawFrame->data;
        *newBufferCapacity = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
    }

    break;
    case ARSTREAM_READER_CAUSE_FRAME_TOO_SMALL:
        /* This case should not happen, as we've allocated a frame pointer to the maximum possible size. */
    {
        RawFrame_t *freeRawFrame = getFreeRawFrame(deviceManager);
        ret = freeRawFrame->data;
        *newBufferCapacity = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
    }
    break;
    case ARSTREAM_READER_CAUSE_COPY_COMPLETE:
        /* Same as before ... but return value are ignored, so we just do nothing */
        break;
    case ARSTREAM_READER_CAUSE_CANCEL:
        /* Called when the library closes, return values ignored, so do nothing here */
        break;
    default:
        break;
    }

    return ret;
}

/************************** Commands part **************************/
int sendPCMD(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    // Send pcmd command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingPCMD(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataPCMD.flag, (uint8_t)deviceManager->dataPCMD.roll, deviceManager->dataPCMD.pitch, (uint8_t)deviceManager->dataPCMD.yaw, (uint8_t)deviceManager->dataPCMD.gaz, 0);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }

    return sentStatus;
}

int sendCameraOrientation(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    // Send camera orientation command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3CameraOrientation(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataCam.tilt, (uint8_t)deviceManager->dataCam.pan);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }

    return sentStatus;
}

int sendDate(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send date");

    // Send date command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, "2015-04-20");
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    if (sentStatus)
    {
        // Send time command
        cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, "'T'101533+0200");
        if (cmdError == ARCOMMANDS_GENERATOR_OK)
        {
            netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
        }

        if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
            sentStatus = 0;
        }
    }

    return sentStatus;
}

int sendAllSettings(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all settings");

    // Send get all settings command
    cmdError = ARCOMMANDS_Generator_GenerateCommonSettingsAllSettings(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all settings command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;

}

int sendAllStates(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all states");

    // Send get all states command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendBeginStream(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Streaming Begin");

    // Send Streaming begin command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendTakeoff(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send take off");

    // Send take off command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingTakeOff(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send takeoff command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendLanding(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send landing");

    // Send landing command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingLanding(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send landing command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendEmergency(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Emergency");

    // Send emergency command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingEmergency(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_EMERGENCY_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send emergency command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}
int settingMaxTilt(BD_MANAGER_t *deviceManager){
	int sentStatus = 1;
	u_int8_t cmdBuffer[128];
	int32_t cmdSize = 0;
	eARCOMMANDS_GENERATOR_ERROR cmdError;
	eARNETWORK_ERROR netError = ARNETWORK_ERROR;
	ARSAL_PRINT(ARSAL_PRINT_INFO,TAG,"- set max tilt");
	float maxTiltDeg = 35.0;

	cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingSettingsMaxTilt (cmdBuffer, sizeof(cmdBuffer), &cmdSize, maxTiltDeg);
	if(cmdError == ARCOMMANDS_GENERATOR_OK){
		netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback),1);
	}
	if((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK)){
		ARSAL_PRINT(ARSAL_PRINT_WARNING,TAG, "Failed to send settingMaxTilt command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
		sentStatus = 0;
	}
	return sentStatus;
}

int settingMaxRotationSpeed(BD_MANAGER_t *deviceManager){
	int sentStatus = 1;
		u_int8_t cmdBuffer[128];
		int32_t cmdSize = 0;
		eARCOMMANDS_GENERATOR_ERROR cmdError;
		eARNETWORK_ERROR netError = ARNETWORK_ERROR;
		ARSAL_PRINT(ARSAL_PRINT_INFO,TAG,"- set max rotation speed");
		float maxRotationSpeed = 35.0;

		cmdError = ARCOMMANDS_Generator_GenerateARDrone3SpeedSettingsMaxRotationSpeed (cmdBuffer, sizeof(cmdBuffer), &cmdSize, maxRotationSpeed);
		if(cmdError == ARCOMMANDS_GENERATOR_OK){
			netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback),1);
		}
		if((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK)){
			ARSAL_PRINT(ARSAL_PRINT_WARNING,TAG, "Failed to send settingMaxRotationSpeed command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
			sentStatus = 0;
		}
		return sentStatus;
}
int settingMaxVerticalSpeed(BD_MANAGER_t *deviceManager){
	int sentStatus = 1;
		u_int8_t cmdBuffer[128];
		int32_t cmdSize = 0;
		eARCOMMANDS_GENERATOR_ERROR cmdError;
		eARNETWORK_ERROR netError = ARNETWORK_ERROR;
		ARSAL_PRINT(ARSAL_PRINT_INFO,TAG,"- set max vertical speed");
		float maxVerticalSpeed = 3.0;
		ARCOMMANDS_Generator_GenerateARDrone3SpeedSettingsMaxVerticalSpeed (cmdBuffer , sizeof(cmdBuffer), &cmdSize, maxVerticalSpeed);
		if(cmdError == ARCOMMANDS_GENERATOR_OK){
			netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback),1);
		}
		if((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK)){
			ARSAL_PRINT(ARSAL_PRINT_WARNING,TAG, "Failed to send settingMaxVerticalSpeed command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
			sentStatus = 0;
		}
		return sentStatus;
}
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "    - arnetworkCmdCallback %d, cause:%d ", buffer_id, cause);

    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }

    return retval;
}

/************************** Commands callback part **************************/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager)
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback(batteryStateChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(flyingStateChangedCallback, deviceManager);
    // ADD HERE THE CALLBACKS YOU ARE INTERESTED IN
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(speedChangedCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback (attitudeChangedCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback (altitudeChangedCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingSettingsStateMaxTiltChangedCallback (MaxTiltChangedCallback_t,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3SpeedSettingsStateMaxRotationSpeedChangedCallback (maxRotationSpeedChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3SpeedSettingsStateMaxVerticalSpeedChangedCallback (maxVerticalSpeedChangedCallback, deviceManager);
}
void unregisterARCommandsCallbacks (void)
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback (NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(NULL,NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback (NULL,NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback (NULL,NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingSettingsStateMaxTiltChangedCallback (NULL,NULL);
    ARCOMMANDS_Decoder_SetARDrone3SpeedSettingsStateMaxVerticalSpeedChangedCallback (NULL, NULL);
}

void batteryStateChangedCallback (uint8_t percent, void *custom)
{
    // callback of changing of battery level
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;

    if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
    {
        IHM_PrintBattery (deviceManager->ihm, percent);
    }
}

void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    if (deviceManager != NULL)
    {
        deviceManager->flyingState = state;

        switch (deviceManager->flyingState) {
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED:
            IHM_PrintInfo(deviceManager->ihm, "Flying state : landed");
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_TAKINGOFF:
            IHM_PrintInfo(deviceManager->ihm, "Flying state : taking off");
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING:
            IHM_PrintInfo(deviceManager->ihm, "Flying state : hovering");
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING:
            IHM_PrintInfo(deviceManager->ihm, "Flying state : flying");
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDING:
            IHM_PrintInfo(deviceManager->ihm, "Flying state : landing");
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_EMERGENCY:
            IHM_PrintInfo(deviceManager->ihm, "Flying state : emergency");
            break;
        default:
            break;
        }
    }
}
void speedChangedCallback(float speedX,float speedY,float speedZ,void *custom){
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
	if(deviceManager != NULL){
		deviceManager->speedX = speedX;
		deviceManager->speedY = speedY;
		deviceManager->speedZ = speedZ;
	}
}
void attitudeChangedCallback(float roll, float pitch, float yaw, void *custom){
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
	if(deviceManager != NULL){
		deviceManager->roll = roll;
		deviceManager->pitch = pitch;
		deviceManager->yaw = yaw;
	}
}
void altitudeChangedCallback(double altitude, void *custom){
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
	if(deviceManager != NULL){
		deviceManager->altitude = altitude;
	}
}
void MaxTiltChangedCallback_t (float current, float min, float max, void *custom){
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
	if(deviceManager != NULL){
		deviceManager->maxTilt = max;
		deviceManager->minTilt = min;
		deviceManager->currentTilt = current;
	}
}
void maxRotationSpeedChangedCallback (float current, float min, float max, void *custom){
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
	if(deviceManager != NULL){
		deviceManager->maxRotationSpeed = max;
		deviceManager->minRotationSpeed = min;
		deviceManager->currentRotationSpeed = current;
	}
}
void maxVerticalSpeedChangedCallback (float current, float min, float max, void *custom){

	BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
	if(deviceManager != NULL){
		deviceManager->maxVerticalSpeed = max;
		deviceManager->minVerticalSpeed = min;
		deviceManager->currentVerticalSpeed = current;
	}
}
/************************** Decoding part **************************/
int startDecoder(BD_MANAGER_t *deviceManager)
{
    int failed = 0;

    // create ffmpeg decoder
    if (!failed)
    {
        eARCODECS_ERROR error;
        deviceManager->decoder = ARCODECS_Manager_New (getNextDataCallback, deviceManager, &error);
        if (error != ARCODECS_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        if (ARSAL_Mutex_Init (&(deviceManager->mutex)) != 0)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // prepare the pool : a large number of frame allocated
        deviceManager->freeRawFramePool = static_cast<RawFrame_t**>(malloc(sizeof(RawFrame_t*) * BD_RAW_FRAME_POOL_SIZE));
        if (deviceManager->freeRawFramePool != NULL)
        {
            deviceManager->rawFramePoolCapacity = BD_RAW_FRAME_POOL_SIZE;
        }
        else
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during pool init");
        }
    }

    if (!failed)
    {
        // Allocate all frames of the pool
        int i = 0;
        for (i = 0; (!failed) && (i < deviceManager->rawFramePoolCapacity); i++)
        {
            RawFrame_t *rawFrame = static_cast<RawFrame_t*>(malloc(sizeof(RawFrame_t)));
            if (rawFrame != NULL)
            {
                rawFrame->size = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
                rawFrame->isIframe = 0;
                rawFrame->data = static_cast<uint8_t*>(malloc (rawFrame->size));
                if (rawFrame->data == NULL)
                {
                    failed = 1;
                }
                else
                {
                    deviceManager->freeRawFramePool[i] = rawFrame;
                }
            }
            else
            {
                failed = 1;
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during pool object init");
            }
        }

        if (!failed)
        {
            deviceManager->lastRawFrameFreeIdx = 0;
        }
    }

    if (!failed)
    {
        // Init the frame fifo
        deviceManager->rawFrameFifo = static_cast<RawFrame_t**>(calloc (BD_RAW_FRAME_BUFFER_SIZE, sizeof(RawFrame_t*)));
        if (deviceManager->rawFrameFifo == NULL)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create the decoding thread
        deviceManager->decodingCanceled = 0;
        if (ARSAL_Thread_Create(&(deviceManager->decodingThread), Decode_RunDataThread, deviceManager) != 0)
        {
            deviceManager->decodingCanceled = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video decodingThread failed.");
            failed = 1;
        }
    }

    return failed;
}

void stopDecoder(BD_MANAGER_t *deviceManager)
{
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Stop decoding");
    if (deviceManager->decodingCanceled == 0)
    {
        deviceManager->decodingCanceled = 1;
        ARSAL_Thread_Join(deviceManager->decodingThread, NULL);
        ARSAL_Thread_Destroy(&(deviceManager->decodingThread));
        deviceManager->decodingThread = NULL;
    }

    if (deviceManager->videoFrame)
    {
        free (deviceManager->videoFrame);
        deviceManager->videoFrame = NULL;
    }

    if (deviceManager->freeRawFramePool != NULL)
    {
        flushFifo(deviceManager);
        int i = 0;
        for (i = 0; i < deviceManager->rawFramePoolCapacity; i++)
        {
            free(deviceManager->freeRawFramePool[i]);
            deviceManager->freeRawFramePool[i] = NULL;
        }

        free(deviceManager->freeRawFramePool);
        deviceManager->freeRawFramePool = NULL;
    }

    if (deviceManager->rawFrameFifo)
    {
        free (deviceManager->rawFrameFifo);
        deviceManager->rawFrameFifo = NULL;
    }

    if (deviceManager->mutex != NULL)
    {
        ARSAL_Mutex_Destroy(&deviceManager->mutex);
        deviceManager->mutex = NULL;
    }

    if (deviceManager->decoder != NULL)
    {
        ARCODECS_Manager_Delete(&deviceManager->decoder);
    }
}

RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager)
{
    // get a free raw frame in the pool
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *freeRawFrame = NULL;
    if (deviceManager->lastRawFrameFreeIdx < deviceManager->rawFramePoolCapacity)
    {
        freeRawFrame = deviceManager->freeRawFramePool[deviceManager->lastRawFrameFreeIdx];
        if (freeRawFrame == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Free frame is null.");
        }

        deviceManager->lastRawFrameFreeIdx++;
    }
    else
    {
        // there is no more free raw frame in the pool, create a new one and add it to the pool
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "No more frame free, need to alloc a new one and add it to the pool");
        freeRawFrame = static_cast<RawFrame_t*>(malloc(sizeof(RawFrame_t)));
        if (freeRawFrame != NULL)
        {
            RawFrame_t **freeRawFramePoolReallocated = static_cast<RawFrame_t**>(realloc(deviceManager->freeRawFramePool, deviceManager->rawFramePoolCapacity + 1));
            if (freeRawFramePoolReallocated != NULL)
            {
                deviceManager->freeRawFramePool = freeRawFramePoolReallocated;
                deviceManager->rawFramePoolCapacity++;

                deviceManager->freeRawFramePool[deviceManager->lastRawFrameFreeIdx] = freeRawFrame;
                deviceManager->lastRawFrameFreeIdx++;
            }
        }
    }
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    return freeRawFrame;
}

RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data)
{
    // get the frame which has the given data
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *rawFrame = NULL;
    int i = 0;
    for (i = 0; i < deviceManager->rawFramePoolCapacity; i++)
    {
        RawFrame_t *currentRawFrame = deviceManager->freeRawFramePool[i];
        if (currentRawFrame != NULL && currentRawFrame->data != NULL)
        {
            if (currentRawFrame->data == data)
            {
                rawFrame = currentRawFrame;
                break;
            }
        }
    }
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    return rawFrame;
}

void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame)
{
    // put the frame from the pool to the Fifo
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    int idxToWrite = (deviceManager->fifoWriteIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;

    // put the old frame on the free frame pool
    if (deviceManager->rawFrameFifo[idxToWrite] != NULL)
    {
        putRawFrameBackToPool(deviceManager, idxToWrite);
    }

    // put the new rawFrame at the write idx
    deviceManager->rawFrameFifo[idxToWrite] = rawFrame;
    deviceManager->fifoWriteIdx = idxToWrite;
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
}

void flushFifo(BD_MANAGER_t *deviceManager)
{
    ARSAL_Mutex_Lock (&(deviceManager->mutex));

    int currentRawFrameIdx = deviceManager->fifoReadIdx;
    do
    {
        if (deviceManager->rawFrameFifo[currentRawFrameIdx] != NULL)
        {
            putRawFrameBackToPool(deviceManager, currentRawFrameIdx);
        }
        currentRawFrameIdx = (currentRawFrameIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
    } while (currentRawFrameIdx != deviceManager->fifoReadIdx);
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
}

void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx)
{
    // remove the frame from the fifo and put it back to the pool => declare the frame as unused
    if (deviceManager->rawFrameFifo[fifoIdx] != NULL)
    {
        deviceManager->lastRawFrameFreeIdx--;

        deviceManager->rawFrameFifo[fifoIdx] = NULL;
    }
}

int getNextDataCallback(uint8_t **data, void *customData)
{
    // this callback is used by the decoder to get the frame to decode
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    int size = 0;

    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *rawFrame = deviceManager->rawFrameFifo[deviceManager->fifoReadIdx];
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));

    if (data != NULL && rawFrame != NULL)
    {
        *data = rawFrame->data;
        size = rawFrame->size;

    }
    return size;
}


/************************** IHM callbacks **************************/
void onInputEvent(eIHM_INPUT_EVENT event, void *customData, int autoFlag,Mat infoWindow) {
	// Manage IHM input events
	BD_MANAGER_t *deviceManager = (BD_MANAGER_t *) customData;
	stringstream pitch;
	pitch << "tilt:" <<deviceManager->dataCam.tilt;
	if (autoFlag == 1) {
		autonomousFlying(event, deviceManager,infoWindow);
	} else {
		if (abs(deviceManager->Ece) <= 250 && abs(deviceManager->Ecx) <= 10 && abs(deviceManager->Ecy) < 30	&& abs(deviceManager->Ecr) < 0.01 && deviceManager->numOfFace != 0) {
			deviceManager->isFront = 1;
		}else{
			deviceManager->isFront = 0;
		}

		deviceManager->currentRoll = 0;
		putText(infoWindow,"controlled flying",Point(184,320),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
		putText(infoWindow,pitch.str(),Point(100,100),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
		switch (event) {
		case IHM_INPUT_EVENT_EXIT:
			gIHMRun = 0;
			break;
		case IHM_INPUT_EVENT_EMERGENCY:
			if (deviceManager != NULL) {
				sendEmergency(deviceManager);
			}
			break;
		case IHM_INPUT_EVENT_TAKEOFF_LANDING:
			if (deviceManager != NULL) {
				if (deviceManager->flyingState
						== ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED) {
					sendTakeoff(deviceManager);
				} else if ((deviceManager->flyingState
						== ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING)
						|| (deviceManager->flyingState
								== ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING)) {
					sendLanding(deviceManager);
				}

			}
			break;
		case IHM_INPUT_EVENT_FORWARD:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.flag = 1;
				deviceManager->dataPCMD.pitch = 50;
			}
			break;
		case IHM_INPUT_EVENT_BACK:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.flag = 1;
				deviceManager->dataPCMD.pitch = -50;
			}
			break;
		case IHM_INPUT_EVENT_RIGHT:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.flag = 1;
				deviceManager->dataPCMD.roll = 50;
			}
			break;
		case IHM_INPUT_EVENT_LEFT:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.flag = 1;
				deviceManager->dataPCMD.roll = -50;
			}
			break;
		case IHM_INPUT_EVENT_YAW_RIGHT:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.yaw = 50;
			}
			break;
		case IHM_INPUT_EVENT_YAW_LEFT:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.yaw = -50;
			}
			break;
		case IHM_INPUT_EVENT_UP:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.gaz = 50;
			}
			break;
		case IHM_INPUT_EVENT_DOWN:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.gaz = -50;
			}
			break;
		case IHM_INPUT_EVENT_CAM_UP:
			if (deviceManager != NULL) {
				deviceManager->dataCam.tilt += 2;
				if (deviceManager->dataCam.tilt > 80) {
					deviceManager->dataCam.tilt = 80;
				}
			}
			break;
		case IHM_INPUT_EVENT_CAM_DOWN:
			if (deviceManager != NULL) {
				deviceManager->dataCam.tilt -= 2;
				if (deviceManager->dataCam.tilt < -80) {
					deviceManager->dataCam.tilt = -80;
				}
			}
			break;
		case IHM_INPUT_EVENT_CAM_RIGHT:
			if (deviceManager != NULL) {
				deviceManager->dataCam.pan += 2;
				if (deviceManager->dataCam.pan > 80) {
					deviceManager->dataCam.pan = 80;
				}
			}
			break;
		case IHM_INPUT_EVENT_CAM_LEFT:
			if (deviceManager != NULL) {
				deviceManager->dataCam.pan -= 2;
				if (deviceManager->dataCam.pan < -80) {
					deviceManager->dataCam.pan = -80;
				}
			}
			break;
		case IHM_INPUT_EVENT_NONE:
			if (deviceManager != NULL) {
				deviceManager->dataPCMD.flag = 0;
				deviceManager->dataPCMD.roll = 0;
				deviceManager->dataPCMD.pitch = 0;
				deviceManager->dataPCMD.yaw = 0;
				deviceManager->dataPCMD.gaz = 0;
			}
			break;
		default:
			break;
		}
	}

}

int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
    // Custom callback used when ncurses is runing for not disturb the IHM

    if ((level == ARSAL_PRINT_ERROR) && (strcmp(TAG, tag) == 0))
    {
        // save the last Error
        vsnprintf(gErrorStr, (ERROR_STR_LENGTH - 1), format, va);
        gErrorStr[ERROR_STR_LENGTH - 1] = '\0';
    }

    return 1;
}

/************************** Image processing part **************************/
void imageProc(struct _ARCODECS_Manager_Frame_t_* frame,HOGDescriptor hog,BD_MANAGER_t *deviceManager){
	unsigned int height = 368;
	unsigned int width = 640;
	unsigned int yHeight = 368;
	unsigned int yWidth = 672;
	unsigned int index = 0;
	Mat bgrImage(height,width,CV_8UC3);	//mat初期化
	Mat yuvImage(height,width,CV_8UC3);
	Mat yComp(height,width,CV_8UC1);
	Mat uComp(height,width,CV_8UC1);
	Mat vComp(height,width,CV_8UC1);
	vector<Mat> splitYUV(3);
	vector<Rect> faces,ubodys,fbodys;
	stringstream ssFaceRectSize,ssUbodyRectSize,ssFbodyRectSize,ssAltitude,ssSpeed,ssAttitude,ssMaxTilt,ssMaxRotationSpeed,ssMaxVerticalSpeed;

	for(int i = 0;i < yHeight*yWidth; i++){

		if((i%yWidth)+1 > width) continue;

		yComp.data[index] = frame->componentArray[0].data[i];
		//uComp.data[index] = frame->componentArray[1].data[i/2];
		//vComp.data[index] = frame->componentArray[2].data[i/2];
		index++;
	}
	ssAltitude << "altitude:" << deviceManager->altitude;
	ssSpeed << "speedX:" << deviceManager->speedX << " speedY:" << deviceManager->speedY <<  "speedZ:" << deviceManager->speedZ;
	ssAttitude << "yaw:" << deviceManager->yaw << " pitch:" << deviceManager->pitch << " roll:" << deviceManager->roll;
	ssMaxTilt << "currentTilt:" << deviceManager->currentTilt << " maxTilt:" << deviceManager->maxTilt << " minTilt:" << deviceManager->minTilt;
	ssMaxRotationSpeed << "currentRotationSpeed:" << deviceManager->currentRotationSpeed << " maxRotationSpeed:" << deviceManager->maxRotationSpeed << " minRotationSpeed:" << deviceManager->minRotationSpeed;
	ssMaxVerticalSpeed << "currentVerticalSpeed:" << deviceManager->currentVerticalSpeed << " maxVerticalSpeed:" << deviceManager->maxVerticalSpeed << " minVerticalSpeed:" << deviceManager->minVerticalSpeed;
	putText(yComp,ssAltitude.str(),Point(0,10),0,0.5,Scalar(255,255,255));
	putText(yComp,ssSpeed.str(),Point(0,30),0,0.5,Scalar(255,255,255));
	putText(yComp,ssAttitude.str(),Point(0,50),0,0.5,Scalar(255,255,255));
	putText(yComp,ssMaxTilt.str(),Point(0,70),0,0.5,Scalar(255,255,255));
	putText(yComp,ssMaxRotationSpeed.str(),Point(0,90),0,0.5,Scalar(255,255,255));
	putText(yComp,ssMaxVerticalSpeed.str(),Point(0,110),0,0.5,Scalar(255,255,255));
	deviceManager->faceCascade.detectMultiScale(yComp,faces,1.1,2,0|CASCADE_SCALE_IMAGE,Size(30,30));
	//deviceManager->upperbodyCascade.detectMultiScale(yComp,ubodys,1.1,2,0|CASCADE_SCALE_IMAGE,Size(80,80));
	//hog.detectMultiScale(yComp,fbodys,0,Size(),Size(),1.05,5,false);

	//検出された矩形データの表示*3
	if(faces.size()){
		ssFaceRectSize << "X:" << faces[0].width << "Y:" << faces[0].height;
		putText(yComp,ssFaceRectSize.str(),Point((faces[0].x)-20,(faces[0].y)-20),0,0.5,Scalar(255,255,255));
		for (int i = 0; i < faces.size(); i++) {
			rectangle(yComp, Point(faces[i].x, faces[i].y),
					Point(faces[i].x + faces[i].width,
							faces[i].y + faces[i].height), 255, 5);
			circle(yComp,
					Point(faces[i].x + faces[i].width / 2,
							faces[i].y + faces[i].height / 2), 3,
					Scalar(255, 255, 255), -1);

		}
	}

	if(ubodys.size()){
		ssUbodyRectSize << "X:" << ubodys[0].width << "Y:" << ubodys[0].height;
		putText(yComp,ssUbodyRectSize.str(),Point((ubodys[0].x)-20,(ubodys[0].y)+ubodys[0].height+20),0,0.5,Scalar(255,255,255));
		for (int i = 0; i < ubodys.size(); i++) {
			rectangle(yComp, Point(ubodys[i].x, ubodys[i].y),
					Point(ubodys[i].x + ubodys[i].width,
							ubodys[i].y + ubodys[i].height), 255, 5);
			circle(yComp,
					Point(ubodys[i].x + ubodys[i].width / 2,
							ubodys[i].y + ubodys[i].height / 2), 3,
					Scalar(255, 255, 255), -1);

		}
	}
	if(fbodys.size()){
		ssFbodyRectSize << "X:" << fbodys[0].width << "Y:" << fbodys[0].height;
		putText(yComp,ssFbodyRectSize.str(),Point((fbodys[0].x)-20,(fbodys[0].y)-20),0,0.5,Scalar(255,255,255));
		for (int i = 0; i < fbodys.size(); i++) {
			rectangle(yComp, Point(fbodys[i].x, fbodys[i].y),
					Point(fbodys[i].x + fbodys[i].width,
							fbodys[i].y + fbodys[i].height), 255, 5);
			circle(yComp,
					Point(fbodys[i].x + fbodys[i].width / 2,
							fbodys[i].y + fbodys[i].height / 2), 3,
					Scalar(255, 255, 255), -1);

		}
	}
	//splitYUV[0] = yComp;
	//splitYUV[1] = uComp;
	//splitYUV[2] = vComp;

	//merge(splitYUV,yuvImage);
	//cvtColor(yuvImage,bgrImage,CV_YCrCb2BGR);
	if(faces.size())
		deviceManager->faceRectDetected = faces;	//検出された矩形情報をdeviceManagerに渡す
	if(ubodys.size())
		deviceManager->upperBodyRectDetected = ubodys;
	if(fbodys.size())
		deviceManager->fullBodyRectDetected =fbodys;
	imshow("frame",yComp);
	waitKey(1);
	return;
}
void imageProc2(uint8_t* frame,HOGDescriptor hog,BD_MANAGER_t *deviceManager){

	unsigned int height = 368;
	unsigned int width = 640;
	unsigned int yHeight = 368;
	unsigned int yWidth = 672;
	unsigned int index = 0;
	unsigned int printNum = 0;
	Mat bgrImage(height,width,CV_8UC3);	//mat初期化
	Mat yComp(height,width,CV_8UC1);
	Mat uComp(height,width,CV_8UC1);
	Mat vComp(height,width,CV_8UC1);
	Mat print = Mat::zeros(Size(500,500),CV_8UC1);
	Mat plot = Mat::zeros(Size(800,500),CV_8UC1);
	Mat yuvImage(height+height/2,width,CV_8UC1,frame); //Mat yuvImageをframeで初期化
	Mat hsvImage(height,width,CV_8UC3);
	Mat binary(height,width,CV_8UC1,Scalar(255));
	Mat channels[3];
	Mat labelImg;
	Mat blur;
	Mat gray;
	Mat cutImage;
	Mat cutFace;
	Mat coordinate;
	Mat dst(height,width,CV_8UC3,Scalar(0));
	Mat output(height,width,CV_16UC1,Scalar(0));
	vector<Mat> splitYUV(3);
	stringstream ssPrint[17];
	stringstream ssPID[12];
	vector<vector<int> > stats;
	vector<vector<double> > centroids;
	PCA pca;
	int nLab;
	int count = 0;
	int coordinateNum = 0;
	int key = 0;
	stringstream ssFaceRectSize;

	deviceManager->cameraCount++;
	cvtColor(yuvImage,bgrImage,CV_YUV420p2RGB); //yuvをbgrに変換
	if(deviceManager->cameraCount == 150){
		imwrite("color.jpg",bgrImage);
	}

	bilateralFilter(bgrImage,blur,-1,50,2);	//2が正常に作動するギリギリ
	cvtColor(blur,gray,CV_BGR2GRAY);	//gray画像作成
	//medianBlur(bgrImage,blur,3);
	cvtColor(blur,hsvImage,CV_BGR2HSV); //bgrをhsvに変換
	//split(hsvImage,channels);
	//hsvで二値化
	for(int i = 0;i < height * width;i++){
		if(hsvImage.data[i*3] >= 20 || hsvImage.data[i*3+1] <= 180){
		binary.data[i] = 0;
		}
	}
	/*if(deviceManager->cameraCount == 150){
		imwrite("hsv.jpg",hsvImage);
		imwrite("binary_noisiy.jpg",binary);
		imwrite("blur.jpg",blur);
	}*/
    labeling(binary,output,dst,deviceManager,300);
	/*if(deviceManager->cameraCount == 150){
		imwrite("binary_clear.jpg",binary);
		imwrite("label.jpg",dst);
		deviceManager->cameraCount = 0;
	}*/

    //binary画像のregion部分切り出し
	if (deviceManager->stats.size() > 1) {
		//regionのピクセル数分coordinate確保
		coordinate = Mat(deviceManager->stats[1][CC_STAT_AREA],2,CV_64F);
		cutImage = Mat(output,
				Rect(deviceManager->stats[1][CC_STAT_LEFT],
						deviceManager->stats[1][CC_STAT_TOP],
						deviceManager->stats[1][CC_STAT_WIDTH],
						deviceManager->stats[1][CC_STAT_HEIGHT])).clone();
		//顔画像切り出し.TOP - HEIGHT /2 がマイナスにならないようにしている
		if(deviceManager->stats[1][CC_STAT_TOP] - deviceManager->stats[1][CC_STAT_HEIGHT] / 2 <= 0){
			cutFace = Mat(gray,
				Rect(deviceManager->stats[1][CC_STAT_LEFT],
						0,
						deviceManager->stats[1][CC_STAT_WIDTH],
						deviceManager->stats[1][CC_STAT_HEIGHT] / 2)).clone();


		}else{
			cutFace = Mat(gray,
				Rect(deviceManager->stats[1][CC_STAT_LEFT],
						deviceManager->stats[1][CC_STAT_TOP]
								- deviceManager->stats[1][CC_STAT_HEIGHT] / 2,
						deviceManager->stats[1][CC_STAT_WIDTH],
						deviceManager->stats[1][CC_STAT_HEIGHT] / 2)).clone();


		}

		//顔検出
		deviceManager->faceCascade.detectMultiScale(cutFace,deviceManager->faceRectDetected,1.1,2,0|CASCADE_SCALE_IMAGE,Size(30,30));	//顔検出
		deviceManager->numOfFace = deviceManager->faceRectDetected.size();

		//検出された矩形データの表示*3
		if (deviceManager->numOfFace != 0) {
			ssFaceRectSize << "X:" << deviceManager->faceRectDetected[0].width
					<< "Y:" << deviceManager->faceRectDetected[0].height;
			putText(cutFace, ssFaceRectSize.str(),
					Point((deviceManager->faceRectDetected[0].x) - 20,
							(deviceManager->faceRectDetected[0].y) - 20), 0,
					0.5, Scalar(255, 255, 255));
			for (int i = 0; i < deviceManager->numOfFace; i++) {
				rectangle(cutFace,
						Point(deviceManager->faceRectDetected[i].x,
								deviceManager->faceRectDetected[i].y),
						Point(
								deviceManager->faceRectDetected[i].x
										+ deviceManager->faceRectDetected[i].width,
								deviceManager->faceRectDetected[i].y
										+ deviceManager->faceRectDetected[i].height),
						255, 5);
				circle(cutFace,
						Point(
								deviceManager->faceRectDetected[i].x
										+ deviceManager->faceRectDetected[i].width
												/ 2,
								deviceManager->faceRectDetected[i].y
										+ deviceManager->faceRectDetected[i].height
												/ 2), 3, Scalar(255, 255, 255),
						-1);

			}
		}
		for (int i = 0; i < deviceManager->stats[1][CC_STAT_HEIGHT]; i++) {
			int *cutImageP = cutImage.ptr<int>(i);
			for (int j = 0; j < deviceManager->stats[1][CC_STAT_WIDTH]; j++) {
				if (cutImageP[j] == 1) {

					coordinate.at<double>(coordinateNum, 0) = i;
					coordinate.at<double>(coordinateNum, 1) = j;

					coordinateNum++;

				}
			}
		}
		deviceManager->coordinatenum = coordinateNum;
		ssPrint[0] << "Area - coordNum:" << deviceManager->stats[1][CC_STAT_AREA] - deviceManager->coordinatenum;

		//主成分分析実行
		pca(coordinate,Mat(),CV_PCA_DATA_AS_ROW,2);
		//過去にrocとったなら
		if(deviceManager->ROCFlag){
			//各データ引き継ぎ部
			deviceManager->pastFEV = deviceManager->firstEV;
			deviceManager->Eppe = deviceManager->Epe;
			deviceManager->Epe = deviceManager->Ece;	//1フレーム前の第一固有値の偏差を代入

			deviceManager->pastX = deviceManager->currentX;
			deviceManager->Eppx = deviceManager->Epx;
			deviceManager->Epx = deviceManager->Ecx;

			deviceManager->pastY = deviceManager->currentY;
			deviceManager->Eppy = deviceManager->Epy;
			deviceManager->Epy = deviceManager->Ecy;

			deviceManager->Epr = deviceManager->Ecr;

		}else{
			//各データリセット部
			deviceManager->pastFEV = 0.0;
			deviceManager->Epe = 0.0;
			deviceManager->Eppe = 0.0;

			deviceManager->pastX = 0.0;	//目標値が0.0なので初期値が0.0ではダメかもしれない
			deviceManager->Eppx = 0.0;
			deviceManager->Epx = 0.0;

			deviceManager->pastY = 0.0;
			deviceManager->Eppy = 0.0;
			deviceManager->Epy = 0.0;

			deviceManager->Epr = 0;
		}
		//各データ更新部
		deviceManager->firstEV = pca.eigenvalues.at<double>(0);
		deviceManager->secondEV = pca.eigenvalues.at<double>(1);
		deviceManager->Ece = 1000.0 - deviceManager->firstEV;	//現在の第一固有値の偏差
		deviceManager->Ese += deviceManager->Ece;	//第一固有値の累計偏差

		deviceManager->currentX = pixToDig(deviceManager->stats[1][CENTER_X]);
		deviceManager->Ecx = deviceManager->currentX;

		deviceManager->currentY = deviceManager->stats[1][CENTER_Y];
		deviceManager->Ecy = 184 - deviceManager->currentY;	//184は画像の中心y座標 他の2つと偏差の計算が逆
		deviceManager->Ecr = abs(FRONT_ROC - deviceManager->currentROC);

		//deviceManager->differenceROC = 0;	//differenceROCリセット
		//過去にrocとったなら
		if(deviceManager->ROCFlag){
			deviceManager->rocCount = deviceManager->rocCount % 6;
			//deviceManager->pastROC = deviceManager->currentROC;
			deviceManager->currentROC = deviceManager->secondEV / deviceManager->firstEV;

			//ROCの変化量を計算して代入
			deviceManager->rocArray[deviceManager->rocCount] = deviceManager->currentROC - deviceManager->pastROC;
			/*
			//ROCの変化量が正なら1負なら-1
			if((deviceManager->currentROC - deviceManager->pastROC) > 0){
				deviceManager->rocArray[deviceManager->rocCount] = 1.0;
				//差が負(ROC減少)
			}else{
				deviceManager->rocArray[deviceManager->rocCount] = -1.0;
			}
			*/
			//ここからrollControlに移動した

			/*
			//difference集計
			for(int i = 0;i < 6;i++){
				deviceManager->differenceROC += deviceManager->rocArray[i];
			}
			//difference判定
			if(deviceManager->differenceROC >= 2){
				deviceManager->differenceROC = 1.0;
			}else if(deviceManager->differenceROC <= -2){
				deviceManager->differenceROC = -1.0;
			}else{
				deviceManager->differenceROC = 0.0;
			}
			*/

			deviceManager->rocCount++;
			//とっていないなら
		}else{
			deviceManager->currentROC = deviceManager->secondEV / deviceManager->firstEV;
			deviceManager->ROCFlag = true;
		}

		ssPrint[1] << "ROC:" << deviceManager->currentROC;
		ssPrint[2] << "eigenVecX:" << pca.eigenvectors.at<double>(0,1) << " eigenVecY:" << pca.eigenvectors.at<double>(0,0);
		ssPrint[3] << "eigenValue1:" << pca.eigenvalues.at<double>(0) << " eigenValue2:" << pca.eigenvalues.at<double>(1);
		ssPrint[4] << "differenceOfROC:" << deviceManager->currentROC - deviceManager->pastROC;
		ssPrint[5] << "difference:"  << deviceManager->differenceROC;
		ssPrint[6] << "isNegative:"  << deviceManager->isNegative;
		ssPrint[7] << "yaw:"  << deviceManager->dataPCMD.yaw;
		ssPrint[8] << "pitch:"  << deviceManager->dataPCMD.pitch;
		ssPrint[9] << "roll:"  << deviceManager->dataPCMD.roll;
		ssPrint[10] << "gaz:"  << deviceManager->dataPCMD.gaz;
		ssPrint[11] << "currentRoll:"  << deviceManager->currentRoll;
		ssPrint[12] << "speedX:" << deviceManager->speedX;
		ssPrint[13] << "speedY:" << deviceManager->speedY;
		ssPrint[14] << "speedZ:" << deviceManager->speedZ;
		ssPrint[15] << "isFront:" << deviceManager->isFront;
		ssPrint[16] << "numOfFace:" << deviceManager->numOfFace;


		//PID制御実験
		ssPID[0] << "Mp:" << deviceManager->Mp << "Mpp:" << deviceManager->Mpp << "Mpd:" << deviceManager->Mpd;
		ssPID[1] << "Ece:" << deviceManager->Ece << " Epe:" << deviceManager->Epe << " Eppe:" << deviceManager->Eppe;
		ssPID[2] << "PD:" << (Kpp + deviceManager->Kppitch) * deviceManager->Ece + (Kpd + deviceManager->Kdp) * (deviceManager->Ece - deviceManager->Epe) << " P:" << (Kpp + deviceManager->Kppitch) * deviceManager->Ece << " D:" << (Kpd + deviceManager->Kdp) * (deviceManager->Ece - deviceManager->Epe);
		ssPID[9] << "Kpd:" << Kpd + deviceManager->Kdp;

		ssPID[3] << "My:" << deviceManager->My << "Mpy:" << deviceManager->Mpy << "Myd:" << deviceManager->Myd;
		ssPID[4] << "Ecx:" << deviceManager->Ecx << " Epx:" << deviceManager->Epx << " Eppx:" << deviceManager->Eppx;
		ssPID[5] << "PD:" << (Kyp + deviceManager->Kpy) * deviceManager->Ecx + (Kyd + deviceManager->Kdy) * (deviceManager->Ecx - deviceManager->Epx) << " P:" << Kyp * deviceManager->Ecx << " D:" << Kyd * ((deviceManager->Ecx - deviceManager->Epx));
		ssPID[10] << "Kyd:" << Kyd + deviceManager->Kdy;

		ssPID[6] << "Mg:" << deviceManager->Mg << "Mpg:" << deviceManager->Mpg << "Mgd:" << deviceManager->Mgd;
		ssPID[7] << "Ecy:" << deviceManager->Ecy << " Epy:" << deviceManager->Epy << " Eppy:" << deviceManager->Eppy;
		ssPID[8] << "PD:" << (Kgp + deviceManager->Kpg) * deviceManager->Ecy + (Kgd + deviceManager->Kdg) * (deviceManager->Ecy - deviceManager->Epy) << " P:" << Kgp * deviceManager->Ecy << " D:" << Kgd * (deviceManager->Ecy - deviceManager->Epy);
		ssPID[11] << "Kdg:" << Kgd + deviceManager->Kdg;
		//imshow("cutImage",cutImage);
	}else{	//人未検出
		//ROCFlagはドローンと人の距離が離れた場合にも呼ぶ必要がある
		deviceManager->ROCFlag = false;
		deviceManager->firstEV = 0.0;
		deviceManager->secondEV = 0.0;
		deviceManager->rocCount = 0;
		deviceManager->Ece = 0.0;
		deviceManager->Epe = 0.0;
		deviceManager->Eppe = 0.0;
		deviceManager->Ese = 0.0;
		deviceManager->pastFEV = 0.0;
		deviceManager->Mp = 0.0;
		deviceManager->Mpp = 0.0;
		deviceManager->Mpd = 0.0;

		deviceManager->pastX = 0.0;	//目標値が0.0なので初期値が0.0ではダメかもしれない
		deviceManager->Eppx = 0.0;
		deviceManager->Epx = 0.0;
		deviceManager->My = 0.0;
		deviceManager->Mpy = 0.0;
		deviceManager->Myd = 0.0;

		deviceManager->pastY = 0.0;
		deviceManager->Eppy = 0.0;
		deviceManager->Epy = 0.0;
		deviceManager->Mg = 0.0;
		deviceManager->Mpg = 0.0;
		deviceManager->Mgd = 0.0;

		deviceManager->isNegative = 0;

		deviceManager->Ecr = 0;
		deviceManager->Epr = 0;

		deviceManager->isFront = 0;

		for(int i = 0;i < 6;i++){
			deviceManager->rocArray[i] = 0;
		}
		ssPrint[0] << "Area - coordNum:" << 0;
		ssPrint[1] << "ROC:" << 0;
		ssPrint[2] << "eigenVecX:" << 0 << " eigenVecY:" << 0;
		ssPrint[3] << "eigenValue1:" << 0 << " eigenValue2:" << 0;
		ssPrint[4] << "differenceOfROC:" << 0;
		ssPrint[5] << "height:" << 0 << "width:" << 0;
		ssPrint[6] << "isNegative:"  << 0;
		ssPrint[7] << "yaw:"  << 0;
		ssPrint[8] << "pitch:"  << 0;
		ssPrint[9] << "roll:"  << 0;
		ssPrint[10] << "gaz:"  << 0;
		ssPrint[11] << "currentRoll:"  << 0;
		ssPrint[12] << "speedX:" << 0;
		ssPrint[13] << "speedY:" << 0;
		ssPrint[14] << "speedZ:" << 0;
		ssPrint[15] << "isFront:" << deviceManager->isFront;
		ssPrint[16] << "numOfFace:" << deviceManager->numOfFace;
		ssPID[0] << "Mp:" << 0 << "Mpp:" << 0 << "Mpd:" << 0;
		ssPID[1] << "Ece:" << 0 << " Epe:" << 0 << " Eppe:" << 0;
		ssPID[2] << "P:" << 0 << " I:" << 0 << " D:" << 0;
		ssPID[3] << "My:" << 0 << "Mpy:" << 0 << "Myd:" << 0;
		ssPID[4] << "Ecx:" << 0 << " Epx:" << 0 << " Eppx:" << 0;
		ssPID[5] << "P:" << 0 << " I:" << 0 << " D:" << 0;
		ssPID[6] << "Mg:" << 0 << "Mpg:" << 0 << "Mgd:" << 0;
		ssPID[7] << "Ecy:" << 0 << " Epy:" << 0 << " Eppy:" << 0;
		ssPID[8] << "P:" << 0 << " I:" << 0 << " D:" << 0;
		ssPID[9] << "Kpd:" << Kpd + deviceManager->Kdp;
		ssPID[10] << "Kyd:" << Kyd + deviceManager->Kdy;
		ssPID[11] << "Kdg:" << Kgd + deviceManager->Kdg;

	}
	/*for(int i = 0;i < yHeight*yWidth; i++){

		if((i%yWidth)+1 > width) continue;

		yComp.data[index] = frame->componentArray[0].data[i];
		//uComp.data[index] = frame->componentArray[1].data[i/2];
		//vComp.data[index] = frame->componentArray[2].data[i/2];
		index++;
	}*/

	//グラフ描画

	plotGraph(deviceManager);

	putText(print,ssPrint[0].str(),Point(0,10),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[1].str(),Point(0,30),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[2].str(),Point(0,50),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[3].str(),Point(0,70),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[4].str(),Point(0,90),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[5].str(),Point(0,110),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[6].str(),Point(0,130),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[7].str(),Point(0,150),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[8].str(),Point(0,170),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[9].str(),Point(0,190),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[10].str(),Point(0,210),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[11].str(),Point(0,230),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[12].str(),Point(0,250),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[13].str(),Point(0,270),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[14].str(),Point(0,290),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[15].str(),Point(0,310),0,0.5,Scalar(255,255,255));
	putText(print,ssPrint[16].str(),Point(0,330),0,0.5,Scalar(255,255,255));


	/*
	//表示部
	for(int i = 0,j = 0;i < 12,j <= 300;i++,j+=20){
		if(i % 3 == 0){
			j += 20;
		}
		printNum = 130 + j;

		putText(print,ssPID[i].str(),Point(0,printNum),0,0.5,Scalar(255,255,255));
	}
	printNum = 0;
	*/
	//deviceManager->faceCascade.detectMultiScale(yComp,faces,1.1,2,0|CASCADE_SCALE_IMAGE,Size(30,30));
	imshow("binary",binary);
	imshow("frame",print);
	imshow("face",cutFace);
	//imshow("bgrImage",bgrImage);
	//imshow("blur",blur);
	imshow("label",dst);
	//imshow("gray",gray);
	//imshow("hsvImage",hsvImage);
	//imshow("h",channels[0]);
	//imshow("s",channels[1]);
	//imshow("v",channels[2]);
	key = waitKey(1);

	switch(key){
	//キー1
	case 1048625:
		deviceManager->plotType = 0;
		break;
	//キー2
	case 1048626:
		deviceManager->plotType = 1;
		break;
	//キー3
	case 1048627:
		deviceManager->plotType = 2;
		break;
	default:
		break;
	}

	//ゲイン微調整
	switch(deviceManager->plotType){
	//キー0
	case 0:
		//キー↑
		if(key == 1113938){
			deviceManager->Kdy = deviceManager->Kdy + 0.01;
			//キー↓
		}else if(key == 1113940){
			deviceManager->Kdy = deviceManager->Kdy - 0.01;
		}
		break;
	//キー1
	case 1:
		if(key == 1113938){
			//deviceManager->Kppitch = deviceManager->Kppitch + 0.01;
			deviceManager->Kdp = deviceManager->Kdp + 0.01;
		}else if(key == 1113940){
			//deviceManager->Kppitch = deviceManager->Kppitch - 0.01;
			deviceManager->Kdp = deviceManager->Kdp - 0.01;
		}
		break;
	//キー2
	case 2:
		if(key == 1113938){
			deviceManager->Kdg = deviceManager->Kdg + 0.01;
		}else if(key == 1113940){
			deviceManager->Kdg = deviceManager->Kdg - 0.01;
		}
		break;
	default:
		break;

	}

	return;
}

void plotGraph(BD_MANAGER_t *deviceManager){
	int j;
	//グラフ表示用データ配列更新
	if(deviceManager->time < 300){
		deviceManager->contVariable[CT_yaw][deviceManager->time] = deviceManager->stats[1][CENTER_X];	//グラフ表示用のyaw(0)行time列に人のX座標代入
		deviceManager->contVariable[CT_gaz][deviceManager->time] = deviceManager->stats[1][CENTER_Y];
		deviceManager->contVariable[CT_pitch][deviceManager->time] = deviceManager->firstEV;
	}else{
		for(int i = 0;i < 299;i++){
			deviceManager->contVariable[CT_yaw][i] = deviceManager->contVariable[CT_yaw][i+1];
			deviceManager->contVariable[CT_gaz][i] = deviceManager->contVariable[CT_gaz][i+1];
			deviceManager->contVariable[CT_pitch][i] = deviceManager->contVariable[CT_pitch][i+1];
		}
		deviceManager->contVariable[CT_yaw][deviceManager->time-1] = deviceManager->stats[1][CENTER_X];
		deviceManager->contVariable[CT_gaz][deviceManager->time-1] = deviceManager->stats[1][CENTER_Y];
		deviceManager->contVariable[CT_pitch][deviceManager->time-1] = deviceManager->firstEV;
	}

	switch(deviceManager->plotType){
	case 0:
		//描画設定
		fprintf(deviceManager->gp,"set xrange [0:300]\n");	//範囲の指定
		fprintf(deviceManager->gp,"set yrange [0:640]\n");
		fprintf(deviceManager->gp,"set xlabel \"time\"\n");	//ラベル表示
		fprintf(deviceManager->gp,"set ylabel \"pixelX\"\n");
		fprintf(deviceManager->gp,"set xtics 0,30,300\n");	//x軸メモリ設定
		fprintf(deviceManager->gp,"set grid xtics\n");	//x軸グリッド線設定
		fprintf(deviceManager->gp,"set style arrow 1 nohead\n");	//矢印の頭消す設定
		fprintf(deviceManager->gp,"set arrow 1 from 0,320 to 300,320 arrowstyle 1\n");	//矢印の描画範囲設定
		fprintf(deviceManager->gp,"set time\n");	//現在時刻表示

		//描画データ入力
		fprintf(deviceManager->gp,"plot '-' with lines linetype 1\n");
		j = 0;
		do{
			fprintf(deviceManager->gp,"%f\t%f\n",(double)j,deviceManager->contVariable[CT_yaw][j]);
			j++;
		}while(j < deviceManager->time);
		//描画実行
		fprintf(deviceManager->gp,"e\n");

		break;

	case 1:

		fprintf(deviceManager->gp,"set xrange [0:300]\n");	//範囲の指定
		fprintf(deviceManager->gp,"set yrange [0:2000]\n");
		fprintf(deviceManager->gp,"set xlabel \"time\"\n");	//ラベル表示
		fprintf(deviceManager->gp,"set ylabel \"eigenvalue\"\n");
		fprintf(deviceManager->gp,"set xtics 0,30,300\n");
		fprintf(deviceManager->gp,"set grid xtics\n");
		fprintf(deviceManager->gp,"set style arrow 1 nohead\n");
		fprintf(deviceManager->gp,"set arrow 1 from 0,1000 to 300,1000 arrowstyle 1\n");
		fprintf(deviceManager->gp,"set time\n");


		fprintf(deviceManager->gp,"plot '-' with lines linetype 1\n");
		j = 0;
		do{
			fprintf(deviceManager->gp,"%f\t%f\n",(double)j,deviceManager->contVariable[CT_pitch][j]);
			j++;
		}while(j < deviceManager->time);
		//描画実行
		fprintf(deviceManager->gp,"e\n");

		break;

	case 2:

		fprintf(deviceManager->gp,"set xrange [0:300]\n");	//範囲の指定
		fprintf(deviceManager->gp,"set yrange [0:368]\n");
		fprintf(deviceManager->gp,"set xlabel \"time\"\n");	//ラベル表示
		fprintf(deviceManager->gp,"set ylabel \"pixelY\"\n");
		fprintf(deviceManager->gp,"set xtics 0,30,300\n");
		fprintf(deviceManager->gp,"set grid xtics\n");
		fprintf(deviceManager->gp,"set style arrow 1 nohead\n");
		fprintf(deviceManager->gp,"set arrow 1 from 0,184 to 300,184 arrowstyle 1\n");
		fprintf(deviceManager->gp,"set time\n");


		fprintf(deviceManager->gp,"plot '-' with lines linetype 1\n");
		j = 0;
		do{
			fprintf(deviceManager->gp,"%f\t%f\n",(double)j,deviceManager->contVariable[CT_gaz][j]);
			j++;
		}while(j < deviceManager->time);
		//描画実行
		fprintf(deviceManager->gp,"e\n");
		break;

	default:
		break;

	}
	//時間更新
	if(deviceManager->time < 300){
		deviceManager->time++;
	}

	return;
}

/************************** Autonomous flying part **************************/
void autonomousFlying (eIHM_INPUT_EVENT event,BD_MANAGER_t *deviceManager,Mat infoWindow){
	 // Manage IHM input events
	stringstream velSS,coordDetectedSS,eventSS,print[4];
	eventSS << "event:" << event;
	vector<Point> coordDetected;
	//ここから
	double diff;
	int vel;
	//ここまで
//	int rectSize = deviceManager->fullBodyRectDetected.size();
//	if (rectSize != 0) {
//		coordDetected.resize(deviceManager->fullBodyRectDetected.size());
//		for (int i = 0; i < deviceManager->fullBodyRectDetected.size(); i++) {
//			coordDetected[i].x = deviceManager->fullBodyRectDetected[i].x
//					+ deviceManager->fullBodyRectDetected[i].width / 2;
//			coordDetected[i].y = deviceManager->fullBodyRectDetected[i].y
//					+ deviceManager->fullBodyRectDetected[i].height / 2;
//		}
//
//		/*
//		 coordDetectedSS << "x:" << deviceManager->fullBodyRectDetected[0].x << " y:"
//				<< deviceManager->fullBodyRectDetected[0].y;
//		putText(infoWindow, coordDetectedSS.str(), Point(200, 100), FONT_ITALIC,
//				1.2, Scalar(255, 200, 100), 2, CV_AA);
//				*/
//		/*
//		for (int i = 0; i < rectSize; i++) {
//			circle(infoWindow,
//					Point(deviceManager->fullBodyRectDetected[i].x,
//							deviceManager->fullBodyRectDetected[i].y), 3,
//					Scalar(255, 255, 255), -1);
//			circle(infoWindow, Point(coordDetected[i].x, coordDetected[i].y), 3,
//					Scalar(255, 255, 255), -1);
//			circle(infoWindow,
//					Point(
//							deviceManager->fullBodyRectDetected[i].x
//									+ deviceManager->fullBodyRectDetected[i].width,
//							deviceManager->fullBodyRectDetected[i].y
//									+ deviceManager->fullBodyRectDetected[i].height), 3,
//					Scalar(255, 255, 255), -1);
//		}
//		*/
//	//ここから
//	//diff = pixToDig(coordDetected[0].x);
//	//vel = 100.0 * (diff / pixToDig(640));
//	//if(fabs(diff) < pixToDig(340)){
//	//	vel = 0;
//	//}
//	diff = (238.5 - (double)deviceManager->fullBodyRectDetected[0].height)/28.5;
//	if(fabs(diff) > 1.0) diff = diff/fabs(diff);
//	if(fabs(238.5 - (double)deviceManager->fullBodyRectDetected[0].height) < 10.0) diff = 0.0;
//	vel = 10.0 * diff;
//	velSS << "velocity" <<vel;
//	putText(infoWindow,velSS.str(),Point(100,100),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
//	//ここまで
//	}

	putText(infoWindow,"autonomous flying",Point(184,320),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
	putText(infoWindow,eventSS.str(),Point(0,30),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
	line(infoWindow,Point(320,0),Point(320,368),Scalar(255,255,255),2);
	line(infoWindow,Point(310,0),Point(310,368),Scalar(255,255,255),2);
	line(infoWindow,Point(330,0),Point(330,368),Scalar(255,255,255),2);
	rectangle(infoWindow,Point(310,174),Point(330,194),255,5);

	switch (event) {
	case IHM_INPUT_EVENT_EXIT:
		gIHMRun = 0;
		break;
	case IHM_INPUT_EVENT_EMERGENCY:
		if (deviceManager != NULL) {
			sendEmergency(deviceManager);
		}
		break;
	case IHM_INPUT_EVENT_TAKEOFF_LANDING:
		if (deviceManager != NULL) {
			if (deviceManager->flyingState
					== ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED) {
				sendTakeoff(deviceManager);
			} else if ((deviceManager->flyingState
					== ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING)
					|| (deviceManager->flyingState
							== ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING)) {
				sendLanding(deviceManager);
			}

		}
		break;
	case IHM_INPUT_EVENT_RIGHT:
		if (deviceManager != NULL) {
			deviceManager->dataPCMD.flag = 1;
			deviceManager->dataPCMD.roll = 10;
		}
		break;
	case IHM_INPUT_EVENT_LEFT:
		if (deviceManager != NULL) {
			deviceManager->dataPCMD.flag = 1;
			deviceManager->dataPCMD.roll = -10;
		}
		break;

	case IHM_INPUT_EVENT_NONE:
		deviceManager->dataPCMD.flag = 0;
		deviceManager->dataPCMD.roll = 0;
		break;
	default:
		break;
	}
	if (deviceManager != NULL) {

		if (deviceManager->stats.size() > 1) {
			//cameraControl(deviceManager,coordDetected);

			if (abs(deviceManager->Ece) <= 250 && abs(deviceManager->Ecx) <= 10
					&& abs(deviceManager->Ecy) < 30
					&& abs(deviceManager->Ecr) < 0.01 && deviceManager->numOfFace != 0) {

				deviceManager->isFront = 1;
				deviceManager->dataPCMD.flag = 0;
				deviceManager->dataPCMD.roll = 0;
				deviceManager->dataPCMD.pitch = 0;
				deviceManager->dataPCMD.yaw = 0;
				deviceManager->dataPCMD.gaz = 0;
				deviceManager->dataCam.pan = 0;
				deviceManager->dataCam.tilt = 0;
				deviceManager->currentRoll = 0;

			} else {
				deviceManager->dataPCMD.flag = 1;
				deviceManager->isFront = 0;
				directionControl(deviceManager);
				distanceControl(deviceManager);
				altitudeControl(deviceManager);
				rollControl(deviceManager);
			}

//				if(coordDetected[0].x > 350){
//					putText(infoWindow,"30",Point(200,30),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
//				}else if(coordDetected[0].x < 300){
//					putText(infoWindow,"-30",Point(200,30),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
//				}else{
//					putText(infoWindow,"0",Point(200,30),FONT_ITALIC,1.2,Scalar(255,200,100),2,CV_AA);
//				}ssPrint[6] < "pitch:" << deviceManager->dataPCMD.pitch << " yaw:" << deviceManager->dataPCMD.yaw;
      		print[0] << "pitch:" << deviceManager->dataPCMD.pitch << "yaw:" << deviceManager->dataPCMD.yaw;
      		print[1] << "gaz:" << deviceManager->dataPCMD.gaz;
      		print[2] << "roll:" << deviceManager->dataPCMD.roll;
      		print[3] << "rollControllFag:" << deviceManager->rollControllFlag;
      		//deviceManager->ROCLog << (int)deviceManager->differenceROC << endl;
      		//rocLog << (int)deviceManager->differenceROC << endl;

		} else {
			deviceManager->isFront = 0;
			deviceManager->dataPCMD.flag = 0;
			deviceManager->dataPCMD.roll = 0;
			deviceManager->dataPCMD.pitch = 0;
			deviceManager->dataPCMD.yaw = 0;
			deviceManager->dataPCMD.gaz = 0;
			deviceManager->dataCam.pan = 0;
			deviceManager->dataCam.tilt = 0;
			deviceManager->currentRoll = 0;
		    print[0] << "pitch:" << deviceManager->dataPCMD.pitch << "yaw:" << deviceManager->dataPCMD.yaw;
      		print[1] << "gaz:" << 0;
      		print[2] << "roll:" << 0;
      		print[3] << "rollControllFag:" << 0;

		}
	    putText(infoWindow,print[0].str(),Point(0,100),0,0.5,Scalar(255,255,255));
	    putText(infoWindow,print[1].str(),Point(0,120),0,0.5,Scalar(255,255,255));
	    putText(infoWindow,print[2].str(),Point(0,140),0,0.5,Scalar(255,255,255));
	    putText(infoWindow,print[3].str(),Point(0,160),0,0.5,Scalar(255,255,255));

	}else{
		deviceManager->dataPCMD.flag = 0;
		deviceManager->dataPCMD.roll = 0;
		deviceManager->dataPCMD.pitch = 0;
		deviceManager->dataPCMD.yaw = 0;
		deviceManager->dataPCMD.gaz = 0;
		deviceManager->dataCam.pan = 0;
		deviceManager->dataCam.tilt = 0;
		deviceManager->currentRoll = 0;
	}
	//保険のために動かないようにする　動かすときは消す
	/*
	deviceManager->dataPCMD.flag = 0;
	deviceManager->dataPCMD.roll = 0;
	deviceManager->dataPCMD.pitch = 0;
	deviceManager->dataPCMD.yaw = 0;
	deviceManager->dataPCMD.gaz = 0;
	deviceManager->dataCam.pan = 0;
	deviceManager->dataCam.tilt = 0;
*/
}

void cameraControl(BD_MANAGER_t *deviceManager,vector<Point> coordDetected){
	if (coordDetected[0].x > 330) {
						deviceManager->dataCam.pan += 1;
					} else if (coordDetected[0].x < 310) {
						deviceManager->dataCam.pan -= 1;
					}
					if (coordDetected[0].y < 174) {
						deviceManager->dataCam.tilt += 1;
					} else if (coordDetected[0].y > 194) {
						deviceManager->dataCam.tilt -= 1;
					}
					if (deviceManager->dataCam.pan > 80) {
						deviceManager->dataCam.pan = 80;
					}
					if (deviceManager->dataCam.pan < -80) {
						deviceManager->dataCam.pan = -80;
					}
					if (deviceManager->dataCam.tilt > 80) {
						deviceManager->dataCam.pan = 80;
					}
					if (deviceManager->dataCam.tilt < -80) {
						deviceManager->dataCam.pan = -80;
					}
}
void directionControl(BD_MANAGER_t *deviceManager){
	double diff;
	int vel;

	/*
	diff = pixToDig(deviceManager->stats[1][CENTER_X]);
	vel = 70 * (diff / pixToDig(640)); //100はやり過ぎ？ 最大が1になるようにpixToDig(640)で割っている
	if(fabs(diff) < pixToDig(340)){	//中心からのピクセル距離が20以下なら速度を0に
		vel = 0;
	}
*/

	//deviceManager->Myd = (Kyp + deviceManager->Kpy) * (deviceManager->Ecx - deviceManager->Epx) + Kyi * deviceManager->Ecy + Kyd * ((deviceManager->Ecy - deviceManager->Epy) - (deviceManager->Epy - deviceManager->Eppy));
	deviceManager->Myd = (Kyp + deviceManager->Kpy) * (deviceManager->Ecx - deviceManager->Epx);
	//deviceManager->My = deviceManager->Mpy + deviceManager->Myd;
	deviceManager->My = (Kyp + deviceManager->Kpy) * deviceManager->Ecx + (Kyd + deviceManager->Kdy) * (deviceManager->Ecx - deviceManager->Epx);
	deviceManager->dataPCMD.yaw = deviceManager->My;
	deviceManager->Mpy = deviceManager->My;

}


void distanceControl(BD_MANAGER_t *deviceManager){
	//この関数の各数値は計測し直す必要がある
	//1.5mと2.5mの時の第一主成分も計測し、2mの時の第一主成分との差をとって28.5と置き換える
	//1000は人とdroneの距離が2mの時のfirtsEVの値　目標値
	//2mより近づくと反対に制御入る　2m手前にはtilt角0区間がある
	double diff, vel,kp,ki,kd;
	/*if ((1000.0 - (double) deviceManager->firstEV) < 100.0 && (1000.0 - (float) deviceManager->firstEV) >= 0.0) {
		diff = 0.0;
	}
	else{
			}*/
	//vel = kp * deviceManager->Ece + ki * deviceManager->Esie + kd * (deviceManager->Ece - deviceManager->Epe);


	/*
	diff = (1000.0 - (double) deviceManager->firstEV) / 380.0; //最も離れた時のfirtsEVが380なのでその数字で割っている
	if (fabs(diff) > 1.0) diff = diff / fabs(diff);

	vel = 10.0 * (diff/fabs(diff));
	deviceManager->dataPCMD.pitch = (int)vel;
	*/
	//deviceManager->Mpd = (Kpp + deviceManager->Kppitch) * (deviceManager->Ece - deviceManager->Epe) + Kpi * deviceManager->Ece + Kpd * ((deviceManager->Ece - deviceManager->Epe) - (deviceManager->Epe - deviceManager->Eppe));
	deviceManager->Mpd = (Kpp + deviceManager->Kppitch) * (deviceManager->Ece - deviceManager->Epe);
	//deviceManager->Mp = deviceManager->Mpp + deviceManager->Mpd;
	deviceManager->Mp = (Kpp + deviceManager->Kppitch) * deviceManager->Ece + (Kpd + deviceManager->Kdp) * (deviceManager->Ece - deviceManager->Epe);
	deviceManager->dataPCMD.pitch = deviceManager->Mp;
	deviceManager->Mpp = deviceManager->Mp;

}

void rollControl(BD_MANAGER_t *deviceManager) {
	//この関数はpixelサイズが一定以上の時呼ばれる
	//第一主成分が一定以上の時
	//まず顔が検出できているかできていないかを判断する
	//ここのroll値はすべて最大角の割合
	//800は仮　十分に近づいているか？

	if (deviceManager->firstEV > 180 && deviceManager->firstEV < 2200) {

		//顔検出判定
		if (deviceManager->numOfFace != 0) {
			deviceManager->findFace = 1;
		} else {
			deviceManager->findFace = 0;
		}

		deviceManager->rollControllFlag = true;

		//直前フレームの角度やピクセル情報を保持しているかチェック　していなければ代入
		if (deviceManager->rollFlag == 0) {

			//deviceManager->Ecr = abs(FRONT_ROC - deviceManager->currentROC);
			//deviceManager->Epr = deviceManager->Ecr;
			//currentRollはrollの割合
			//たまに0.44を超えるので絶対値を取る
			deviceManager->currentRoll = deviceManager->Ecr * Krp;

			deviceManager->dataPCMD.roll = deviceManager->currentRoll;//とりあえず右に傾ける
			deviceManager->pastRoll = deviceManager->currentRoll;
			deviceManager->rollFlag = 1;
			return;
		} else {	//直前の情報を保持していたら

			//基本differenceは1
			//deviceManager->Ecr = abs(FRONT_ROC - deviceManager->currentROC);
			deviceManager->differenceROC = 1.0;

			//前のフレームの静止カウントが1だった場合
			if (deviceManager->isStop == 1) {
				deviceManager->isStop = 0;
				//セーブされたrollを実行
				deviceManager->dataPCMD.roll = deviceManager->rollSeved;
				deviceManager->pastRoll = deviceManager->rollSeved;
			} else {
				//6フレームに一度向き判定
				if (deviceManager->isGetDifference == 0) {
					//差分負状態が5回(1秒)連続したら反転
					if (deviceManager->currentROC - deviceManager->pastROC
							<= 0) {
						//負カウント
						deviceManager->isNegative++;
						if (deviceManager->isNegative >= ROLL_COUNT) {
							deviceManager->isNegative = 0;
							//反転
							deviceManager->differenceROC = -1.0;
							//静止フラグオン
							deviceManager->isStop = STOP_COUNT;
							deviceManager->rollSeved = deviceManager->pastRoll / abs(deviceManager->pastRoll) * (float)deviceManager->differenceROC;
							deviceManager->rollSeved = deviceManager->rollSeved * deviceManager->Ecr * Krp;
						}
					} else {
						if (deviceManager->isNegative > 0) {
							//正カウント
							deviceManager->isNegative--;
						}
					}
					deviceManager->pastROC = deviceManager->currentROC;

					if(deviceManager->isStop != 0){
						deviceManager->isStop--;
					}
				}
				deviceManager->isGetDifference++;
				deviceManager->isGetDifference = deviceManager->isGetDifference
						% 6;


				//前フレームの符号だけを取り出すために絶対値を取る
				//符号決定
				deviceManager->currentRoll = deviceManager->pastRoll
						/ abs(deviceManager->pastRoll)
						* (float) deviceManager->differenceROC;
				//値決定
				deviceManager->currentRoll = deviceManager->currentRoll
						* deviceManager->Ecr * Krp;

				if (deviceManager->isStop == 0) {

					//値送信
					deviceManager->dataPCMD.roll = deviceManager->currentRoll;

				} else {
					//その場で静止させる
					deviceManager->dataPCMD.flag = 0;
					deviceManager->dataPCMD.roll = 0;
					deviceManager->dataPCMD.pitch = 0;
					deviceManager->dataPCMD.yaw = 0;
					deviceManager->dataPCMD.gaz = 0;
					deviceManager->isNegative = 0;
				}

				//引き継ぎ
				deviceManager->pastRoll = deviceManager->currentRoll;
				//deviceManager->Epr = deviceManager->Ecr;
			}

		}

	} else {
		//rollをリセットするかしないか
		//rollFlagをリセットするかしないか
		deviceManager->rollControllFlag = false;
		deviceManager->dataPCMD.roll = 0;
	}

	//to do
	//rollの角度をPPHの目標値との差に比例させる
	//PPHの目標値とある程度近くなったら顔検出カウントを開始する
	//顔検出カウントが0になったら反対側に回りこむ(回り込みフェーズフラグを作成する必要あり)
	//回り込む場合は一度PPHをさげて(側面PPH要測定)からまた目標値まで上げる必要がある
	//上がりきったら回りこみフェーズフラグをオフにする
	//符号だけ直前PPHとの差で判定して、数値は目標PPHとの誤差と、最大速度で決める
	//二値化する前に画像をぼやかす
	//
	//rollの更新量はpixPerHeightの目標値との差（要計測）に比例させる必要がある
	//できていなければ、PPHが一定以上か以下かを判断する
	//一定以下なら、目標値に近づくようにyawを右方向に傾ける
	//一定以上なら、カウントを始める
}

void altitudeControl(BD_MANAGER_t *deviceManager){
/*
	if(deviceManager->stats[1][CENTER_Y] < 164){
		deviceManager->dataPCMD.gaz = 10; //max vertical speed(1m/s)の割合
	}else if(deviceManager->stats[1][CENTER_Y] > 204){
		deviceManager->dataPCMD.gaz = -10;
	}else{
		deviceManager->dataPCMD.gaz = 0;
	}
	*/

	//更新量計算
	//deviceManager->Mgd = (Kgp + deviceManager->Kpg) * (deviceManager->Ecx - deviceManager->Epx) + Kgi * deviceManager->Ecx + Kgd * ((deviceManager->Ecx - deviceManager->Epx) - (deviceManager->Epx - deviceManager->Eppx));
	deviceManager->Mgd = (Kgp + deviceManager->Kpg) * (deviceManager->Ecy - deviceManager->Epy);
	//操作量計算
	//差分ではなく直接計算する
	deviceManager->Mg = (Kgp + deviceManager->Kpg) * deviceManager->Ecy + (Kgd + deviceManager->Kdg) * (deviceManager->Ecy - deviceManager->Epy);
	//130超えると制御できなくなるので100で留めるストッパー
	if(deviceManager->Mg > 100){
		deviceManager->Mg = 100;
	}
	//操作量送信
	deviceManager->dataPCMD.gaz = deviceManager->Mg;
	//1フレーム前の操作量に現在の操作量を代入
	deviceManager->Mpg = deviceManager->Mg;


}
//中心から何度ずれているのか求める
double pixToDig(const int pix){
	return ((double)pix-320.0)/(640.0/80.9946);
}

void labeling(const Mat input,Mat &output,Mat &dst,BD_MANAGER_t *deviceManager,const int minLabelSize){
	int nLab;
	vector<Vec3b> colors;
	Mat s,c;
	vector<int> itmp;
	vector<double> dtmp;
	int sCount = 0;
	//openingして領域つなげてclosingでノイズ除去
	morphologyEx(input,input,MORPH_OPEN,Mat(),Point(-1,-1),1);
	morphologyEx(input,input,MORPH_CLOSE,Mat(),Point(-1,-1),1);
	nLab = connectedComponentsWithStats(input,output,s,c,8,CV_32S); //ラベリング実行
	deviceManager->stats.clear();
	if (nLab > 1) {
		//regionの大きさが規定値以上なら追加
		for (int i = 0; i < nLab; i++) {
			int *param1 = s.ptr<int>(i);
			double *param2 = c.ptr<double>(i);
			if (param1[CC_STAT_AREA] >= minLabelSize) {
				//statsにpushするためのitmp作成
				for (int j = 0; j < 6; j++) {
					itmp.push_back(param1[j]);
				}
				//statsにitmpをpush
				deviceManager->stats.push_back(itmp);
				itmp.clear();	//重要　これがないとitmpの要素数がstatsの要素数を超える
				//centroidsにpushするためのdtmp作成
				//dtmp.push_back(param2[0]);
				//dtmp.push_back(param2[1]);
				//centroidsにdtmpをpush
				deviceManager->stats[sCount].push_back(
						static_cast<int>(param2[0]));
				deviceManager->stats[sCount].push_back(
						static_cast<int>(param2[1]));
				//古いラベルを代入
				deviceManager->stats[sCount].push_back(i);
				//dtmp.clear();  //重要　これがないとdtmpの要素数がcentroidsの要素数を超える
				sCount++;
			}

		}
		nLab = deviceManager->stats.size();	//新しいlabel数
		//regionの大きさ順にソート
		sort(deviceManager->stats.begin() + 1, deviceManager->stats.end(),
				areaComparator<int>);
		//ラベル画像をソート結果によって書き直し　AREA一定以下のlabelは塗りつぶされ、label0(背景)の大きさが増える
		for (int i = 0; i < output.rows; i++) {
			int *outputP = output.ptr<int>(i);
			for (int j = 0; j < output.cols; j++) {
				for (int k = 0; k < nLab + 1; k++) {
					if (k >= nLab) {
						outputP[j] = 0;
					} else if (outputP[j]
							== (int) deviceManager->stats[k][LABEL]) {
						outputP[j] = k;
						break;
					}
				}
			}
		}

		colors.resize(nLab);
		colors[0] = Vec3b(0, 0, 0);
		//ラベル色設定
		for (int i = 1; i < nLab; i++) {
			colors[i] = Vec3b(rand() & 255, rand() & 255, rand() & 255);
		}
		//ラベリング結果描画
		for (int i = 0; i < input.rows; i++) {
			int *lb = output.ptr<int>(i);
			Vec3b *pix = dst.ptr<Vec3b>(i);
			for (int j = 0; j < dst.cols; j++) {
				/*
				 if(lb[j] == 1){
				 pix[j] = Vec3b(255,0,0);
				 }else{
				 pix[j] = Vec3b(0,0,0);
				 }
				 */

				pix[j] = colors[lb[j]];
			}
		}
		//ROIの設定
		for (int i = 1; i < nLab; i++) {
			int x = deviceManager->stats[i][CC_STAT_LEFT];
			int y = deviceManager->stats[i][CC_STAT_TOP];
			int height = deviceManager->stats[i][CC_STAT_HEIGHT];
			int width = deviceManager->stats[i][CC_STAT_WIDTH];
			rectangle(dst, Rect(x, y, width, height), Scalar(0, 255, 0), 2);
		}
		//重心の出力
		for (int i = 1; i < nLab; i++) {
			int x = static_cast<int>(deviceManager->stats[i][CENTER_X]);
			int y = static_cast<int>(deviceManager->stats[i][CENTER_Y]);
			circle(dst, Point(x, y), 3, Scalar(0, 0, 255), -1);
		}
		//面積値の出力
		for (int i = 1; i < nLab; i++) {
			int x = deviceManager->stats[i][CC_STAT_LEFT];
			int y = deviceManager->stats[i][CC_STAT_TOP];
			stringstream num;
			num << "number:" << i << " area:"
					<< deviceManager->stats[i][CC_STAT_AREA];
			putText(dst, num.str(), Point(x + 5, y + 20), FONT_HERSHEY_COMPLEX,
					0.7, Scalar(0, 255, 255), 2);

		}
	}



}
void exePca(const Mat input,BD_MANAGER_t *deviceManager){
	const int dimension = 2;
	const int datanum = input.rows;
	PCA pca;
	pca(input,noArray(),0,2);
	deviceManager->currentROC = (float)pca.eigenvalues.at<double>(1) / (float)pca.eigenvalues.at<double>(0);
	for(int i = 0;i < 2;i++){
		for(int j = 0;j < 2;j++){
			deviceManager->eigenvectors[i][j] = pca.eigenvectors.at<double>(i,j);
		}
	}
}
template<class T>
bool areaComparator(const vector<T>& a,const vector<T>& b){
	return a[CC_STAT_AREA] > b[CC_STAT_AREA];
}
