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
#ifndef _SDK_EXAMPLE_BD_H_
#define _SDK_EXAMPLE_BD_H_


#include "ihm.h"
#include "DecoderManager.h"
#include <libARCommands/ARCommands.h>

using namespace std;

typedef struct
{
    int flag;
    int roll;
    int pitch;
    int yaw;
    int gaz;
}BD_PCMD_t;

typedef struct _ARDrone3CameraData_t_
{
    int tilt;
    int pan;
} BD_Cam_t;

typedef struct READER_THREAD_DATA_t READER_THREAD_DATA_t;

typedef struct RawFrame_t RawFrame_t;

class RollAndPPH {
private:
	float rollAngle; //ラジアン
	float pixPerHeight; //pixNum/pixHeight
public:
	//pixPerHeightの値が0.0のときそのインデックスにはまだ値が入っていないということになる
	RollAndPPH(){
		rollAngle = 200.0;
		pixPerHeight = 0.0;
	}
	//リングバッファに値を追加していく
	void setAngleAndPix(float angle,float pix){
		rollAngle = angle;
		pixPerHeight = pix;
	}
	//人とdroneの距離が範囲を超えた場合に値をリセットする
	void reset() {
		rollAngle = 200.0;
		pixPerHeight = 0.0;
	}
	float getCurrentSpeed(){
		return rollAngle;
	}
	float getCurrentPixPerHeight(){
		return pixPerHeight;
	}

};

typedef struct
{
    ARNETWORKAL_Manager_t *alManager;
    ARNETWORK_Manager_t *netManager;
    ARSTREAM_Reader_t *streamReader;
    ARSAL_Thread_t looperThread;
    ARSAL_Thread_t rxThread;
    ARSAL_Thread_t txThread;
    ARSAL_Thread_t videoTxThread;
    ARSAL_Thread_t videoRxThread;
    int d2cPort;
    int c2dPort;
    int arstreamFragSize;
    int arstreamFragNb;
    int arstreamAckDelay;
    uint8_t *videoFrame;
    uint32_t videoFrameSize;

    BD_PCMD_t dataPCMD;
    BD_Cam_t dataCam;

    ARCODECS_Manager_t *decoder;
    int decodingCanceled;
    ARSAL_Thread_t decodingThread;

    int hasReceivedFirstIFrame;
    RawFrame_t **freeRawFramePool;
    int rawFramePoolCapacity;
    int lastRawFrameFreeIdx;
    RawFrame_t **rawFrameFifo;
    int fifoReadIdx;
    int fifoWriteIdx;
    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState;
    vector<Rect> faceRectDetected;	//検出された矩形の座標
    vector<Rect> fullBodyRectDetected;	//検出された矩形の座標
    vector<Rect> upperBodyRectDetected;	//検出された矩形の座標
    //vector<double> ROC;
    vector<vector<int> > stats;
    vector<vector<int> >eigenvectors;
    FILE *video_out;

    ARSAL_Mutex_t mutex;

    ARSAL_Thread_t *readerThreads;
    READER_THREAD_DATA_t *readerThreadsData;
    int run;
    IHM_t *ihm;
    CascadeClassifier faceCascade;
    CascadeClassifier fullbodyCascade;
    CascadeClassifier upperbodyCascade;
    int imageFlag;	//画像処理実行フラグ
    bool findFace;	//顔発見フラグ
    bool downPPH;	//ピクセル/高さ　が下がったか
    bool ROCFlag;	//過去にROCを求めているか
    bool rollFlag;	//過去のroll値を保持しているか
    int coordinatenum;
    //int rocCount;
    float speedX,speedY,speedZ,roll,pitch,yaw,maxTilt,minTilt,currentTilt,maxRotationSpeed,minRotationSpeed,
	currentRotationSpeed,maxVerticalSpeed,minVerticalSpeed,currentVerticalSpeed;
    double altitude;
    float pastRoll,pastPixPerHeight,currentRoll,currentPixPerHeight,maxTargetPPH,minTargetPPH;	//pastRollはdegree
    double pastROC,currentROC,differenceROC,firstEV,secondEV;

} BD_MANAGER_t;

struct READER_THREAD_DATA_t
{
    BD_MANAGER_t *deviceManager;
    int readerBufferId;
};
//statsの7番目と8番目にcentroids格納
enum CentroidTypes{
	CENTER_X = 6,
	CENTER_Y = 7,
	LABEL = 8
};
/** Connection part **/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/** Network part **/
int startNetwork (BD_MANAGER_t *deviceManager);
void stopNetwork (BD_MANAGER_t *deviceManager);
void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);

/** Video part **/
int startVideo (BD_MANAGER_t *deviceManager);
void stopVideo (BD_MANAGER_t *deviceManager);
uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom);

/** decoding part **/
int startDecoder (BD_MANAGER_t *deviceManager);
void stopDecoder (BD_MANAGER_t *deviceManager);
int getNextDataCallback(uint8_t **data, void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);

/** Commands part **/
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
int sendPCMD(BD_MANAGER_t *deviceManager);
int sendCameraOrientation(BD_MANAGER_t *deviceManager);
int sendDate(BD_MANAGER_t *deviceManager);
int sendAllStates(BD_MANAGER_t *deviceManager);
int sendAllSettings(BD_MANAGER_t *deviceManager);
int sendTakeoff(BD_MANAGER_t *deviceManager);
int sendLanding(BD_MANAGER_t *deviceManager);
int sendEmergency(BD_MANAGER_t *deviceManager);
int sendBeginStream(BD_MANAGER_t *deviceManager);
int settingMaxTilt(BD_MANAGER_t *deviceManager);
int settingMaxRotationSpeed(BD_MANAGER_t *deviceManager);
int settingMaxVerticalSpeed(BD_MANAGER_t *deviceManager);

/** Commands callback part **/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager);
void unregisterARCommandsCallbacks(void);
void batteryStateChangedCallback (uint8_t percent, void *custom);
void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom);
void speedChangedCallback(float speedX,float speedY,float speedZ,void *custom);
void attitudeChangedCallback(float roll, float pitch, float yaw, void *custom);
void altitudeChangedCallback(double altitude, void *custom);
void MaxTiltChangedCallback_t (float current, float min, float max, void *custom);
void maxRotationSpeedChangedCallback (float current, float min, float max, void *custom);
void maxVerticalSpeedChangedCallback (float current, float min, float max, void *custom);
/** IHM callbacks **/
void onInputEvent (eIHM_INPUT_EVENT event, void *customData, int autoFlag,Mat infoWindow);
int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);

/** Image processing part **/
void imageProc(ARCODECS_Manager_Frame_t* frame,HOGDescriptor hog,BD_MANAGER_t *deviceManager);
void imageProc2(uint8_t* frame,HOGDescriptor hog,BD_MANAGER_t *deviceManager);
void autonomousFlying (eIHM_INPUT_EVENT event,BD_MANAGER_t *deviceManager,Mat InfoWindow);
void cameraControl(BD_MANAGER_t *deviceManager,vector<Point> coordDetected);
void directionControl(BD_MANAGER_t *deviceManager);
void distanceControl(BD_MANAGER_t *deviceManager);
void rollControl(BD_MANAGER_t *deviceManager);
void altitudeControl(BD_MANAGER_t *deviceManager);
double pixToDig(const int pix);
void labeling(const Mat input,Mat &output,Mat &dst,BD_MANAGER_t *deviceManager,const int maxLabelNum);
void exePca(const Mat input,BD_MANAGER_t *deviceManager);
template<class T>
bool areaComparator(const vector<T>& a,const vector<T>& b);
#endif /* _SDK_EXAMPLE_BD_H_ */
