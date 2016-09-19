#ifndef __TEST_SRS_LIBRTMP_HH_
#define __TEST_SRS_LIBRTMP_HH_

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "srs_librtmp.h"

#define RTSP_CLIENT_VERBOSITY_LEVEL 0

//MAX width*height*1.5  .e.g  IDR -> 1280x720x1.5
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE  (1024*1024+512*1024)

#define CHECK_ALIVE_TASK_TIMER_INTERVAL 30*1000*1000

//#define DEBUG

void openURL(UsageEnvironment& env, char const* rtspURL, char const* rtmpURL);

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString);
void setupNextSubsession(RTSPClient* rtspClient);
void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

void subsessionAfterPlaying(void* clientData);
void subsessionByeHandler(void* clientData);
void streamTimerHandler(void* clientData);
void sendLivenessCommandHandler(void* clientData);

void usage(UsageEnvironment& env);
void announceStream(RTSPClient* rtspClient);

Boolean isSPS(u_int8_t nut)  { return nut == 7; } //Sequence parameter set
Boolean isPPS(u_int8_t nut)  { return nut == 8; } //Picture parameter set
Boolean isIDR(u_int8_t nut)  { return nut == 5; } //Coded slice of an IDR picture
Boolean isNonIDR(u_int8_t nut)  { return nut == 1; } //Coded slice of a non-IDR picture
//Boolean isSEI(u_int8_t nut) const { return nut == 6; } //Supplemental enhancement information
//Boolean isAUD(u_int8_t nut) const { return nut == 9; } //Access unit delimiter

class StreamClientState {
public:
    StreamClientState();
    virtual ~StreamClientState();
public:
	MediaSession* session;
	MediaSubsessionIterator* iter;
	MediaSubsession* subsession;
	TaskToken streamTimerTask;
	TaskToken checkAliveTimerTask;
	double duration;
};

class ourRTSPClient: public RTSPClient {
public:
	static ourRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL,
					char const* rtmpURL, int verbosityLevel = 0, char const* applicationName = NULL,
					portNumBits tunnelOverHTTPPortNum = 0);
protected:
	ourRTSPClient(UsageEnvironment& env, char const* rtspURL, char const* rtmpURL,
					int verbosityLevel, char const* applicationName,
					portNumBits tunnelOverHTTPPortNum);

	virtual ~ourRTSPClient();
public:
	StreamClientState scs;
	void* publisher;        //ourRTMPClient
	char const* endpoint() const { return fDestUrl; }       //rtmpURL
private:
    char* fDestUrl;
};

class ourRTMPClient: public Medium {
public:
	static ourRTMPClient* createNew(UsageEnvironment& env,  RTSPClient* rtspClient);
protected:
	ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient);
	virtual ~ourRTMPClient();
public:
	Boolean sendH264FramePacket(u_int8_t* data, unsigned size, unsigned timestamp);
private:
	Boolean connect();
private:
	srs_rtmp_t rtmp;
	unsigned fTimestamp;
	u_int32_t dts, pts;
	ourRTSPClient* fSource;
};

class DummySink: public MediaSink {
public:
	static DummySink* createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId = NULL);
protected:
	DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId);
	virtual ~DummySink();

	static void afterGettingFrame(void* clientData, unsigned frameSize,
					unsigned numTruncatedBytes, struct timeval presentationTime,
					unsigned durationInMicroseconds);

	void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
					struct timeval presentationTime, unsigned durationInMicroseconds);

	// redefined virtual functions:
	virtual Boolean continuePlaying();
public:
	Boolean sendSpsPacket(u_int8_t* data, unsigned size, unsigned timestamp = 0) {
		if (fSps != NULL) { delete[] fSps; fSps = NULL; }
		fSpsSize = size+4;
		fSps = new u_int8_t[fSpsSize];

		fSps[0] = 0;    fSps[1] = 0;
		fSps[2] = 0;    fSps[3] = 1;
		memmove(fSps+4, data, size);

		if(timestamp == 0)
			return True;
		else
			return  rtmpClient->sendH264FramePacket(fSps, fSpsSize, timestamp);
	}

	Boolean sendPpsPacket(u_int8_t* data, unsigned size, unsigned timestamp = 0) {
		if (fPps != NULL) { delete[] fPps; fPps = NULL; }
		fPpsSize = size+4;
		fPps = new u_int8_t[fPpsSize];

		fPps[0] = 0;    fPps[1] = 0;
		fPps[2] = 0;    fPps[3] = 1;
		memmove(fPps+4, data, size);

		if(timestamp == 0)
			return True;
		else
			return  rtmpClient->sendH264FramePacket(fPps, fPpsSize, timestamp);
	}
private:
	u_int8_t* fSps;
	u_int8_t* fPps;
	unsigned fSpsSize;
	unsigned fPpsSize;
	u_int8_t* fReceiveBuffer;
	char* fStreamId;
	MediaSubsession& fSubsession;
	Boolean fHaveWrittenFirstFrame;
	ourRTMPClient* rtmpClient ;
};

UsageEnvironment& operator << (UsageEnvironment& env, const RTSPClient& rtspClient) {
    return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

UsageEnvironment& operator<< (UsageEnvironment& env, const MediaSubsession& subsession) {
    return env << subsession.mediumName() << "/" << subsession.codecName();
}

#endif /* __TEST_SRS_LIBRTMP_HH_ */
