#ifndef RTMPPUSHER_HH_
#define RTMPPUSHER_HH_

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "GroupsockHelper.hh"
#include "srs_librtmp.h"

#define RTSP_CLIENT_VERBOSITY_LEVEL 0

//MAX width*height*1.5  .e.g  IDR -> 1280x720x1.5
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE  (1024*1024*4)//(1024*1024+512*1024)

#define CHECK_ALIVE_TASK_TIMER_INTERVAL 5*1000*1000

#define RECONNECT_WAIT_DELAY(n) if (n >= 3) { usleep(CHECK_ALIVE_TASK_TIMER_INTERVAL); n = 0; }

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
	struct timeval lastGettingFrameTime;
};

class ourRTMPClient: public Medium {
public:
	static ourRTMPClient* createNew(UsageEnvironment& env,
			RTSPClient* rtspClient);
protected:
	ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient);
	virtual ~ourRTMPClient();
public:
	Boolean sendH264FramePacket(u_int8_t* data, unsigned size, u_int32_t currTimestamp);
private:
	srs_rtmp_t rtmp;
	u_int32_t priorTimestamp;
	u_int32_t dts, pts;
	RTSPClient* fSource;
};

class ourRTSPClient: public RTSPClient {
public:
	static ourRTSPClient* createNew(UsageEnvironment& env, unsigned channelId);
protected:
	ourRTSPClient(UsageEnvironment& env, unsigned channelId);
	virtual ~ourRTSPClient();
public:
	StreamClientState scs;
	ourRTMPClient* publisher;
	unsigned id() const { return fChannelId; };
private:
	unsigned fChannelId;
};

class DummySink: public MediaSink {
public:
	static DummySink* createNew(UsageEnvironment& env,
			MediaSubsession& subsession, char const* streamId = NULL);
protected:
	DummySink(UsageEnvironment& env, MediaSubsession& subsession,
			char const* streamId);
	virtual ~DummySink();

	static void afterGettingFrame(void* clientData, unsigned frameSize,
			unsigned numTruncatedBytes, struct timeval presentationTime,
			unsigned durationInMicroseconds);

	void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
			struct timeval presentationTime, unsigned durationInMicroseconds);

	// redefined virtual functions:
	virtual Boolean continuePlaying();
public:
	Boolean sendSpsPacket(u_int8_t* data, unsigned size, u_int32_t timestamp = 0) {
		if (fSps != NULL) {
			delete[] fSps; fSps = NULL;
		}
		fSpsSize = size + 4;
		fSps = new u_int8_t[fSpsSize];

		fSps[0] = 0;	fSps[1] = 0;
		fSps[2] = 0;	fSps[3] = 1;
		memmove(fSps + 4, data, size);

		if (timestamp == 0)
			return True;
		else {
			ourRTMPClient* rtmpClient = (ourRTMPClient*)((ourRTSPClient*) fSubsession.miscPtr)->publisher;
			return rtmpClient->sendH264FramePacket(fSps, fSpsSize, timestamp);
		}
	}

	Boolean sendPpsPacket(u_int8_t* data, unsigned size, u_int32_t timestamp = 0) {
		if (fPps != NULL) {
			delete[] fPps; fPps = NULL;
		}
		fPpsSize = size + 4;
		fPps = new u_int8_t[fPpsSize];

		fPps[0] = 0;	fPps[1] = 0;
		fPps[2] = 0;	fPps[3] = 1;
		memmove(fPps + 4, data, size);

		if (timestamp == 0)
			return True;
		else {
			ourRTMPClient* rtmpClient = (ourRTMPClient*)((ourRTSPClient*) fSubsession.miscPtr)->publisher;
			return rtmpClient->sendH264FramePacket(fPps, fPpsSize, timestamp);
		}
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
};

#endif /* RTMPPUSHER_HH_ */
