#include <pthread.h>

#include "rtmpPusher.hh"
#include "ourMD5.hh"

//forward
void afterPlaying(void* clientData);
void *readFileSource(void *args);
void sendFramePacket(void* clientData);
void usage(UsageEnvironment& env);

#define DEFAULT_FPS 25
//#define DEBUG

UsageEnvironment* thatEnv;
char const* progName = NULL;
static unsigned rtmpReconnectCount = 0;

typedef struct {
	char const* inputFileName;
	char const* rtmpURL;
	ourRTMPClient* rtmpClient;
} args_t;

args_t args;

struct timeval timeNow;

UsageEnvironment& operator << (UsageEnvironment& env, const ourRTMPClient& rtmpClient) {
    return env << "[URL:\"" << rtmpClient.url() << "\"]: ";
}

int main(int argc, char** argv) {
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	thatEnv = BasicUsageEnvironment::createNew(*scheduler);

	progName = argv[0];

	if (argc < 2) {
		usage(*thatEnv);
		exit(0);
	}

	int opt;
	char rtmpUrl[120] = {0}, token[66] = {0}, md5[33] = {0};
	long nonce = 0L;
	char const* host = "127.0.0.1:1935";
	char const* app = "live";
	char const* stream = NULL;

	while ((opt = getopt(argc, argv, "f:e:h:a:p:s:")) != -1) {
		switch (opt) {
			case 'f':
				args.inputFileName = optarg;
				break;
			case 'e':
				sprintf(rtmpUrl, "%s", optarg);
				break;
			case 'h':
			case 'a':
			case 'p':
			case 's':
				if (opt == 'h')
					host = optarg;
				else if (opt == 'a')
					app = optarg;
				else if (opt == 'p') {
					nonce = our_random();
					sprintf(token, "%ld%s%s", nonce, optarg, "-1");
					our_MD5Data((unsigned char*)token, strlen(token), md5);
					sprintf(token, "?token=%s%ld", md5, nonce);
				} else if (opt == 's')
					stream = optarg;
				break;
			default:
				usage(*thatEnv);
				exit(0);

		}
	}

	if(strlen(rtmpUrl) == 0) {
		sprintf(rtmpUrl, "rtmp://%s/%s%s/%s", host, app, token, stream);
	}

	args.rtmpURL = strDup(rtmpUrl);

	pthread_t cthread;
	pthread_attr_t attributes;
	//void *cthread_return;
	pthread_attr_init(&attributes);
	pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
	pthread_create(&cthread, NULL, readFileSource, (void*)(&args));

	if (pthread_join(cthread, NULL) != 0) //pthread_join(cthread, &cthread_return)
		exit(1);

	thatEnv->taskScheduler().doEventLoop();
	return 0;
}

void afterPlaying(void* clientData){
	DummyFileSink* sink = (DummyFileSink*)clientData;
	sink->stopPlaying();
	Medium::close(sink->source());
	Medium::close(args.rtmpClient);
	//play
	readFileSource((void*)&args);
}

void *readFileSource(void *args) {
	if (args == NULL)
		return (void*) EXIT_FAILURE;

	args_t* params = (args_t*)args;
	FramedSource* fSource = ByteStreamFileSource::createNew(*thatEnv, params->inputFileName);
	if (fSource == NULL) {
		*thatEnv << "ERROR:\tUnable to open file \"" << params->inputFileName << "\" as a byte-stream file source\n";
		return (void*) EXIT_FAILURE;
	}

	params->rtmpClient = ourRTMPClient::createNew(*thatEnv, params->rtmpURL);
	if(params->rtmpClient == NULL) {
		*thatEnv << "ERROR:\tPublish the failed. endpoint:\"" << params->rtmpURL << "\n";
		return (void*) EXIT_FAILURE;
	}

	H264VideoStreamFramer* framer = H264VideoStreamFramer::createNew(*thatEnv, fSource, True);
	DummyFileSink* sink = DummyFileSink::createNew(*thatEnv, params->rtmpClient);
	sink->setBufferSize(4096);
	sink->startPlaying(*framer, afterPlaying, sink);

	return (void*)EXIT_SUCCESS;
}

void sendFramePacket(void* clientData) {
	DummyFileSink* sink = (DummyFileSink*)clientData;
	u_int8_t nut = sink->fData()[4] & 0x1F;

	if(isIDR(nut) || isNonIDR(nut)) {
		if(!sink->fClient->sendH264FramePacket(sink->fData(), sink->fSize, sink->fPts)) {

		}
	}

	sink->continuePlaying();
}

//Implementation of "DummyFileSink":
DummyFileSink* DummyFileSink::createNew(UsageEnvironment& env, ourRTMPClient* rtmpClient, char const* streamId) {
	return new DummyFileSink(env, rtmpClient, streamId);
}

DummyFileSink::DummyFileSink(UsageEnvironment& env, ourRTMPClient* rtmpClient, char const* streamId)
	: MediaSink(env), fClient(rtmpClient), fSize(0), fPts(0.0), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0),
	  fReceiveBuffer(NULL), fBufferSize(0), fPtsOffset(0.0), fHaveWrittenFirstFrame(True) {
	fStreamId = strDup(streamId);
	gettimeofday(&timeNow, NULL);
	fPtsOffset = (double)(timeNow.tv_sec * 1000.0 + timeNow.tv_usec/1000.0);
}

DummyFileSink::~DummyFileSink() {
	delete[] fReceiveBuffer;
	delete[] fStreamId;
}

Boolean DummyFileSink::continuePlaying() {
	if (fSource == NULL) return False;
	fSource->getNextFrame(fReceiveBuffer, fBufferSize, afterGettingFrame, this, onSourceClosure, this);
	return True;
}

void DummyFileSink::afterGettingFrame(void* clientData, unsigned frameSize,
		unsigned numTruncatedBytes, struct timeval presentationTime,
		unsigned durationInMicroseconds) {
	DummyFileSink* sink = (DummyFileSink*)clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime);
}

void DummyFileSink::afterGettingFrame(unsigned frameSize,
		unsigned numTruncatedBytes, struct timeval presentationTime) {
	fSize = frameSize;
	u_int8_t nut = fReceiveBuffer[4] & 0x1F;

	if(fHaveWrittenFirstFrame) {
		if (isSPS(nut)) {
			int width, height, fps;
			fSpsSize = frameSize;
			fSps = new u_int8_t[fSpsSize];
			memmove(fSps, fReceiveBuffer, fSpsSize);

			h264_decode_sps(fSps+4, fSpsSize-4, width, height, fps);
			envir() << *fClient << "H264 width:" << width << "\theight:" << height << "\tfps:" << DEFAULT_FPS << "\n";
			setBufferSize(width * height * 1.5 / 8);

			fClient->sendH264FramePacket(fSps, fSpsSize, 0);
		} else if (isPPS(nut)) {
			fPpsSize = frameSize;
			fPps = new u_int8_t[fPpsSize];
			memmove(fPps, fReceiveBuffer, fPpsSize);

			fClient->sendH264FramePacket(fPps, fPpsSize, 0);
		} else if (isIDR(nut)) {
			fClient->sendH264FramePacket(fReceiveBuffer, frameSize, 0);
			fHaveWrittenFirstFrame = False;
		}
		continuePlaying();
	} else {
		gettimeofday(&timeNow, NULL);
		fPts = (double)(timeNow.tv_sec * 1000.0 + timeNow.tv_usec/1000.0) - fPtsOffset;
		envir().taskScheduler().scheduleDelayedTask((1000 / DEFAULT_FPS * 1000), (TaskFunc*)sendFramePacket, this);
	}
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, char const* rtmpUrl, Boolean needAudioTrack) {
	ourRTMPClient* instance = new ourRTMPClient(env, rtmpUrl, needAudioTrack);
	return instance->rtmp != NULL ? instance : NULL;
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, char const* rtmpUrl, Boolean needAudioTrack)
	: Medium(env), rtmp(NULL), fSource(NULL), fNeedAudioTrack(needAudioTrack), fUrl(rtmpUrl) {
	do {
		rtmp = srs_rtmp_create(rtmpUrl);
		if (srs_rtmp_handshake(rtmp) != 0) {
			envir() << *this << "simple handshake failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << *this << "simple handshake success" << "\n";
#endif
		if (srs_rtmp_connect_app(rtmp) != 0) {
			envir() << *this << "connect vhost/app failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << *this << "connect vhost/app success" << "\n";
#endif

		int ret = srs_rtmp_publish_stream(rtmp);
		if (ret != 0) {
			envir() << *this << "publish stream failed.(ret=" << ret << ")\n";
			break;
		}
		envir() << *this << "publish stream success" << "\n";
		rtmpReconnectCount = 0;
		return;
	} while (0);

	Medium::close(this);
}

ourRTMPClient::~ourRTMPClient() {
#ifdef DEBUG
	envir() << *this << "Cleanup when unpublish. rtmpClient disconnect peer" << "\n";
#endif
	srs_rtmp_destroy(rtmp);
	rtmp = NULL;
	//RECONNECT_WAIT_DELAY(++rtmpReconnectCount);
	//ourRTMPClient::createNew(envir(), fUrl);
}

Boolean ourRTMPClient::sendH264FramePacket(u_int8_t* data, unsigned size, double pts) {
	do {
		if (NULL != data && size > 4) {
#ifdef DEBUG
			u_int8_t nut = data[4] & 0x1F;
			envir() << *this << "\n\tsent packet: type=video" << ", time=" << pts
			<< ", size=" << size << ", b[4]="
			<< (unsigned char*) data[4] << "("
			<< (isSPS(nut) ? "SPS" : (isPPS(nut) ? "PPS" : (isIDR(nut) ? "I" : (isNonIDR(nut) ? "P" : "Unknown"))))
			<< ")\n";
#endif
			int ret = srs_h264_write_raw_frames(rtmp, (char*) data, size, pts, pts);
			if (ret != 0) {
				if (srs_h264_is_dvbsp_error(ret)) {
					envir() << *this << "ignore drop video error, code=" << ret << "\n";
				} else if (srs_h264_is_duplicated_sps_error(ret)) {
					envir() << *this << "ignore duplicated sps, code=" << ret << "\n";
				} else if (srs_h264_is_duplicated_pps_error(ret)) {
					envir() << *this << "ignore duplicated pps, code=" << ret << "\n";
				} else {
					envir() << *this << "send h264 raw data failed. code=" << ret << "\n";
					break;
				}
			}
		}
		return True;
	} while (0);

	Medium::close(this);
	return False;
}

void usage(UsageEnvironment& env) {
	env << "Usage: " << progName << " -f <fifo_name> <[-e <url> | -h <host[:port]> [-a app] [-p password] -s <stream>]>\n";
	env << "Options:" << "\n";
	env << " -f: fifo pathname" << "\n";
	env << " -e: publish endpoint url e.g: rtmp://host:port/app?token=[AUTH_KEY]/stream" << "\n";
	env << " -h: publish endpoint host e.g: 127.0.0.1:1935" << "\n";
	env << " -a: publish appname default: live" << "\n";
	env << " -p: publish authentication password" << "\n";
	env << " -s: publish streamname" << "\n";
}
