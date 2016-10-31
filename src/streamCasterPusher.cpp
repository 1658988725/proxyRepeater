#include <pthread.h>

#include "rtmpPusher.hh"
#include "ourMD5.hh"

#define DEBUG

typedef struct {
	unsigned packetType;
	DummySink* sink;
	u_int8_t* data;
	unsigned size;
} send_frame_packet_t, *send_frame_packet_ptr;

//forward
void afterPlaying(void* clientData);
void *readFileSource(void *args);
void sendFramePacket(void* clientData);
void usage(UsageEnvironment& env);

conn_item_params_t params;

send_frame_packet_t packet;

void sendFramePacket(void* clientData) {
	send_frame_packet_ptr packet = (send_frame_packet_ptr)clientData;
	if (packet->packetType == 0) {
		u_int8_t nut = packet->data[4] & 0x1F;
		if ((isIDR(nut) || isNonIDR(nut)) && !packet->sink->fClient->sendH264FramePacket(packet->data, packet->size)) {
		}
	} else {

	}
	packet->sink->continuePlaying();
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

	params.videoFps = 0;

	while ((opt = getopt(argc, argv, "f:r:e:h:a:p:s:")) != -1) {
		switch (opt) {
			case 'f':
				params.srcStreamURL = optarg;
				break;
			case 'r':
				params.videoFps = atoi(optarg);
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

	params.destStreamURL = strDup(rtmpUrl);

	pthread_t cthread;
	pthread_attr_t attributes;
	//void *cthread_return;
	pthread_attr_init(&attributes);
	pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
	pthread_create(&cthread, NULL, readFileSource, (void*)&params);

	if (pthread_join(cthread, NULL) != 0) //pthread_join(cthread, &cthread_return)
		exit(1);

	thatEnv->taskScheduler().doEventLoop();
	return 0;
}

void afterPlaying(void* clientData){
	DummySink* sink = (DummySink*)clientData;
	sink->stopPlaying();
	Medium::close(sink->source());
	Medium::close(sink->fClient);
	//play
	readFileSource((void*)&params);
}

void *readFileSource(void *args) {
	if (args == NULL)
		return (void*) EXIT_FAILURE;

	conn_item_params_ptr ptr = (conn_item_params_ptr)args;
	FramedSource* fSource = ByteStreamFileSource::createNew(*thatEnv, ptr->srcStreamURL);
	if (fSource == NULL) {
		*thatEnv << "ERROR:\tUnable to open file \"" << ptr->srcStreamURL << "\" as a byte-stream file source\n";
		return (void*) EXIT_FAILURE;
	}

	ourRTMPClient* rtmpClient = ourRTMPClient::createNew(*thatEnv, ptr->destStreamURL);
	if(rtmpClient == NULL) {
		*thatEnv << "ERROR:\tPublish the failed. endpoint:\"" << ptr->destStreamURL << "\n";
		return (void*) EXIT_FAILURE;
	}

	H264VideoStreamFramer* framer = H264VideoStreamFramer::createNew(*thatEnv, fSource, True);
	MediaSession* session = MediaSession::createNew(*thatEnv, "v=0\r\n");
	if (session == NULL) {
		*thatEnv << "ERROR:\ttUnable create MediaSession\n";
		return (void*) EXIT_FAILURE;
	}

	MediaSubsessionIterator* iter = new MediaSubsessionIterator(*session);
	DummySink* sink = DummySink::createNew(*thatEnv, *(iter->next()));
	if (ptr->videoFps > 0) {
		sink->setVideoFps(ptr->videoFps);
	}

	sink->fClient = rtmpClient;

	sink->startPlaying(*framer, afterPlaying, sink);

	return (void*)EXIT_SUCCESS;
}

//Implementation of "DummyFileSink":
DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
	return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
	: MediaSink(env), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0),
	  fReceiveBuffer(NULL), fSubsession(subsession), fWidth(640), fHeight(480), fFps(25) {
	fStreamId = strDup(streamId);
	setBufferSize(fWidth * fHeight * 2 / 8);
}

DummySink::~DummySink() {
	delete[] fSps;
	delete[] fPps;
	delete[] fReceiveBuffer;
	delete[] fStreamId;
}

Boolean DummySink::continuePlaying() {
	if (fSource == NULL) return False;
	fSource->getNextFrame(fReceiveBuffer, fBufferSize, afterGettingFrame, this, onSourceClosure, this);
	return True;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize,
		unsigned numTruncatedBytes, struct timeval presentationTime,
		unsigned durationInMicroseconds) {
	DummySink* sink = (DummySink*)clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void DummySink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
		struct timeval presentationTime, unsigned /*durationInMicroseconds*/) {
	u_int8_t nut = fReceiveBuffer[4] & 0x1F;

	if(fClient->fWaitFirstFrameFlag) {
		if (isSPS(nut)) {
			parseSpsPacket(fReceiveBuffer + 4, frameSize);
#ifdef DEBUG
			envir() << *fClient << "H264 width:" << fWidth << "\theight:" << fHeight << "\tfps:" << fFps << "\n";
#endif
		} else if (isPPS(nut)) {
			parsePpsPacket(fReceiveBuffer + 4, frameSize);
		} else if (isIDR(nut)) {
			if (!fClient->sendH264FramePacket(fSps, fSpsSize, 0)
				|| !fClient->sendH264FramePacket(fPps, fPpsSize, 0)
				|| !fClient->sendH264FramePacket(fReceiveBuffer, frameSize+4, 0)) return;
			fClient->fWaitFirstFrameFlag = False;
		}
		continuePlaying();
	} else {
		packet.packetType = 0;
		packet.sink = this;
		packet.data = fReceiveBuffer;
		packet.size = frameSize;
		envir().taskScheduler().scheduleDelayedTask(1000 / fFps * 1000, (TaskFunc*)sendFramePacket, &packet);
	}
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, char const* rtmpUrl) {
	ourRTMPClient* instance = new ourRTMPClient(env, rtmpUrl);
	return instance->rtmp != NULL ? instance : NULL;
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, char const* rtmpUrl)
	: Medium(env), fWaitFirstFrameFlag(True), rtmp(NULL), fSource(NULL), fPtsOffset(0), fUrl(rtmpUrl) {
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

		gettimeofday(&timeNow, NULL);
		fPtsOffset = timeNow.tv_sec * 1000000 + timeNow.tv_usec;

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
}

Boolean ourRTMPClient::sendH264FramePacket(u_int8_t* data, unsigned size, double pts) {
	do {
		if (NULL != data && size > 4) {
			if (pts == -1.0) {
				gettimeofday(&timeNow, NULL);
				pts = (DWORD(timeNow.tv_sec * 1000000 + timeNow.tv_usec) - fPtsOffset) / 1000.0;
			}
#ifdef DEBUG
			u_int8_t nut = data[4] & 0x1F;
			envir() << *this << "sent packet: type=video" << ", time=" << pts
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
	env << "Usage: " << progName << " -f <fifo_name> [-r <frame rate>] <[-e <url> | -h <host[:port]> [-a app] [-p password] -s <stream>]>\n";
	env << "Options:" << "\n";
	env << " -f: fifo pathname" << "\n";
	env << " -r: set frame rate default: 25" << "\n";
	env << " -e: publish endpoint url e.g: rtmp://host:port/app?token=[AUTH_KEY]/stream1" << "\n";
	env << " -h: set endpoint host e.g: 127.0.0.1:1935" << "\n";
	env << " -a: set endpoint appname default: live" << "\n";
	env << " -p: set endpoint password" << "\n";
	env << " -s: set endpoint streamname" << "\n";
}
