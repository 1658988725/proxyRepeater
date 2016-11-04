
#include "rtmpPusher.hh"

typedef struct {
	DummySink* sink;
	u_int8_t* data;
	unsigned size;
	u_int32_t pts;
	u_int32_t channels;
} send_frame_packet_t, *send_frame_packet_ptr;

conn_item_params_t params;
ourRTMPClient* rtmpClient;
send_frame_packet_t videoPacket;
send_frame_packet_t audioPacket;

//forward
void afterPlaying(void* clientData);
void readByteStreamSource(void* args);
int readSdpFile(char const* path, char*& sdpDescription);
void sendFramePacket(void* clientData);
void usage(UsageEnvironment& env);

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
	char const* stream = "demo";

	while ((opt = getopt(argc, argv, "i:u:h:a:p:s:")) != -1) {
		switch (opt) {
			case 'i':
				params.srcStreamURL = optarg;
				break;
			case 'u':
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

	if (strlen(rtmpUrl) == 0) {
		sprintf(rtmpUrl, "rtmp://%s/%s%s/%s", host, app, token, stream);
	}

	params.destStreamURL = strDup(rtmpUrl);

	thatEnv->taskScheduler().scheduleDelayedTask(100 * 1000, (TaskFunc*)readByteStreamSource, &params);
	thatEnv->taskScheduler().doEventLoop();
	return 0;
}

void afterPlaying(void* clientData){
	MediaSubsession* subsession = (MediaSubsession*)clientData;
	Medium::close(subsession->sink);
	subsession->sink = NULL;

	MediaSession& session = subsession->parentSession();
	MediaSubsessionIterator iter(session);
	while ((subsession = iter.next()) != NULL) {
		if (subsession->sink != NULL) return;
	}
	Medium::close(&session);

	Medium::close(rtmpClient);
	rtmpClient = NULL;

	//play
	thatEnv->taskScheduler().scheduleDelayedTask(500 * 1000, (TaskFunc*)readByteStreamSource, &params);
}

void readByteStreamSource(void* args) {
	if (args == NULL) return;

	conn_item_params_ptr ptr = (conn_item_params_ptr)args;

	char* sdpDescription = NULL;
	if (readSdpFile(ptr->srcStreamURL, sdpDescription) == 0) {
		*thatEnv << "ERROR: Unable to open file \"" << ptr->srcStreamURL << "\" as a sdp file\n";
		return;
	}

	*thatEnv << "Got a SDP description:\n" << sdpDescription << "\n";

	MediaSession* session = MediaSession::createNew(*thatEnv, sdpDescription);
	if (session == NULL) {
		*thatEnv << "ERROR: Unable create MediaSession\n";
		return;
	}

	rtmpClient = ourRTMPClient::createNew(*thatEnv, ptr->destStreamURL);
	if(rtmpClient == NULL) {
		*thatEnv << "ERROR: Publish the failed. endpoint:\"" << ptr->destStreamURL << "\n";
		return;
	}

	if (session != NULL) {
		MediaSubsessionIterator iter(*session);
		MediaSubsession* subsession;
		while ((subsession = iter.next()) != NULL) {
			if (strcasecmp(subsession->mediumName(), "video") == 0 && strcasecmp(subsession->codecName(), "H264") == 0) {
				FramedSource* videoSource = ByteStreamFileSource::createNew(*thatEnv, subsession->attrVal_str("src"));
				if (videoSource == NULL) {
					*thatEnv << "[URL:\"" << rtmpClient->url() << "\"]: " << "WARN: Unable to open file \"" << subsession->attrVal_str("src") << "\" as a byte-stream file video source\n";
					continue;
				}

				subsession->addFilter(H264VideoStreamFramer::createNew(*thatEnv, videoSource, True));

			} else if (strcasecmp(subsession->mediumName(), "audio") == 0 && strcasecmp(subsession->codecName(), "MPA") == 0) {
				FramedSource* audioSource = ADTSAudioFileSource::createNew(*thatEnv, subsession->attrVal_str("src"));
				if (audioSource == NULL) {
					*thatEnv << "[URL:\"" << rtmpClient->url() << "\"]: " << "WARN: Unable to open file \"" << subsession->attrVal_str("src") << "\" as a adts audio file source\n";
					continue;
				}

				subsession->addFilter((FramedFilter*)audioSource);
			}
			subsession->sink = DummySink::createNew(*thatEnv, *subsession, ptr->destStreamURL);
			subsession->sink->startPlaying(*(subsession->readSource()), afterPlaying, subsession);
		}
	}
}

int readSdpFile(char const* path, char*& sdpDescription) {
	FILE *f;
	if((f = fopen(path,"rb")) == NULL)
		return 0;
	fseek(f,0,SEEK_END);
	long len = ftell(f);
	fseek(f,0,SEEK_SET);
	sdpDescription = (char*)malloc(len+1);
	if (fread(sdpDescription,1,len,f) == (size_t)len) {
		sdpDescription[len]='\0';
	}
	fclose(f);
	return 1;
}

//Implementation of "DummyFileSink":
DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
	return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
	: MediaSink(env), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0), fBufferSize(100000),
	 fSubsession(subsession), fWidth(0), fHeight(0), fFps(0), fPtsOffset(0), fIdrOffset(0),
	 fWaitFirstFrameFlag(True), aacEncHandle(NULL), fAACBuffer(NULL) {

	gettimeofday(&timeNow, NULL);
	fPtsOffset = u_int64_t(timeNow.tv_sec * 1000000 + timeNow.tv_usec);

	fStreamId = strDup(streamId);

	if(strcasecmp(fSubsession.mediumName(), "video") == 0) {
		fWidth = fSubsession.videoWidth() == 0 ? VIDEO_MIN_WIDTH : fSubsession.videoWidth();
		fHeight = fSubsession.videoHeight() == 0 ? VIDEO_MIN_HEIGHT : fSubsession.videoHeight();
		fFps = fSubsession.videoFPS() == 0 ? VIDEO_DEFAULT_FPS : fSubsession.videoFPS();
		fBufferSize = fWidth * fHeight * 2 / 8;
	} else if (strcasecmp(fSubsession.mediumName(), "audio") == 0) {
		unsigned nPCMBitSize = 16;
		fBufferSize = 1024 * fSubsession.numChannels() * nPCMBitSize / 8;
		unsigned nMaxOutputBytes = (6144 / 8) * fSubsession.numChannels();
		fFps = 1000000 / fSubsession.rtpTimestampFrequency();

		if (strcasecmp(fSubsession.codecName(), "MPA") == 0
				|| (strcasecmp(fSubsession.codecName(), "MPEG4-GENERIC") == 0
						&& strcasecmp(fSubsession.attrVal_str("mode"), "AAC-hbr") == 0)) {
			fAACBuffer = new BYTE[nMaxOutputBytes];
		} else if (strcasecmp(fSubsession.codecName(), "PCMU") == 0
				|| strcasecmp(fSubsession.codecName(), "PCMA") == 0
				|| strcmp(fSubsession.codecName(), "G726") == 0) {
			InitParam initParam;
			initParam.u32AudioSamplerate = fSubsession.rtpTimestampFrequency();
			initParam.ucAudioChannel = fSubsession.numChannels();
			initParam.u32PCMBitSize = nPCMBitSize;
			initParam.ucAudioCodec = fSubsession.rtpPayloadFormat() == 0 ? Law_ULaw :
					fSubsession.rtpPayloadFormat() == 8 ? Law_ALaw : Law_G726;
			if (initParam.ucAudioCodec == Law_G726) {
				initParam.g726param.ucRateBits = Rate16kBits;
			}

			aacEncHandle = Easy_AACEncoder_Init(initParam);
			if (aacEncHandle != NULL) {
				fAACBuffer = new BYTE[nMaxOutputBytes];
			}
		}
	}

	fReceiveBuffer = new u_int8_t[fBufferSize];
	//envir() << fSubsession.mediumName() << "/" << fSubsession.codecName() << "\t" << fBufferSize << "\t" << fFps << "\n";
}

DummySink::~DummySink() {
	delete[] fSps;
	delete[] fPps;
	delete[] fReceiveBuffer;
	delete[] fStreamId;
	delete[] fAACBuffer;
	Easy_AACEncoder_Release(aacEncHandle);
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
	if (rtmpClient == NULL)
		goto RECONNECT;

	if (strcasecmp(fSubsession.mediumName(), "video") == 0) {
		if (strcasecmp(fSubsession.codecName(), "H264") == 0) {
			u_int8_t nal_unit_type = fReceiveBuffer[4] & 0x1F; //0xFF;
			if (fWaitFirstFrameFlag) {
				if (isSPS(nal_unit_type)) {
					parseSpsPacket(fReceiveBuffer+4, frameSize);
				} else if (isPPS(nal_unit_type)) {
					parsePpsPacket(fReceiveBuffer+4, frameSize);
				} else if (isIDR(nal_unit_type)) {
					if (!rtmpClient->sendH264FramePacket(fSps, fSpsSize, 0))
						goto RECONNECT;

					if (!rtmpClient->sendH264FramePacket(fPps, fPpsSize, 0))
						goto RECONNECT;

					checkComplexIDRFrame();

					if (!rtmpClient->sendH264FramePacket(fReceiveBuffer+fIdrOffset, frameSize+4-fIdrOffset, 0))
						goto RECONNECT;

					fWaitFirstFrameFlag = False;
				}
				goto NEXT_FRAME;
			} else {
				videoPacket.sink = this;
				videoPacket.data = isIDR(nal_unit_type) ? fReceiveBuffer+fIdrOffset : fReceiveBuffer;
				videoPacket.size = isIDR(nal_unit_type) ? frameSize-fIdrOffset : frameSize;
				gettimeofday(&timeNow, NULL);
				videoPacket.pts = (u_int64_t(timeNow.tv_sec * 1000000 + timeNow.tv_usec) - fPtsOffset) / 1000;
				videoPacket.channels = 0;
				envir().taskScheduler().scheduleDelayedTask(1000 / fFps * 1000, (TaskFunc*)sendFramePacket, &videoPacket);
				return;
			}
		}
	} else if (strcasecmp(fSubsession.mediumName(), "audio") == 0) {
		if (fAACBuffer == NULL)
			goto NEXT_FRAME;

		unsigned out_size;
		if (aacEncHandle != NULL) { //g711uLaw  g711alaw g726
			if(Easy_AACEncoder_Encode(aacEncHandle, fReceiveBuffer, frameSize, fAACBuffer, &out_size) <= 0)
				goto NEXT_FRAME;
		} else { //MPEG4-GENERIC MPA
			if (!srs_aac_is_adts((char*)fReceiveBuffer, frameSize)) {
				memmove(fAACBuffer+7, fReceiveBuffer, frameSize);
				out_size = frameSize+7;
				if(!addADTStoPacket(fAACBuffer, out_size))
					goto NEXT_FRAME;
			} else {
				memmove(fAACBuffer, fReceiveBuffer, frameSize);
				out_size =  frameSize;
			}
		}

		audioPacket.sink = this;
		audioPacket.data = fAACBuffer;
		audioPacket.size = out_size;
		gettimeofday(&timeNow, NULL);
		audioPacket.pts = (u_int64_t(timeNow.tv_sec * 1000000 + timeNow.tv_usec) - fPtsOffset) / 1000;
		audioPacket.channels = fSubsession.numChannels();
		envir().taskScheduler().scheduleDelayedTask(fFps * 1000, (TaskFunc*)sendFramePacket, &audioPacket);
		return;
	}

RECONNECT:
	rtmpClient = ourRTMPClient::createNew(envir(), fStreamId);
NEXT_FRAME:
	continuePlaying();
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, char const* rtmpUrl) {
	ourRTMPClient* instance = new ourRTMPClient(env, rtmpUrl);
	return instance->rtmp != NULL ? instance : NULL;
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, char const* rtmpUrl)
	: Medium(env), rtmp(NULL), fSource(NULL), fUrl(rtmpUrl) {
	do {
		rtmp = srs_rtmp_create(fUrl);
		if (srs_rtmp_handshake(rtmp) != 0) {
			envir() << "[URL:\"" << fUrl << "\"]: " << "simple handshake failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << "[URL:\"" << fUrl << "\"]: " << "simple handshake success" << "\n";
#endif
		if (srs_rtmp_connect_app(rtmp) != 0) {
			envir() << "[URL:\"" << fUrl << "\"]: " << "connect vhost/app failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << "[URL:\"" << fUrl << "\"]: " << "connect vhost/app success" << "\n";
#endif

		int ret = srs_rtmp_publish_stream(rtmp);
		if (ret != 0) {
			envir() << "[URL:\"" << fUrl << "\"]: " << "publish stream failed.(ret=" << ret << ")\n";
			break;
		}
		envir() << "[URL:\"" << fUrl << "\"]: " << "publish stream success" << "\n";
		return;
	} while (0);

	Medium::close(this);
}

ourRTMPClient::~ourRTMPClient() {
	envir() << "[URL:\"" << fUrl << "\"]: " << "Cleanup when unpublish. rtmpClient disconnect peer" << "\n";
	srs_rtmp_destroy(rtmp);
	rtmp = NULL;
}

void sendFramePacket(void* clientData) {
	send_frame_packet_ptr pkt = (send_frame_packet_ptr)clientData;
	if (pkt->channels == 0) {
		u_int8_t nut = pkt->data[4] & 0x1F;
		if (isIDR(nut) || isNonIDR(nut)) {
			if (!rtmpClient->sendH264FramePacket(pkt->data, pkt->size, pkt->pts)) {

			}
		}
	} else {
		if (!rtmpClient->sendAACFramePacket(pkt->data, pkt->size, pkt->pts, 1, pkt->channels-1)) {

		}
	}
	pkt->sink->continuePlaying();
}

void usage(UsageEnvironment& env) {
	env << "Usage: " << progName << " -i <sdp file> <[-u <url> | [-h <host>] [-a <app>] [-p <password>] [-s <stream>]>\n";
	env << "Options:" << "\n";
	env << " -i: sdp pathname" << "\n";
	env << " -u: publish endpoint url. e.g: rtmp://host:port/app?token=[AUTH_KEY]/stream1" << "\n";
	env << " -h: publish host. default: 127.0.0.1:1935" << "\n";
	env << " -a: publish appName. default: live" << "\n";
	env << " -p: publish password." << "\n";
	env << " -s: publish streamName. default: demo" << "\n";
}
