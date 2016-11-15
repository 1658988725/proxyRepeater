
#include "rtmpPusher.hh"
#include <vector>

using namespace std;

typedef std::vector<conn_item_params_t>  CP_ARRAY;

//forward
cJSON* loadConfigFile(char const* path);
CP_ARRAY parseConfData(cJSON* conf);

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString);

void subsessionAfterPlaying(void* clientData);
void subsessionByeHandler(void* clientData);
void sendLivenessCommandHandler(void* clientData);
void streamTimerHandler(void* clientData);

void setupNextSubsession(RTSPClient* rtspClient);
void shutdownStream(RTSPClient* rtspClient, int exitCode = 0);
void usage(UsageEnvironment& env);

CP_ARRAY items;
char eventLoopWatchVariable = 0;
Boolean runDaemonMode = False;
unsigned rtspReconnectCount = 0, rtmpReconnectCount = 0;

void openURL(void* args) {
	if (args == NULL) return;

	conn_item_params_ptr params = (conn_item_params_ptr)args;

	ourRTSPClient* rtspClient = ourRTSPClient::createNew(*thatEnv, *params);
	if (rtspClient == NULL) {
		*thatEnv << "ERROR: Failed to create a RTSP client for URL \""
				<< params->srcStreamURL << "\": " << thatEnv->getResultMsg()
				<< "\n";
		return;
	}

	rtspClient->sendDescribeCommand(continueAfterDESCRIBE, params->rtspAUTH);
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
	cJSON* conf;

	while ((opt = getopt(argc, argv, "hc:d")) != -1) {
		switch (opt) {
		case 'h':
			usage(*thatEnv);
			exit(0);
		case 'c':
			conf = loadConfigFile(optarg);
			if (!conf) {
				*thatEnv << "File not found or json parse fail.\"" << optarg << "\"\n";
			} else {
				items = parseConfData(conf);
				unsigned delay = 100000;
				for (CP_ARRAY::iterator it = items.begin(); it != items.end(); ++it) {
					thatEnv->taskScheduler().scheduleDelayedTask(delay, (TaskFunc*)openURL, &(*it));
					delay += delay;
				}
				cJSON_Delete(conf);
				conf = NULL;
			}
			break;
		case 'd':
			runDaemonMode = True;
			break;
		default:
			usage(*thatEnv);
			exit(0);
		}
	}

	if (runDaemonMode) {
		pid_t pid = fork();
		if (pid < 0) {
			*thatEnv << "Exception: fork process error!" << "\n";
			exit(1);
		} else if (pid == 0)
			goto DONE;
		else
			exit(0);
	}

DONE:
	thatEnv->taskScheduler().doEventLoop(&eventLoopWatchVariable);
	return 0;
}

#define RTSP_CLIENT_VERBOSITY_LEVEL 0
#define TUNNEL_OVER_HTTP_PORTNUM 0

// Implementation of "ourRTSPClient":
ourRTSPClient* ourRTSPClient::createNew(UsageEnvironment& env, conn_item_params_t& params){
	return new ourRTSPClient(env, params);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment& env, conn_item_params_t& params)
	: RTSPClient(env, params.srcStreamURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName, TUNNEL_OVER_HTTP_PORTNUM, -1),
	  publisher(NULL), fParams(params) {
}

ourRTSPClient::~ourRTSPClient() {
#ifdef DEBUG
	envir() << *this << "Closed the rtspClient.\n";
#endif
	if (publisher != NULL) {
		Medium::close(publisher);
	}
	RECONNECT_WAIT_DELAY(++rtspReconnectCount);
	openURL(&fParams);
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, RTSPClient* rtspClient) {
	ourRTMPClient* instance = new ourRTMPClient(env, rtspClient);
	return instance->rtmp != NULL ? instance : NULL;
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient)
	: Medium(env), rtmp(NULL), fSource(rtspClient) {
	conn_item_params_t params = ((ourRTSPClient*)fSource)->fParams;
	fUrl = params.destStreamURL;
	do {
		rtmp = srs_rtmp_create(fUrl);
		if (srs_rtmp_handshake(rtmp) != 0) {
			envir() << "[URL:\"" << fUrl << "\"]: " << "simple handshake failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << *fSource <<"simple handshake success" << "\n";
#endif
		if (srs_rtmp_connect_app(rtmp) != 0) {
			envir() << "[URL:\"" << fUrl << "\"]: " << "connect vhost/app failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << "[URL:\"" << fUrl << "\"]: " <<"connect vhost/app success" << "\n";
#endif

		int ret = srs_rtmp_publish_stream(rtmp);
		if (ret != 0) {
			envir() << "[URL:\"" << fUrl << "\"]: " << "publish stream failed.(ret=" << ret << ")\n";
			break;
		}
#ifdef DEBUG
		envir() << *fSource << "publish stream success" << "\n";
#endif
		((ourRTSPClient*)fSource)->publisher = this;
		env << "[URL:\"" << fUrl << "\"]: " << "\n\tPublish the stream from \"" << params.srcStreamURL << "\"\n";
		rtmpReconnectCount = 0;
		return;
	} while (0);

	Medium::close(this);
}

ourRTMPClient::~ourRTMPClient() {
#ifdef DEBUG
	envir() << "[URL:\"" << fUrl << "\"]: " << "Cleanup when unpublish. rtmpClient disconnect peer" << "\n";
#endif
	srs_rtmp_destroy(rtmp);
	rtmp = NULL;
	((ourRTSPClient*)fSource)->publisher = NULL;
	RECONNECT_WAIT_DELAY(++rtmpReconnectCount);
}

//Implementation of "StreamClientState":
StreamClientState::StreamClientState()
	: session(NULL), iter(NULL), subsession(NULL), streamTimerTask(NULL),
	  checkAliveTimerTask(NULL), duration(0.0) {
}

StreamClientState::~StreamClientState() {
	delete iter;
	if (session != NULL) {
		UsageEnvironment& env = session->envir(); // alias
		env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
		env.taskScheduler().unscheduleDelayedTask(checkAliveTimerTask);
		Medium::close(session);
	}
}

// Implementation of "DummyRTPSink":
DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
	return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
	: MediaSink(env), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0), fBufferSize(100000),
	  fSubsession(subsession), fWidth(0), fHeight(0), fFps(0), fPtsOffset(0), fIdrOffset(0),
	  fWaitFirstFrameFlag(True), aacEncHandle(NULL), fAACBuffer(NULL) {
	fStreamId = strDup(streamId);
	if(strcasecmp(fSubsession.mediumName(), "video") == 0) {
		fWidth = VIDEO_MIN_WIDTH;
		fHeight = VIDEO_MIN_HEIGHT;
		fFps = VIDEO_DEFAULT_FPS;
		fBufferSize = fWidth * fHeight / 2;
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

			if ((aacEncHandle = Easy_AACEncoder_Init(initParam)) != NULL) {
				fAACBuffer = new BYTE[nMaxOutputBytes];
			}
		}
	}

	fReceiveBuffer = new u_int8_t[fBufferSize];
}

DummySink::~DummySink() {
	delete[] fSps;
	delete[] fPps;
	delete[] fReceiveBuffer;
	delete[] fStreamId;
	delete[] fAACBuffer;
	Easy_AACEncoder_Release(aacEncHandle);
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize,
		unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds) {
	DummySink* sink = (DummySink*) clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void DummySink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
		unsigned /*durationInMicroseconds*/) {
	ourRTSPClient* rtspClient = (ourRTSPClient*)(fSubsession.miscPtr);
	StreamClientState& scs = rtspClient->scs;
	u_int32_t pts;

	gettimeofday(&scs.gettingLastFrameTime, NULL);

	if (rtspClient->publisher == NULL)
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
					if (!rtspClient->publisher->sendH264FramePacket(fSps, fSpsSize, 0))
						goto RECONNECT;

					if (!rtspClient->publisher->sendH264FramePacket(fPps, fPpsSize, 0))
						goto RECONNECT;

					checkComplexIDRFrame();

					if (!rtspClient->publisher->sendH264FramePacket(fReceiveBuffer+fIdrOffset, frameSize+4-fIdrOffset, 0))
						goto RECONNECT;

					fWaitFirstFrameFlag = False;
				}
				goto NEXT_FRAME;
			} else {
				pts = fSubsession.getNormalPlayTime(presentationTime) * 1000;
				if (isIDR(nal_unit_type)) {
					if (!rtspClient->publisher->sendH264FramePacket(fReceiveBuffer+fIdrOffset, frameSize+4-fIdrOffset, pts))
						goto RECONNECT;
				} else if (isNonIDR(nal_unit_type)) {
					if (!rtspClient->publisher->sendH264FramePacket(fReceiveBuffer, frameSize+4, pts))
						goto RECONNECT;
				}
			}
		}
	} else if (strcasecmp(fSubsession.mediumName(), "audio") == 0) {
		if (fAACBuffer != NULL) {
			unsigned out_size;
			if (aacEncHandle != NULL) { //g711uLaw  g711alaw g726
				if(Easy_AACEncoder_Encode(aacEncHandle, fReceiveBuffer+4, frameSize, fAACBuffer, &out_size) <= 0)
					goto NEXT_FRAME;
			} else { //MPEG4-GENERIC
				if (!srs_aac_is_adts((char*)fReceiveBuffer+4, frameSize)) {
					memmove(fAACBuffer+7, fReceiveBuffer+4, frameSize);
					out_size = frameSize+7;
					if(!addADTStoPacket(fAACBuffer, out_size))
						goto NEXT_FRAME;
				} else {
					memmove(fAACBuffer, fReceiveBuffer+4, frameSize);
					out_size = frameSize;
				}
			}
			pts = fSubsession.getNormalPlayTime(presentationTime) * 1000;
			if (!rtspClient->publisher->sendAACFramePacket(fAACBuffer, out_size, pts, 1, fSubsession.numChannels()-1))
				goto RECONNECT;

		}
	}
	goto NEXT_FRAME;

RECONNECT:
	fWaitFirstFrameFlag = True;
	ourRTMPClient::createNew(envir(),rtspClient);
NEXT_FRAME:
	continuePlaying();
}

Boolean DummySink::continuePlaying() {
	if (fSource == NULL) return False;
	fReceiveBuffer[0] = 0; fReceiveBuffer[1] = 0;
	fReceiveBuffer[2] = 0; fReceiveBuffer[3] = 1;
	fSource->getNextFrame(fReceiveBuffer+4, fBufferSize-4, afterGettingFrame, this, onSourceClosure, this);
	return True;
}

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString) {
	do {
		UsageEnvironment& env = rtspClient->envir();
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs;

		if (resultCode != 0) {
			env << *rtspClient << "Failed to get a SDP description: " << resultString << "\n";
			delete[] resultString;
			break;
		}

		char const* sdpDescription = resultString;
#ifdef DEBUG
		env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";
#endif
		// Create a media session object from this SDP description:
		scs.session = MediaSession::createNew(env, sdpDescription);
		delete[] sdpDescription;
		if (scs.session == NULL) {
			env << *rtspClient << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
			break;
		} else if (!scs.session->hasSubsessions()) {
			env << *rtspClient << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
			break;
		}

		scs.iter = new MediaSubsessionIterator(*scs.session);
		setupNextSubsession(rtspClient);
		return;
	} while (0);

	// An unrecoverable error occurred with this stream.
	shutdownStream(rtspClient);
}

void setupNextSubsession(RTSPClient* rtspClient) {
	UsageEnvironment& env = rtspClient->envir();
	ourRTSPClient* client = (ourRTSPClient*) rtspClient;
	StreamClientState& scs = client->scs;

	scs.subsession = scs.iter->next();
	if (scs.subsession != NULL) {
		if ((!client->fParams.audioTrack && strcasecmp(scs.subsession->mediumName(), "audio") == 0)
				|| strcasecmp(scs.subsession->codecName(), "VND.ONVIF.METADATA") == 0) {
			setupNextSubsession(rtspClient);
			return;
		}

		if (!scs.subsession->initiate()) {
			env << *rtspClient << "Failed to initiate the \"" << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
			setupNextSubsession(rtspClient);
		} else {
#ifdef DEBUG
			env << *rtspClient << "Initiated the \"" << *scs.subsession << "\" subsession (";
			if (scs.subsession->rtcpIsMuxed()) {
				env << "client port " << scs.subsession->clientPortNum();
			} else {
				env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum() + 1;
			}
			env << ")\n";
#endif
			// By default, we request that the server stream its data using RTP/UDP.
			// If, instead, you want to request that the server stream via RTP-over-TCP, change the following to True:
			rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, client->fParams.rtspUseTcp);
		}
		return;
	}

	//MediaSubSession Iterator over, start Play
	if (scs.session->absStartTime() != NULL) {
		rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
	} else {
		scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
		rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->playStartTime(), scs.session->playEndTime());
	}
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
	Boolean success = False;
	do {
		UsageEnvironment& env = rtspClient->envir();
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs;

		if (resultCode != 0) {
			env << *rtspClient << "Failed to set up the \"" << *scs.subsession << "\" subsession: " << resultString << "\n";
			break;
		}
#ifdef DEBUG
		env << *rtspClient << "Set up the \"" << *scs.subsession << "\" subsession (";
		if (scs.subsession->rtcpIsMuxed()) {
			env << "client port " << scs.subsession->clientPortNum();
		} else {
			env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum() + 1;
		}
		env << ")\n";
#endif
		scs.subsession->sink = DummySink::createNew(env, *scs.subsession, rtspClient->url());
		if (scs.subsession->sink == NULL) {
			env << *rtspClient << "Failed to create a data sink for the \""
					<< *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
			break;
		}

#ifdef DEBUG
		env << *rtspClient << "Created a data sink for the \"" << *scs.subsession << "\" subsession\n";
#endif
		scs.subsession->miscPtr = rtspClient;
		DummySink* sink = (DummySink*) scs.subsession->sink;
		if (strcasecmp(scs.subsession->mediumName(), "video") == 0
				&& strcasecmp(scs.subsession->codecName(), "H264") == 0) {
			char const* spropStr = scs.subsession->attrVal_str("sprop-parameter-sets");
			if ( NULL != spropStr) {
				unsigned numSPropRecords = 0;
				SPropRecord* r = parseSPropParameterSets(spropStr, numSPropRecords);
				for (unsigned n = 0; n < numSPropRecords; ++n) {
					u_int8_t nal_unit_type = r[n].sPropBytes[0] & 0x1F;
					if (isSPS(nal_unit_type)) {
						sink->parseSpsPacket(r[n].sPropBytes, r[n].sPropLength);
					} else if (isPPS(nal_unit_type)) {
						sink->parsePpsPacket(r[n].sPropBytes, r[n].sPropLength);
					}
				}
				delete[] r;
			}
		}

		sink->startPlaying(*(scs.subsession->readSource()), subsessionAfterPlaying, scs.subsession);

		if (scs.subsession->rtcpInstance() != NULL) {
			scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
		}

		success = True;
	} while (0);

	delete[] resultString;

	if (success)
		setupNextSubsession(rtspClient);
	else
		shutdownStream(rtspClient);

}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
	Boolean success = False;
	do {
		UsageEnvironment& env = rtspClient->envir();
		ourRTSPClient* client = (ourRTSPClient*) rtspClient;
		StreamClientState& scs = client->scs;

		if (resultCode != 0) {
			env << *rtspClient << "Failed to start playing session: " << resultString << "\n";
			break;
		}

		if (scs.duration > 0) {
			unsigned const delaySlop = 2;
			scs.duration += delaySlop;
			unsigned uSecsToDelay = (unsigned) (scs.duration * 1000000);
			scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*) streamTimerHandler, rtspClient);
		}
#ifdef DEBUG
		env << *rtspClient << "Started playing session";
		if (scs.duration > 0) {
			env << " (for up to " << scs.duration << " seconds)";
		}
		env << "...\n";
#endif



		if (client->publisher == NULL) {
#ifdef DEBUG
			env << *rtspClient << "Start Creating ourRTMPClient ..." << "\n";
#endif
			if (ourRTMPClient::createNew(env, client) == NULL) {
				env << *rtspClient << "\n\tPublish the failed. endpoint:\"" << client->fParams.destStreamURL << "\n";
				break;
			}
		}
		scs.checkAliveTimerTask = env.taskScheduler().scheduleDelayedTask(CHECK_ALIVE_TASK_TIMER_INTERVAL,
				(TaskFunc*) sendLivenessCommandHandler, rtspClient);
		rtspReconnectCount = 0;
		success = True;
	} while (0);

	delete[] resultString;

	if (!success) {
		shutdownStream(rtspClient);
	}
}

void subsessionAfterPlaying(void* clientData) {
	MediaSubsession* subsession = (MediaSubsession*) clientData;
	RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);

	Medium::close(subsession->sink);
	subsession->sink = NULL;

	MediaSession& session = subsession->parentSession();
	MediaSubsessionIterator iter(session);
	while ((subsession = iter.next()) != NULL) {
		if (subsession->sink != NULL) return;
	}

	shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData) {
	MediaSubsession* subsession = (MediaSubsession*) clientData;
#ifdef DEBUG
	RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);
	UsageEnvironment& env = rtspClient->envir();
	env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession << "\" subsession\n";
#endif
	subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void* clientData) {
	ourRTSPClient* rtspClient = (ourRTSPClient*) clientData;
	StreamClientState& scs = rtspClient->scs;
	scs.streamTimerTask = NULL;
	shutdownStream(rtspClient);
}

void sendLivenessCommandHandler(void* clientData) {
	ourRTSPClient* rtspClient = (ourRTSPClient*) clientData;
	UsageEnvironment& env = rtspClient->envir();
	StreamClientState& scs = rtspClient->scs;

	gettimeofday(&timeNow, NULL);
	if (timeNow.tv_sec - scs.gettingLastFrameTime.tv_sec > CHECK_ALIVE_TASK_TIMER_INTERVAL / 1000000) {
		scs.checkAliveTimerTask = NULL;
		shutdownStream(rtspClient);
		return;
	} else if (rtspClient->sendGetParameterCommand(*scs.session, NULL, NULL) > 0) {
		scs.checkAliveTimerTask = env.taskScheduler().scheduleDelayedTask(CHECK_ALIVE_TASK_TIMER_INTERVAL,
				(TaskFunc*) sendLivenessCommandHandler, rtspClient);
	}
}

void shutdownStream(RTSPClient* rtspClient, int exitCode) {
	StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs;

	if (scs.session != NULL) {
		Boolean someSubsessionsWereActive = False;
		MediaSubsessionIterator iter(*scs.session);
		MediaSubsession* subsession;

		while ((subsession = iter.next()) != NULL) {
			if (subsession->sink != NULL) {
				Medium::close(subsession->sink);
				subsession->sink = NULL;

				if (subsession->rtcpInstance() != NULL) {
					subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
				}

				someSubsessionsWereActive = True;
			}
		}

		if (someSubsessionsWereActive) {
			rtspClient->sendTeardownCommand(*scs.session, NULL);
		}
	}

	Medium::close(rtspClient);
}

cJSON* loadConfigFile(char const* path) {
	FILE *f;
	if((f = fopen(path,"rb")) == NULL)
		return NULL;

	cJSON* json = NULL;
	fseek(f,0,SEEK_END);
	long len = ftell(f);
	fseek(f,0,SEEK_SET);
	char* data = (char*)malloc(len+1);
	if (fread(data,1,len,f) == (size_t)len) {
		data[len]='\0';
		json = cJSON_Parse(data);
	}
	free(data);
	fclose(f);
	return json;
}

CP_ARRAY parseConfData(cJSON* conf) {
	CP_ARRAY rval;
	char const* expire = "-1";

	int iCount = cJSON_GetArraySize(conf);
	for (int i = 0; i < iCount; ++i) {
		cJSON* pItem = cJSON_GetArrayItem(conf, i);
		if (NULL == pItem)
			continue;

		cJSON* enable = cJSON_GetObjectItem(pItem, "enable");
		if (enable != NULL && enable->valueint == 0)
			continue;

		cJSON* rtspUrl = cJSON_GetObjectItem(pItem, "rtspUrl");
		cJSON* rtspUsername = cJSON_GetObjectItem(pItem, "rtspUsername");
		cJSON* rtspPassword = cJSON_GetObjectItem(pItem, "rtspPassword");
		cJSON* rtspUseTcp = cJSON_GetObjectItem(pItem, "rtspUseTcp");
		cJSON* endpoint = cJSON_GetObjectItem(pItem, "endpoint");
		cJSON* stream = cJSON_GetObjectItem(pItem, "stream");
		cJSON* password = cJSON_GetObjectItem(pItem, "password");
		cJSON* audioTrack = cJSON_GetObjectItem(pItem, "audioTrack");

		if (NULL != rtspUrl && NULL != endpoint) {
			conn_item_params_t params;
			params.srcStreamURL = strDup(rtspUrl->valuestring);
			params.rtspAUTH = new Authenticator();
			if (rtspUsername != NULL && rtspPassword != NULL) {
				params.rtspAUTH->setUsernameAndPassword(strDup(rtspUsername->valuestring), strDup(rtspPassword->valuestring));
			}
			params.rtspUseTcp = (rtspUseTcp != NULL) ? rtspUseTcp->valueint == 1 : False;
			params.audioTrack = (audioTrack != NULL) ? audioTrack->valueint == 1 : False;

			char rtmpUrl[255] = {0};
			//rtmp://host:port/app[?nonce=x&token=y]/stream
			sprintf(rtmpUrl, "%s", endpoint->valuestring);
			if (NULL != password && strlen(password->valuestring) > 0) {
				long nonce = our_random();
				char token[64] = {0}, md5[33] = {0};
				sprintf(token, "%ld%s%s", nonce, password->valuestring, expire);
				our_MD5Data((unsigned char*)token, strlen(token), md5);
				sprintf(rtmpUrl, "%s?token=%s%ld", rtmpUrl, md5, nonce);
			}

			if (stream != NULL) {
				sprintf(rtmpUrl, "%s/%s", rtmpUrl, stream->valuestring);
			} else {
				sprintf(rtmpUrl, "%s/ch%d", rtmpUrl, i);
			}

			params.destStreamURL = strDup(rtmpUrl);
			rval.push_back(params);
		}
	}
	return rval;
}

void usage(UsageEnvironment& env) {
	env << "Usage: " << progName << " -c <conf> [-d]\n";
	env << "Options:" << "\n";
	env << " -c: load config file" << "\n";
	env << " -d: daemon mode" << "\n";
}
