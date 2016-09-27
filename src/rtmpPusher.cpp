

#include <vector>
#include <pthread.h>

#ifdef NODE_V8_ADDON
#include <node.h>
#include <node_version.h>
#include <v8.h>
#if NODE_MAJOR_VERSION == 0 && NODE_MINOR_VERSION == 12
#define NODE_VERSION_12		1
#endif
#endif

#include "rtmpPusher.hh"
#include "cJSON.h"
#include "ourMD5.hh"

using namespace std;

UsageEnvironment* thatEnv;
char eventLoopWatchVariable = 0;
Boolean runDaemonMode = False;
unsigned rtspReconnectCount = 0, rtmpReconnectCount = 0;
char const* progName = NULL;

Boolean isSPS(u_int8_t nut) { return nut == 7; } //Sequence parameter set
Boolean isPPS(u_int8_t nut) { return nut == 8; } //Picture parameter set
Boolean isIDR(u_int8_t nut) { return nut == 5; } //Coded slice of an IDR picture
Boolean isNonIDR(u_int8_t nut) { return nut == 1; } //Coded slice of a non-IDR picture
//Boolean isSEI(u_int8_t nut) { return nut == 6; } //Supplemental enhancement information
//Boolean isAUD(u_int8_t nut) { return nut == 9; } //Access unit delimiter

UsageEnvironment& operator << (UsageEnvironment& env, const RTSPClient& rtspClient) {
    return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

UsageEnvironment& operator<< (UsageEnvironment& env, const MediaSubsession& subsession) {
    return env << subsession.mediumName() << "/" << subsession.codecName();
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

typedef struct {
	char const* rtspURL;
	Authenticator* rtspAUTH;
	char const* rtmpURL;
	Boolean rtspUseTcp;
} ConnParams;

typedef std::vector<ConnParams>  CP_ARRAY;
CP_ARRAY channels;

int parseConfData(cJSON* conf) {
	struct timeval timeNow;
	gettimeofday(&timeNow, NULL);
	long nonce = timeNow.tv_sec * 1000 + timeNow.tv_usec / 1000;
	char const* expire = "-1";

	int iCount = cJSON_GetArraySize(conf);
	for (int i = 0; i < iCount; ++i) {
		cJSON* pItem = cJSON_GetArrayItem(conf, i);
		if (NULL == pItem)
			continue;

		ConnParams params;
		cJSON* rtspUrl = cJSON_GetObjectItem(pItem, "rtspUrl");
		cJSON* rtspUsername = cJSON_GetObjectItem(pItem, "rtspUsername");
		cJSON* rtspPassword = cJSON_GetObjectItem(pItem, "rtspPassword");
		cJSON* rtspUseTcp = cJSON_GetObjectItem(pItem, "rtspUseTcp");
		cJSON* endpoint = cJSON_GetObjectItem(pItem, "endpoint");
		cJSON* stream = cJSON_GetObjectItem(pItem, "stream");
		cJSON* password = cJSON_GetObjectItem(pItem, "password");
		if (NULL != rtspUrl && NULL != endpoint) {
			params.rtspURL = strDup(rtspUrl->valuestring);
			if (rtspUsername != NULL && rtspPassword != NULL) {
				params.rtspAUTH = new Authenticator(rtspUsername->valuestring, rtspPassword->valuestring, False);
			}
			params.rtspUseTcp = (rtspUseTcp != NULL) ? rtspUseTcp->valueint == 1 : False;

			char rtmpUrl[120] = { '\0' };
			//rtmp://host:port/app[?nonce=x&token=y]/stream
			sprintf(rtmpUrl, "%s", endpoint->valuestring);
			if (NULL != password && strlen(password->valuestring) > 0) {
				nonce += 500;
				char token[50] = { '\0' }, md5[33] = { '\0' };
				sprintf(token, "%ld%s%s", nonce, password->valuestring, expire);
				our_MD5Data((unsigned char*) token, strlen(token), md5);
				sprintf(rtmpUrl, "%s?nonce=%ld&token=%s", rtmpUrl, nonce, md5);
			}

			if (stream != NULL)
				sprintf(rtmpUrl, "%s/%s", rtmpUrl, stream->valuestring);
			else
				sprintf(rtmpUrl, "%s/ch%d", rtmpUrl, i);

			params.rtmpURL = strDup(rtmpUrl);

			channels.push_back(params);
		}
	}
	return iCount;
}

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

void *openURL(void *args) {
	unsigned id = *((unsigned*)args);

	ourRTSPClient* rtspClient = ourRTSPClient::createNew(*thatEnv, id);
	if (rtspClient == NULL) {
		*thatEnv << "ERROR: Failed to create a RTSP client for URL \"" << channels[id].rtspURL << "\": " << thatEnv->getResultMsg() << "\n";
		pthread_exit(0);
	}
	rtspClient->sendDescribeCommand(continueAfterDESCRIBE, channels[id].rtspAUTH);
	return (void *) 0;
}

#ifndef NODE_V8_ADDON
int main(int argc, char** argv) {
	OutPacketBuffer::maxSize = DUMMY_SINK_RECEIVE_BUFFER_SIZE;
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	thatEnv = BasicUsageEnvironment::createNew(*scheduler);

	if (argc < 2) {
		usage(*thatEnv);
		exit(0);
	}

	progName = argv[0];

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
				eventLoopWatchVariable = parseConfData(conf) > 0 ? 0 : 1;
				unsigned ch = 0;
				for (CP_ARRAY::iterator it = channels.begin(); it != channels.end(); ++it, ch++) {
					pthread_t cthread;
					pthread_attr_t attributes;
					//void *cthread_return;
					pthread_attr_init(&attributes);
					pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
					pthread_create(&cthread, NULL, openURL, (void*)(&ch));

					//pthread_join(cthread, &cthread_return)
					if (pthread_join(cthread, NULL) != 0)
						continue;
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
#else //Build NODE_V8_ADDON

#ifdef NODE_VERSION_12
void InitMethod(const v8::FunctionCallbackInfo<v8::Value>& args) {
	v8::Isolate* isolate = v8::Isolate::GetCurrent();
	v8::HandleScope scope(isolate);
#else
v8::Handle<v8::Value> InitMethod(const v8::Arguments& args) {
	v8::HandleScope scope;
#endif

	if (args.Length() < 1) {
#ifdef NODE_VERSION_12
		isolate->ThrowException(v8::Exception::Error(v8::String::NewFromUtf8(isolate, "Wrong arguments")));
		return;
#else
		v8::ThrowException(v8::Exception::Error(v8::String::New("Wrong arguments")));
	    return scope.Close(v8::Undefined());
#endif
	}

	if(!args[0]->IsString()) {
#ifdef NODE_VERSION_12
		isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, "Arguments[0] not string type")));
		return;
#else
		v8::ThrowException(v8::Exception::TypeError(v8::String::New("Arguments[0] not string type")));
	    return scope.Close(v8::Undefined());
#endif
	}

	if(!args[1]->IsFunction()) {
#ifdef NODE_VERSION_12
		isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, "Arguments[1] not function type")));
		return;
#else
	    ThrowException(v8::Exception::TypeError(v8::String::New("Arguments[1] not function type")));
	    return scope.Close(v8::Undefined());
#endif
	}

	v8::Local<v8::Function> cb = v8::Local<v8::Function>::Cast(args[1]);
	v8::String::Utf8Value str(args[0]);
	const unsigned argc = 2;
	cJSON* conf = cJSON_Parse(*str);
	if(!conf) {
#ifdef NODE_VERSION_12
		v8::Local<v8::Value> argv[argc] = { v8::Boolean::New(isolate, True), v8::String::NewFromUtf8(isolate, "Json data parse fail.") };
		cb->Call(isolate->GetCurrentContext()->Global(), argc, argv);
		return;
#else
		Local<v8::Value> argv[argc] = { v8::Local<v8::Value>::New(v8::Boolean::New(True)) , v8::Local<v8::Value>::New(v8::String::New("Json data parse fail.")) };
	    cb->Call(Context::GetCurrent()->Global(), argc, argv);
	    return scope.Close(v8::Undefined());
#endif
	}
	if(parseConfData(conf) == 0) {
#ifdef NODE_VERSION_12
		v8::Local<v8::Value> argv[argc] = { v8::Boolean::New(isolate, True), v8::String::NewFromUtf8(isolate, "Json data not exists.") };
		cb->Call(isolate->GetCurrentContext()->Global(), argc, argv);
		return;
#else
		v8::Local<v8::Value> argv[argc] = { v8::Local<v8::Value>::New(v8::Boolean::New(True)), v8::Local<v8::Value>::New(v8::String::New("Json data not exists.")) };
		cb->Call(Context::GetCurrent()->Global(), argc, argv);
	    return scope.Close(v8::Undefined());
#endif
	}

	unsigned ch = 0;
	for (CP_ARRAY::iterator it = channels.begin(); it != channels.end(); ++it, ch++) {
		pthread_t cthread;
		pthread_attr_t attributes;
		//void *cthread_return;
		pthread_attr_init(&attributes);
		pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
		pthread_create(&cthread, NULL, openURL, (void*)(&ch));

		//pthread_join(cthread, &cthread_return)
		if (pthread_join(cthread, NULL) != 0)
			continue;
	}

	cJSON_Delete(conf);
	conf = NULL;

#ifdef NODE_VERSION_12
	v8::Local<v8::Value> argv[argc-1] = { v8::Boolean::New(isolate, False) };
	cb->Call(isolate->GetCurrentContext()->Global(), argc-1, argv);
#else
	v8::Local<v8::Value> argv[argc-1] = { v8::Local<v8::Value>::New(v8::Boolean::New(False)) };
	cb->Call(Context::GetCurrent()->Global(), argc-1, argv);
	return scope.Close(Undefined());
#endif
}

#ifdef NODE_VERSION_12
void StartMethod(const v8::FunctionCallbackInfo<v8::Value>& args) {
	v8::Isolate* isolate = v8::Isolate::GetCurrent();
	v8::HandleScope scope(isolate);
#else
v8::Handle<v8::Value> StartMethod(const v8::Arguments& args) {
	v8::HandleScope scope;
#endif
	thatEnv->taskScheduler().doEventLoop(&eventLoopWatchVariable);
#ifndef NODE_VERSION_12
	return scope.Close(v8::Undefined());
#endif
}

void Init(v8::Handle<v8::Object> exports, v8::Handle<v8::Object> module) {
	OutPacketBuffer::maxSize = DUMMY_SINK_RECEIVE_BUFFER_SIZE;
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	thatEnv = BasicUsageEnvironment::createNew(*scheduler);
#ifdef NODE_VERSION_12
	v8::Isolate* isolate = v8::Isolate::GetCurrent();
	exports->Set(v8::String::NewFromUtf8(isolate, "init"), v8::FunctionTemplate::New(isolate, InitMethod)->GetFunction());
	exports->Set(v8::String::NewFromUtf8(isolate, "start"), v8::FunctionTemplate::New(isolate, StartMethod)->GetFunction());
#else
	exports->Set(v8::String::NewSymbol("init"), v8::FunctionTemplate::New(InitMethod)->GetFunction());
	exports->Set(v8::String::NewSymbol("start"), v8::FunctionTemplate::New(StartMethod)->GetFunction());
#endif
}

NODE_MODULE(node_nvr_addon, Init);
#endif

// Implementation of "ourRTSPClient":
ourRTSPClient* ourRTSPClient::createNew(UsageEnvironment& env, unsigned channelId){
	return new ourRTSPClient(env, channelId);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment& env, unsigned channelId)
	: RTSPClient(env, channels[channelId].rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, "rtmpPusher", 0, -1),
	  publisher(NULL), fChannelId(channelId) {
}

ourRTSPClient::~ourRTSPClient() {
	rtspReconnectCount++;
#ifdef DEBUG
	envir() << *this << "Closed the rtspClient.\n";
#endif
	if (publisher != NULL) {
		Medium::close(publisher);
	}

	RECONNECT_WAIT_DELAY(rtspReconnectCount);

	openURL(&fChannelId);
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, RTSPClient* rtspClient) {
	return new ourRTMPClient(env, rtspClient);
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient)
	: Medium(env), rtmp(NULL), fTimestamp(0), dts(0), pts(0), fSource(rtspClient) {
	unsigned id = ((ourRTSPClient*)fSource)->id();
	do {
		rtmp = srs_rtmp_create(channels[id].rtmpURL);
		if (srs_rtmp_handshake(rtmp) != 0) {
#ifdef DEBUG
			envir() << *fSource << "simple handshake failed." << "\n";
#endif
			break;
		}
#ifdef DEBUG
		envir() << *fSource <<"simple handshake success" << "\n";
#endif
		if (srs_rtmp_connect_app(rtmp) != 0) {
			envir() << *fSource << "connect vhost/app failed." << "\n";
			break;
		}
#ifdef DEBUG
		envir() << *fSource <<"connect vhost/app success" << "\n";
#endif

		int ret = srs_rtmp_publish_stream(rtmp);
		if (ret != 0) {
			envir() << *fSource << "publish stream failed.(ret=" << ret << ")\n";
			break;
		}
#ifdef DEBUG
		envir() << *fSource << "publish stream success" << "\n";
#endif
		((ourRTSPClient*)fSource)->publisher = this;
		env << *fSource << "\n\tPublish the stream. endpoint:\"" << channels[id].rtmpURL << "\"\n";
		rtmpReconnectCount = 0;
		return;
	} while (0);

	Medium::close(this);
}

ourRTMPClient::~ourRTMPClient() {
#ifdef DEBUG
	envir() << *fSource << "Cleanup when unpublish. rtmpClient disconnect peer" << "\n";
#endif
	rtmpReconnectCount++;
	srs_rtmp_destroy(rtmp);
	((ourRTSPClient*)fSource)->publisher = NULL;

	RECONNECT_WAIT_DELAY(rtmpReconnectCount);
}

Boolean ourRTMPClient::sendH264FramePacket(u_int8_t* data, unsigned size, long timestamp) {
	do {
		if (NULL != data && size > 0) {
			if (fTimestamp == 0)
				fTimestamp = timestamp;

			pts = dts += (timestamp - fTimestamp);
			fTimestamp = timestamp;
			int ret = srs_h264_write_raw_frames(rtmp, (char*) data, size, dts, pts);
			if (ret != 0) {
				if (srs_h264_is_dvbsp_error(ret)) {
					envir() << *fSource << "ignore drop video error, code=" << ret << "\n";
				} else if (srs_h264_is_duplicated_sps_error(ret)) {
					envir() << *fSource << "ignore duplicated sps, code=" << ret << "\n";
				} else if (srs_h264_is_duplicated_pps_error(ret)) {
					envir() << *fSource << "ignore duplicated pps, code=" << ret << "\n";
				} else {
					envir() << *fSource << "send h264 raw data failed. code=" << ret << "\n";
					Medium::close(this);
					break;
				}
			}
#ifdef DEBUG
			u_int8_t nut = data[4] & 0x1F;
			envir() << *fSource << "sent packet: type=video" << ", time=" << dts
			<< ", size=" << size << ", b[4]="
			<< (unsigned char*) data[4] << "("
			<< (isSPS(nut) ? "SPS" : (isPPS(nut) ? "PPS" : (isIDR(nut) ? "I" : (isNonIDR(nut) ? "P" : "Unknown"))))
			<< ")\n";
#endif
		}
		return True;
	} while (0);

	return False;
}

//Implementation of "StreamClientState":
StreamClientState::StreamClientState()
	:session(NULL), iter(NULL), subsession(NULL), streamTimerTask(NULL), checkAliveTimerTask(NULL), duration(0.0) {
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

// Implementation of "DummySink":
DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
	return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
	: MediaSink(env), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0), fSubsession(subsession),
	  fHaveWrittenFirstFrame(True) {
	fStreamId = strDup(streamId);
	fReceiveBuffer = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE];

	ourRTSPClient* rtspClient = (ourRTSPClient*) subsession.miscPtr;
	if (rtspClient->publisher == NULL) {
#ifdef DEBUG
		envir() << *rtspClient << "Start Creating ourRTMPClient ..." << "\n";
#endif
		ourRTMPClient::createNew(env, rtspClient);
		if (rtspClient->publisher == NULL) {
			envir() << *rtspClient << "\n\tPublish the failed. endpoint:\""
					<< channels[rtspClient->id()].rtmpURL << "\n";
		}
	}
}

DummySink::~DummySink() {
	delete[] fReceiveBuffer; fReceiveBuffer = NULL;
	delete[] fStreamId; fStreamId = NULL;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize,
		unsigned numTruncatedBytes, struct timeval presentationTime,
		unsigned durationInMicroseconds) {
	DummySink* sink = (DummySink*) clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void DummySink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
		unsigned /*durationInMicroseconds*/) {
	ourRTSPClient* rtspClient = (ourRTSPClient*)fSubsession.miscPtr;
	StreamClientState& scs = rtspClient->scs;
	u_int8_t nal_unit_type = fReceiveBuffer[4] & 0x1F; //0xFF;

	gettimeofday(&scs.lastGettingFrameTime, NULL);
	long timestamp = scs.lastGettingFrameTime.tv_sec * 1000 + scs.lastGettingFrameTime.tv_usec / 1000;

	if (rtspClient->publisher == NULL)
		goto RECONNECT;

	if (fHaveWrittenFirstFrame) {
		if (!isSPS(nal_unit_type) && !isPPS(nal_unit_type) && !isIDR(nal_unit_type))
			goto NEXT;
		else if (isSPS(nal_unit_type)) {
			if (!sendSpsPacket(fReceiveBuffer + 4, frameSize, timestamp))
				goto RECONNECT;
		} else if (isPPS(nal_unit_type)) {
			if (!sendPpsPacket(fReceiveBuffer + 4, frameSize, timestamp))
				goto RECONNECT;
			fHaveWrittenFirstFrame = False;
		} else if (isIDR(nal_unit_type)) {
			//send sdp: sprop-parameter-sets
			if (!((ourRTMPClient*)rtspClient->publisher)->sendH264FramePacket(fSps, fSpsSize, timestamp))
				goto RECONNECT;

			if (!((ourRTMPClient*)rtspClient->publisher)->sendH264FramePacket(fPps, fPpsSize, timestamp))
				goto RECONNECT;

			fHaveWrittenFirstFrame = False;
		}
		goto NEXT;
	}

	if (strcasecmp(fSubsession.mediumName(), "video") == 0 && (isIDR(nal_unit_type) || isNonIDR(nal_unit_type))) {
		fReceiveBuffer[0] = 0; fReceiveBuffer[1] = 0;
		fReceiveBuffer[2] = 0; fReceiveBuffer[3] = 1;
		if (!((ourRTMPClient*)rtspClient->publisher)->sendH264FramePacket(fReceiveBuffer, frameSize + 4, timestamp))
			goto RECONNECT;
	}
	goto NEXT;

RECONNECT:
	fHaveWrittenFirstFrame = True;
	ourRTMPClient::createNew(envir(),rtspClient);
NEXT:
	continuePlaying();
}

Boolean DummySink::continuePlaying() {
	if (fSource == NULL)
		return False;
	fSource->getNextFrame(fReceiveBuffer + 4, DUMMY_SINK_RECEIVE_BUFFER_SIZE - 4, afterGettingFrame, this, onSourceClosure, this);
	return True;
}

/*
void openURL(UsageEnvironment& env, int id) {
	ourRTSPClient* rtspClient = ourRTSPClient::createNew(env, id);
	if (rtspClient == NULL) {
		env << "ERROR: Failed to create a RTSP client for URL \"" << channels[id].rtspURL << "\": " << env.getResultMsg() << "\n";
		return;
	}
	rtspClient->sendDescribeCommand(continueAfterDESCRIBE, channels[id].rtspAUTH);
}
*/

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString) {
	int nextStepFlag = 0;
	do {
		UsageEnvironment& env = rtspClient->envir();
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs;

		if (resultCode != 0) {
			env << *rtspClient << "Failed to get a SDP description: " << resultString << "\n";
			break;
		}

		if (strstr(resultString, "m=video") == NULL) {
			env << *rtspClient << "Not found video by SDP description (i.e., no \"m=video\" lines)\n";
			break;
		}

		char const* sdpDescription = resultString;
#ifdef DEBUG
		env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";
#endif
		// Create a media session object from this SDP description:
		scs.session = MediaSession::createNew(env, sdpDescription);
		delete[] sdpDescription;
		sdpDescription = NULL;

		if (scs.session == NULL) {
			env << *rtspClient << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
			break;
		} else if (!scs.session->hasSubsessions()) {
			env << *rtspClient << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
			break;
		}
		scs.iter = new MediaSubsessionIterator(*scs.session);
		setupNextSubsession(rtspClient);
		nextStepFlag += 1;
	} while (0);

	// An unrecoverable error occurred with this stream.
	if (nextStepFlag == 0) {
		shutdownStream(rtspClient);
	}
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
	int nextStepFlag = 0;
	do {
		UsageEnvironment& env = rtspClient->envir();
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs;
		scs.subsession->miscPtr = rtspClient;

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
		scs.subsession->sink = DummySink::createNew(env, *scs.subsession);
		// perhaps use your own custom "MediaSink" subclass instead
		if (scs.subsession->sink == NULL) {
			env << *rtspClient << "Failed to create a data sink for the \""
					<< *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
			break;
		}

		if (strcasecmp(scs.subsession->mediumName(), "video") == 0
				&& strcasecmp(scs.subsession->codecName(), "H264") == 0) {
			const char* spropStr = scs.subsession->attrVal_str("sprop-parameter-sets");
			if ( NULL != spropStr) {
				unsigned numSPropRecords = 0;
				DummySink* dummySink = (DummySink*) scs.subsession->sink;
				SPropRecord* r = parseSPropParameterSets(spropStr, numSPropRecords);
				for (unsigned n = 0; n < numSPropRecords; ++n) {
					u_int8_t nal_unit_type = r[n].sPropBytes[0] & 0x1F;
					if (isSPS(nal_unit_type)) {
						dummySink->sendSpsPacket(r[n].sPropBytes, r[n].sPropLength);
					} else if (isPPS(nal_unit_type)) {
						dummySink->sendPpsPacket(r[n].sPropBytes, r[n].sPropLength);
					}
				}
				delete[] r; r = NULL;
			}
		}
#ifdef DEBUG
		env << *rtspClient << "Created a data sink for the \"" << *scs.subsession << "\" subsession\n";
#endif
		scs.subsession->sink->startPlaying(*scs.subsession->readSource(), subsessionAfterPlaying, scs.subsession);
		if (scs.subsession->rtcpInstance() != NULL) {
			scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
		}
		nextStepFlag += 1;
	} while (0);

	delete[] resultString; resultString = NULL;

	if (nextStepFlag == 0)
		shutdownStream(rtspClient);
	else
		setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
	int nextStepFlag = 0;
	do {
		UsageEnvironment& env = rtspClient->envir();
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs;

		if (resultCode != 0) {
			env << *rtspClient << "Failed to start playing session: " << resultString << "\n";
			break;
		}

		if (scs.duration > 0) {
			unsigned const delaySlop = 2;
			scs.duration += delaySlop;
			unsigned uSecsToDelay = (unsigned) (scs.duration * 1000000);
			scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(
					uSecsToDelay, (TaskFunc*) streamTimerHandler,
					scs.subsession);
		}
#ifdef DEBUG
		env << *rtspClient << "Started playing session";
		if (scs.duration > 0) {
			env << " (for up to " << scs.duration << " seconds)";
		}
		env << "...\n";
#endif
		scs.checkAliveTimerTask = env.taskScheduler().scheduleDelayedTask(
				CHECK_ALIVE_TASK_TIMER_INTERVAL,
				(TaskFunc*) sendLivenessCommandHandler, rtspClient);
		nextStepFlag += 1;
		rtspReconnectCount = 0;
	} while (0);

	delete[] resultString; resultString = NULL;

	if (nextStepFlag == 0) {
		shutdownStream(rtspClient);
	}
}

void setupNextSubsession(RTSPClient* rtspClient) {
	UsageEnvironment& env = rtspClient->envir();
	ourRTSPClient* client = (ourRTSPClient*) rtspClient;
	StreamClientState& scs = client->scs;

	scs.subsession = scs.iter->next();
	if (scs.subsession != NULL) {
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
			//#define REQUEST_STREAMING_OVER_TCP      True
			rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, channels[client->id()].rtspUseTcp);
		}
		return;
	}

	//MediaSubSession Iterator over, start Play
	if (scs.session->absStartTime() != NULL) {
		rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
	} else {
		scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
		rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
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

void subsessionAfterPlaying(void* clientData) {
	MediaSubsession* subsession = (MediaSubsession*) clientData;
	RTSPClient* rtspClient = (RTSPClient*) subsession->miscPtr;

	Medium::close(subsession->sink);
	subsession->sink = NULL;

	MediaSession& session = subsession->parentSession();
	MediaSubsessionIterator iter(session);
	while ((subsession = iter.next()) != NULL) {
		if (subsession->sink != NULL)
			return;
	}
	shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData) {
	MediaSubsession* subsession = (MediaSubsession*) clientData;
#ifdef DEBUG
	RTSPClient* rtspClient = (RTSPClient*) subsession->miscPtr;
	UsageEnvironment& env = rtspClient->envir();
	env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession << "\" subsession\n";
#endif
	subsessionAfterPlaying(subsession);
}

void sendLivenessCommandHandler(void* clientData) {
	ourRTSPClient* rtspClient = (ourRTSPClient*) clientData;
	UsageEnvironment& env = rtspClient->envir();
	StreamClientState& scs = rtspClient->scs;

	struct timeval timeNow;
	gettimeofday(&timeNow, NULL);

	if (timeNow.tv_sec - scs.lastGettingFrameTime.tv_sec < 3) {
		rtspClient->sendGetParameterCommand(*scs.session, NULL, NULL);
		scs.checkAliveTimerTask = env.taskScheduler().scheduleDelayedTask(
				CHECK_ALIVE_TASK_TIMER_INTERVAL,
				(TaskFunc*) sendLivenessCommandHandler, rtspClient);
	} else {
		scs.checkAliveTimerTask = NULL;
		shutdownStream(rtspClient);
	}
}

void streamTimerHandler(void* clientData) {
	ourRTSPClient* rtspClient = (ourRTSPClient*) clientData;
	StreamClientState& scs = rtspClient->scs;
	scs.streamTimerTask = NULL;
	shutdownStream(rtspClient);
}

void usage(UsageEnvironment& env) {
	env << "Usage: " << progName << " -c <conf> [-d]\n";
	env << "Options:" << "\n";
	env << "\t-c: load config file" << "\n";
	env << "\t-d: daemon mode" << "\n";
}
