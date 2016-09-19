#include "testSrslibrtmp.hh"
#include "cJSON.h"
#include "ourMD5.hh"

#ifdef NODE_V8_ADDON
#include <node.h>
#include <node_version.h>
#include <v8.h>
#include <pthread.h>

#if NODE_MAJOR_VERSION == 0 && NODE_MINOR_VERSION == 12
#define NODE_VERSION_12		1
#endif
#endif

//static unsigned rtspClientCount = 0;
char eventLoopWatchVariable = 0;
char const* progName = NULL;

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

#ifndef NODE_V8_ADDON
int main(int argc, char** argv) {
	OutPacketBuffer::maxSize = DUMMY_SINK_RECEIVE_BUFFER_SIZE;
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
	progName = strDup(argv[0]);

	if (argc < 2) {
		usage(*env);
		exit(1);
	}

	int opt;
	cJSON* conf;
	while ((opt = getopt(argc, argv, "c:a:s:S:H:p:h")) != -1) {
		switch (opt) {
			case 'c':
			//*env << "option c: " << optarg << "\n";
			conf = loadConfigFile(optarg);
			if (!conf) {
				*env << "File not found or json parse fail.\"" << optarg << "\"\n";
			} else {
				int iCount = cJSON_GetArraySize(conf);
				struct timeval timeNow;
				for (int i=0; i<iCount; ++i) {
					cJSON* pItem = cJSON_GetArrayItem(conf, i);
					if (NULL == pItem)
						continue;

					cJSON* url = cJSON_GetObjectItem(pItem, "url");
					cJSON* endpoint = cJSON_GetObjectItem(pItem, "endpoint");
					cJSON* stream = cJSON_GetObjectItem(pItem, "stream");
					cJSON* password = cJSON_GetObjectItem(pItem, "password");
					if (NULL != url && NULL != endpoint && NULL != stream) {
						char rtmpUrl[120] = {'\0'};
						//rtmp://host:port/app[?nonce=x&token=y]/stream
						sprintf(rtmpUrl, "%s", endpoint->valuestring);
						if(NULL != password && strlen(password->valuestring) > 0) {
							gettimeofday(&timeNow, NULL);
							long nonce = timeNow.tv_sec*1000 + timeNow.tv_usec/1000;
							char token[50] = {'\0'}, md5[33] = {'\0'};
							sprintf(token, "%ld%s%s", nonce, password->valuestring, "-1");
							our_MD5Data((unsigned char*)token, strlen(token), md5);
							sprintf(rtmpUrl, "%s?nonce=%ld&token=%s", rtmpUrl, nonce, md5);
						}
						sprintf(rtmpUrl, "%s/%s", rtmpUrl, stream->valuestring);
						//*env << "\t" << url->valuestring << "\t" << rtmpUrl << "\n";
						openURL(*env, url->valuestring, rtmpUrl);
						usleep(100*1000);
					}
				}
				cJSON_Delete(conf);
				env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
				/*
				pid_t pid = fork();
				if (pid < 0) {
					printf("error in fork!");
					exit(1);
				} else if (pid == 0) {
				 	env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
				} else
					exit(0);
				*/
			}
			break;
			case 'h': default:
				usage(*env);
				break;
		}
	}
	return 0;
}
#else
UsageEnvironment* env ;

void *daemonThreadFunc(void *args) {
	pid_t pid = fork();
	if (pid < 0) {
		exit(1);
	} else if (pid == 0) {
		//printf("i am the child process, my process id is %d\n", getpid());
		env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
	} else if (pid > 0){
		//printf("i am the parent process, my process id is %d\n", getpid());
		exit(0);
	}
	pthread_exit(0);
}

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
	//printf("%s\n", *str);
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

	int iCount = cJSON_GetArraySize(conf);
	if(iCount == 0) {
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

	struct timeval timeNow;

	OutPacketBuffer::maxSize = DUMMY_SINK_RECEIVE_BUFFER_SIZE;
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	env = BasicUsageEnvironment::createNew(*scheduler);

	for (int i=0; i<iCount; ++i) {
	      cJSON* pItem = cJSON_GetArrayItem(conf, i);
	      if (NULL == pItem) continue;
	      
	      cJSON* url = cJSON_GetObjectItem(pItem, "url");
	      cJSON* endpoint = cJSON_GetObjectItem(pItem, "endpoint");
	      cJSON* stream = cJSON_GetObjectItem(pItem, "stream");
	      cJSON* password = cJSON_GetObjectItem(pItem, "password");

	      if (NULL != url && NULL != endpoint && NULL != stream) {
	              char rtmpUrl[120] = {'\0'};
	              //rtmp://host:port/app[?nonce=x&token=y]/stream
	              sprintf(rtmpUrl, "%s", endpoint->valuestring);
	              if(NULL != password && strlen(password->valuestring) > 0) {
	            	  gettimeofday(&timeNow, NULL);
	            	  long nonce = timeNow.tv_sec*1000 + timeNow.tv_usec/1000;
	            	  char token[50] = {'\0'}, md5[33] = {'\0'};
	            	  sprintf(token, "%ld%s%s", nonce, password->valuestring, "-1");
	            	  our_MD5Data((unsigned char*)token, strlen(token), md5);
	            	  sprintf(rtmpUrl, "%s?nonce=%ld&token=%s", rtmpUrl, nonce, md5);
	            }
	            sprintf(rtmpUrl, "%s/%s", rtmpUrl, stream->valuestring);
	            openURL(*env, url->valuestring, rtmpUrl);
	            usleep(100*1000);
	      }
	}
	cJSON_Delete(conf);

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
	pthread_t id;
	int ret = pthread_create(&id, NULL, daemonThreadFunc, NULL);
	if(ret != 0){
#ifdef NODE_VERSION_12
		isolate->ThrowException(v8::Exception::Error(v8::String::NewFromUtf8(isolate, "Create child thread error!")));
		return;
#else
	    v8::ThrowException(v8::Exception::Error(v8::String::New("Create child thread error!")));
	    return scope.Close(v8::Undefined());
#endif
	}

#ifndef NODE_VERSION_12
	return scope.Close(v8::Undefined());
#endif
}

void Init(v8::Handle<v8::Object> exports, v8::Handle<v8::Object> module) {
#ifdef NODE_VERSION_12
	v8::Isolate* isolate = v8::Isolate::GetCurrent();
	exports->Set(v8::String::NewFromUtf8(isolate, "init"), v8::FunctionTemplate::New(isolate, InitMethod)->GetFunction());
	exports->Set(v8::String::NewFromUtf8(isolate, "start"), v8::FunctionTemplate::New(isolate, StartMethod)->GetFunction());
#else
	exports->Set(v8::String::NewSymbol("init"), v8::FunctionTemplate::New(InitMethod)->GetFunction());
	exports->Set(v8::String::NewSymbol("start"), v8::FunctionTemplate::New(StartMethod)->GetFunction());
#endif
}

NODE_MODULE(node_nvr_addon, Init)
#endif

void openURL(UsageEnvironment& env, char const* rtspURL, char const* rtmpURL) {
	ourRTSPClient* rtspClient = ourRTSPClient::createNew(env, rtspURL, rtmpURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
	if (rtspClient == NULL) {
		env << "ERROR: Failed to create a RTSP client for URL \"" << rtspURL << "\": " << env.getResultMsg() << "\n";
		return;
	}
	//++rtspClientCount;
	rtspClient->sendDescribeCommand(continueAfterDESCRIBE);	
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

		if(strstr(resultString, "m=video") == NULL) {
			env << *rtspClient << "Not found video by SDP description: " << resultString << "\n";
			delete[] resultString;
			break;
		}

		char* const sdpDescription = resultString;
#ifdef DEBUG
		env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";
#endif
		// Create a media session object from this SDP description:
		scs.session = MediaSession::createNew(env, sdpDescription);
		delete[] sdpDescription; // because we don't need it anymore

		if (scs.session == NULL) {
			env << *rtspClient << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
			break;
		} else if (!scs.session->hasSubsessions()) {
			env << *rtspClient << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
			break;
		}
		if(ourRTMPClient::createNew(env, rtspClient) == NULL) {
			env << *rtspClient << "Publish the stream. endpoint:\"" << ((ourRTSPClient*) rtspClient)->endpoint() << "\" failed\n";
			break;
		}

		scs.iter = new MediaSubsessionIterator(*scs.session);
		setupNextSubsession(rtspClient);
		return;
	} while (0);

	// An unrecoverable error occurred with this stream.
	shutdownStream(rtspClient);
	usleep(5 * 1000 * 1000);
}

void setupNextSubsession(RTSPClient* rtspClient) {
	UsageEnvironment& env = rtspClient->envir(); // alias
	StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias

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
			rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, False);
		}
		return;
	}

	if (scs.session->absStartTime() != NULL) {
		rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
	} else {
		scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
		rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
	}
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
	do {
		UsageEnvironment& env = rtspClient->envir(); // alias
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias
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
			env << *rtspClient << "Failed to create a data sink for the \"" << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
			break;
		}

		DummySink* dummySink = (DummySink*)scs.subsession->sink;
		if( strcasecmp( scs.subsession->mediumName(), "video" ) == 0  &&
			strcasecmp( scs.subsession->codecName(), "H264" ) == 0) {
			const char* spropStr = scs.subsession->attrVal_str("sprop-parameter-sets");
		if( NULL != spropStr ) {
			unsigned numSPropRecords = 0;
			SPropRecord* r = parseSPropParameterSets(spropStr, numSPropRecords);
			for(unsigned n=0; n<numSPropRecords; ++n) {
				u_int8_t nal_unit_type = r[n].sPropBytes[0] & 0x1F;
				if(isSPS(nal_unit_type)){
					dummySink->sendSpsPacket(r[n].sPropBytes, r[n].sPropLength);
				} else if(isPPS(nal_unit_type)) {
					dummySink->sendPpsPacket(r[n].sPropBytes, r[n].sPropLength);
				}
			}
			delete[] r;
		}
	}
#ifdef DEBUG
	env << *rtspClient << "Created a data sink for the \"" << *scs.subsession << "\" subsession\n";
#endif
	scs.subsession->sink->startPlaying(*(scs.subsession->readSource()), subsessionAfterPlaying, scs.subsession);

	if (scs.subsession->rtcpInstance() != NULL) {
		scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
	}
} while (0);
delete[] resultString;

setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
	Boolean success = False;
	do {
		UsageEnvironment& env = rtspClient->envir(); // alias
		StreamClientState& scs = ((ourRTSPClient*) rtspClient)->scs; // alias

		if (resultCode != 0) {
			env << *rtspClient << "Failed to start playing session: " << resultString << "\n";
			break;
		}

		if (scs.duration > 0) {
			unsigned const delaySlop = 2;
			scs.duration += delaySlop;
			unsigned uSecsToDelay = (unsigned) (scs.duration * 1000000);
			scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*) streamTimerHandler, scs.subsession);
		}
#ifdef DEBUG
		env << *rtspClient << "Started playing session";
		if (scs.duration > 0) {
			env << " (for up to " << scs.duration << " seconds)";
		}
		env << "...\n";
#endif

		scs.checkAliveTimerTask = env.taskScheduler().scheduleDelayedTask(CHECK_ALIVE_TASK_TIMER_INTERVAL, (TaskFunc*) sendLivenessCommandHandler, rtspClient);
		announceStream(rtspClient);
		success = True;
	} while (0);
	delete[] resultString;

	if (!success) {
		shutdownStream(rtspClient);
	}
}

void shutdownStream(RTSPClient* rtspClient, int exitCode) {
	UsageEnvironment& env = rtspClient->envir();
	ourRTSPClient* client = (ourRTSPClient*)rtspClient;
	StreamClientState& scs = client->scs;
	char const* rtspUrl = strDup(rtspClient->url());
	char const* rtmpUrl = strDup(client->endpoint());
	
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

		Medium::close((ourRTMPClient*)client->publisher);
		client->publisher = NULL;
	}

	env << *rtspClient << "Closing the stream.\n";
	Medium::close(rtspClient);
/*
	if (--rtspClientCount == 0) {
		//exit(exitCode);
	}
*/
	openURL(env, rtspUrl, rtmpUrl);
}

void subsessionAfterPlaying(void* clientData) {
	MediaSubsession* subsession = (MediaSubsession*) clientData;
	RTSPClient* rtspClient = (RTSPClient*) (subsession->miscPtr);

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
	RTSPClient* rtspClient = (RTSPClient*) subsession->miscPtr;
	UsageEnvironment& env = rtspClient->envir(); // alias

	env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession << "\" subsession\n";
	subsessionAfterPlaying(subsession);
}

void sendLivenessCommandHandler(void* clientData) {
	ourRTSPClient* rtspClient = (ourRTSPClient*) clientData;
	StreamClientState& scs = rtspClient->scs;
	UsageEnvironment& env = rtspClient->envir(); // alias
	rtspClient->sendGetParameterCommand(*scs.session, NULL, NULL);
	scs.checkAliveTimerTask = env.taskScheduler().scheduleDelayedTask(CHECK_ALIVE_TASK_TIMER_INTERVAL, (TaskFunc*) sendLivenessCommandHandler, rtspClient);
}

void streamTimerHandler(void* clientData) {
	ourRTSPClient* rtspClient = (ourRTSPClient*)clientData;
	StreamClientState& scs = rtspClient->scs; 
	scs.streamTimerTask = NULL;
	shutdownStream(rtspClient);
}

void usage(UsageEnvironment& env) {
	env << "Usage: " << progName << " [-c <conf>]\n";
}

void announceStream(RTSPClient* rtspClient) {
	UsageEnvironment& env = rtspClient->envir();
	env << *rtspClient << "Publish the stream. endpoint:\"" << ((ourRTSPClient*) rtspClient)->endpoint() << "\"\n";
}

// Implementation of "ourRTSPClient":
ourRTSPClient* ourRTSPClient::createNew(UsageEnvironment& env, char const* rtspURL, char const* rtmpURL,
	int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum) {
	return new ourRTSPClient(env, rtspURL, rtmpURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment& env, char const* rtspURL, char const* rtmpURL, int verbosityLevel,
		char const* applicationName, portNumBits tunnelOverHTTPPortNum)
: RTSPClient(env, rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum, -1), publisher(NULL) {
	fDestUrl = strDup(rtmpURL);
}

ourRTSPClient::~ourRTSPClient() {
	delete[] fDestUrl;
}

// Implementation of "StreamClientState":
StreamClientState::StreamClientState()
: session(NULL), iter(NULL), subsession(NULL), streamTimerTask(NULL), checkAliveTimerTask(NULL), duration(0.0) {
}

StreamClientState::~StreamClientState() {
	delete iter;
	if (session != NULL) {
		UsageEnvironment& env = session->envir(); // alias
		env.taskScheduler().unscheduleDelayedTask(checkAliveTimerTask);
		checkAliveTimerTask = NULL; 
		env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
		Medium::close(session);
	}
}

// Implementation of "DummySink":
DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
	return  new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) 
: MediaSink(env), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0), fSubsession(subsession), fHaveWrittenFirstFrame(True) {
	fStreamId = strDup(streamId);
	fReceiveBuffer = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE];
	rtmpClient = (ourRTMPClient*)((ourRTSPClient*)subsession.miscPtr)->publisher;
}

DummySink::~DummySink() {
	delete[] fReceiveBuffer;
	delete[] fStreamId;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes,
	struct timeval presentationTime, unsigned durationInMicroseconds) {
	DummySink* sink = (DummySink*)clientData;
	sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then comment out the following line:
void DummySink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
		unsigned /*durationInMicroseconds*/) {

	u_int8_t nal_unit_type = fReceiveBuffer[4] & 0x1F; //0xFF;
	unsigned timestamp = presentationTime.tv_sec * 1000 + presentationTime.tv_usec / 1000;

	if (rtmpClient == NULL)
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
			if (!rtmpClient->sendH264FramePacket(fSps, fSpsSize, timestamp))
				goto RECONNECT;

			if (!rtmpClient->sendH264FramePacket(fPps, fPpsSize, timestamp))
				goto RECONNECT;

			fHaveWrittenFirstFrame = False;
		}
		goto NEXT;
	}

	if (strcasecmp(fSubsession.mediumName(), "video") == 0 && (isIDR(nal_unit_type) || isNonIDR(nal_unit_type))) {
		fReceiveBuffer[0] = 0;
		fReceiveBuffer[1] = 0;
		fReceiveBuffer[2] = 0;
		fReceiveBuffer[3] = 1;
		if (!rtmpClient->sendH264FramePacket(fReceiveBuffer, frameSize + 4, timestamp))
			goto RECONNECT;
	}
	goto NEXT;

RECONNECT:
	fHaveWrittenFirstFrame = True;
	rtmpClient = ourRTMPClient::createNew(envir(),(ourRTSPClient*) fSubsession.miscPtr);

NEXT:
	// Then continue, to request the next frame of data:
	continuePlaying();
}

Boolean DummySink::continuePlaying() {
	if (fSource == NULL)
		return False; // sanity check (should not happen)

	// Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
	fSource->getNextFrame(fReceiveBuffer + 4, DUMMY_SINK_RECEIVE_BUFFER_SIZE - 4, afterGettingFrame, this, onSourceClosure, this);
	return True;
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, RTSPClient* rtspClient) {
	ourRTMPClient* instance = new ourRTMPClient(env, rtspClient);
	return instance->connect() ? instance : NULL;
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient)
: Medium(env), rtmp(NULL), fTimestamp(0), dts(0), pts(0) {
	fSource = (ourRTSPClient*) rtspClient;
}

ourRTMPClient::~ourRTMPClient() {
	envir() << *fSource << "Cleanup when unpublish. client disconnect peer" << "\n";
	srs_rtmp_destroy(rtmp);
	fSource->publisher = NULL;
}

Boolean ourRTMPClient::connect() {
	do {
		rtmp = srs_rtmp_create(fSource->endpoint());

		if (srs_rtmp_handshake(rtmp) != 0) {
			envir() << *fSource << "simple handshake failed." << "\n";
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
			envir() << *fSource << "publish stream failed.(ret=" << ret
					<< ")\n";
			break;
		}
#ifdef DEBUG
		envir() << *fSource << "publish stream success" << "\n";
#endif
		fSource->publisher = this;
		return True;
	} while (0);

	Medium::close(this);
	return False;
}

Boolean ourRTMPClient::sendH264FramePacket(u_int8_t* data, unsigned size, unsigned timestamp) {
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
					break;
				}
			}
#ifdef DEBUG
			u_int8_t nut = data[4] & 0x1F;
			envir() << *fSource << "sent packet: type=video"
			<< ", time=" << dts << ", size=" << size
			<< ", b[4]=" << (unsigned char*)data[4] << "("
			<< (isSPS(nut)? "SPS" : (isPPS(nut) ? "PPS" : (isIDR(nut) ? "I" : (isNonIDR(nut) ? "P" : "Unknown")))) << ")\n";
#endif
		}
		return True;
	} while (0);

	Medium::close(this);
	return False;
}
