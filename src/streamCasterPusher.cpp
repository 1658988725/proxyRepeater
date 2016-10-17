#include <pthread.h>
#include "rtmpPusher.hh"
#include "cJSON.h"
#include "ourMD5.hh"

void afterPlaying(void* clientData);
void *readFileSource(void *args);
void usage(UsageEnvironment& env);

UsageEnvironment* thatEnv;
char const* progName = NULL;

#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 388800

int main(int argc, char** argv) {
	//OutPacketBuffer::maxSize = DUMMY_SINK_RECEIVE_BUFFER_SIZE;
	TaskScheduler* scheduler = BasicTaskScheduler::createNew();
	thatEnv = BasicUsageEnvironment::createNew(*scheduler);

	progName = argv[0];

	if (argc < 2) {
		usage(*thatEnv);
		exit(0);
	}

	int opt;
	char rtmpUrl[120] = { '\0' };
	char const* inputFileName;
	char* host;
	char* app;
	char* pwd;
	char* stream;

	while ((opt = getopt(argc, argv, "f:e:h:a:p:s:")) != -1) {
		switch (opt) {
			case 'f':
				inputFileName = optarg;
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
				else if (opt == 'p')
					pwd = optarg;
				else if (opt == 's')
					stream = optarg;
				break;
			default:
				usage(*thatEnv);
				exit(0);

		}
	}

	if(strlen(rtmpUrl) == 0)
		sprintf(rtmpUrl, "rtmp://%s/%s?%s/%s", host, app, pwd, stream);

	//*thatEnv << rtmpUrl << "\n";

	pthread_t cthread;
	pthread_attr_t attributes;
	//void *cthread_return;
	pthread_attr_init(&attributes);
	pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
	pthread_create(&cthread, NULL, readFileSource, (void*)inputFileName);

	if (pthread_join(cthread, NULL) != 0) //pthread_join(cthread, &cthread_return)
		exit(1);

	thatEnv->taskScheduler().doEventLoop();
	return 0;
}

void afterPlaying(void* clientData){
	DummyFileSink* sink = (DummyFileSink*)clientData;
	sink->stopPlaying();
	Medium::close(sink->source());
}

void *readFileSource(void *args) {
	char const* inputFileName = (char const*)args;
	FramedSource* fSource = ByteStreamFileSource::createNew(*thatEnv, inputFileName);
	if (fSource == NULL) {
		*thatEnv << "Unable to open file \"" << inputFileName << "\" as a byte-stream file source\n";
		exit(1);
	}

	H264VideoStreamFramer* framer = H264VideoStreamFramer::createNew(*thatEnv, fSource, True);
	DummyFileSink* sink = DummyFileSink::createNew(*thatEnv);
	sink->startPlaying(*framer, afterPlaying, sink);

	return (void*)EXIT_SUCCESS;
}

DummyFileSink* DummyFileSink::createNew(UsageEnvironment& env, char const* streamId) {
	return new DummyFileSink(env, streamId);
}

DummyFileSink::DummyFileSink(UsageEnvironment& env, char const* streamId) :
		MediaSink(env) {
	fStreamId = strDup(streamId);
	fBuffer = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE];
}

DummyFileSink::~DummyFileSink() {
	delete[] fBuffer; fBuffer = NULL;
	delete[] fStreamId; fStreamId = NULL;
}

Boolean DummyFileSink::continuePlaying() {
	if (fSource == NULL)
		return False;

	fSource->getNextFrame(fBuffer, DUMMY_SINK_RECEIVE_BUFFER_SIZE, afterGettingFrame, this, onSourceClosure, this);
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
	unsigned dts = 0;
	u_int8_t nut = fBuffer[4] & 0x1F;
	envir() << "sent packet: type=video" << ", time=" << dts << ", size="
	   << frameSize << ", b[4]=" << fBuffer[4] << "("
	   << (isSPS(nut) ? "SPS" : (isPPS(nut) ? "PPS" : (isIDR(nut) ? "I" : (isNonIDR(nut) ? "P" : "Unknown"))))
	   << ")\n";
	// Then try getting the next frame:
	continuePlaying();
}

void usage(UsageEnvironment& env) {
	env << "Usage: " << progName << " -f <fifo_name> <[-e <url> | -h <host[:port]> [-a app] [-p password] -s <stream>]>\n";
	env << "Options:" << "\n";
	env << " -f: fifo pathname" << "\n";
	env << " -e: publish endpoint url e.g: rtmp://host:port/app?nonce=&token=&/stream" << "\n";
	env << " -h: publish endpoint host e.g: 127.0.0.1:1935" << "\n";
	env << " -a: publish appname default: live" << "\n";
	env << " -p: publish authentication password" << "\n";
	env << " -s: publish streamname" << "\n";
}
