#include <pthread.h>

#include "rtmpPusher.hh"
#include "ourMD5.hh"

//#define DEBUG

conn_item_params_t params;

typedef struct {
	DummySink* sink;
	u_int8_t* data;
	unsigned size;
	u_int32_t pts;
	u_int32_t channels;
} send_frame_packet_t, *send_frame_packet_ptr;

send_frame_packet_t videoPacket;
send_frame_packet_t audioPacket;

//forward
void afterPlaying(void* clientData);
void *readFileSource(void *args);
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
					params.streamName = optarg;
				break;
			default:
				usage(*thatEnv);
				exit(0);

		}
	}

	if (params.streamName == NULL) {
		params.streamName = stream;
	}

	if (strlen(rtmpUrl) == 0) {
		sprintf(rtmpUrl, "rtmp://%s/%s%s/%s", host, app, token, params.streamName);
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
	MediaSubsession* subsession = (MediaSubsession*)clientData;
	ourRTMPClient* rtmpClient = ((DummySink*)subsession->sink)->fClient;

	//*thatEnv << subsession->mediumName() << "/" << subsession->codecName() << "\n";
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
	readFileSource((void*)&params);
}

void *readFileSource(void *args) {
	if (args == NULL)
		return (void*) EXIT_FAILURE;

	conn_item_params_ptr ptr = (conn_item_params_ptr)args;

	ourRTMPClient* rtmpClient = ourRTMPClient::createNew(*thatEnv, ptr->destStreamURL);
	if(rtmpClient == NULL) {
		*thatEnv << "ERROR: Publish the failed. endpoint:\"" << ptr->destStreamURL << "\n";
		return (void*) EXIT_FAILURE;
	}

	char const* sdpDescription =
			"v=0\nm=video 0 RTP/AVP 96\na=rtpmap:96 H264/90000\na=control:track1\nm=audio 0 RTP/AVP 0\na=rtpmap:0 MPA/44100/2\na=control:track2\n";

	MediaSession* session = MediaSession::createNew(*thatEnv, sdpDescription);
	if (session == NULL) {
		*thatEnv << *rtmpClient << "ERROR: Unable create MediaSession\n";
		return (void*) EXIT_FAILURE;
	}

	if (session != NULL) {
		MediaSubsessionIterator iter(*session);
		MediaSubsession* subsession;

		while ((subsession = iter.next()) != NULL) {
			//*thatEnv << mss->mediumName() << "/" << mss->codecName() << "\n";
			if (strcasecmp(subsession->mediumName(), "video") == 0 && strcasecmp(subsession->codecName(), "H264") == 0) {
				ByteStreamFileSource* videoSource = ByteStreamFileSource::createNew(*thatEnv, ptr->srcStreamURL);
				if (videoSource == NULL) {
					*thatEnv << *rtmpClient << "WARN: Unable to open file \"" << ptr->srcStreamURL << "\" as a byte-stream file video source\n";
					continue;
				}

				subsession->addFilter(H264VideoStreamFramer::createNew(*thatEnv, videoSource, True));
				subsession->sink = DummySink::createNew(*thatEnv, *subsession, ptr->destStreamURL);

				DummySink* sink = (DummySink*)subsession->sink;
				sink->fClient = rtmpClient;
				if (ptr->videoFps > 0) {
					sink->setFps(ptr->videoFps);
				}
				sink->startPlaying(*(subsession->readSource()), afterPlaying, subsession);

			} else if (strcasecmp(subsession->mediumName(), "audio") == 0 && strcasecmp(subsession->codecName(), "MPA") == 0) {
				char tmp[30];
				sprintf(tmp, "objs/%s.aac", ptr->streamName);
				ADTSAudioFileSource* audioSource = ADTSAudioFileSource::createNew(*thatEnv, tmp);
				if (audioSource == NULL) {
					*thatEnv << *rtmpClient << "WARN: Unable to open file \"" << tmp << "\" as a adts audio file source\n";
					continue;
				}

				subsession->addFilter((FramedFilter*)audioSource);
				subsession->sink = DummySink::createNew(*thatEnv, *subsession, ptr->destStreamURL);

				DummySink* sink = (DummySink*)subsession->sink;
				sink->fClient = rtmpClient;
				sink->setFps(1000000 / subsession->rtpTimestampFrequency());
				sink->startPlaying(*(subsession->readSource()), afterPlaying, subsession);
			}
		}
	}

	return (void*)EXIT_SUCCESS;
}

//Implementation of "DummyFileSink":
DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
	return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
	: MediaSink(env), fSps(NULL), fPps(NULL), fSpsSize(0), fPpsSize(0), fReceiveBuffer(NULL),
	  fSubsession(subsession), fWidth(640), fHeight(480), fFps(25), fIdrOffset(0) {
	fStreamId = strDup(streamId);
	setBufferSize(fWidth * fHeight * 2 / 8);

	gettimeofday(&timeNow, NULL);
	fPtsOffset = u_int32_t(timeNow.tv_sec * 1000000 + timeNow.tv_usec);
	if (strcasecmp(fSubsession.mediumName(), "audio") == 0) {
		//envir() << fSubsession.rtpTimestampFrequency() << "\t" << fSubsession.numChannels() << "\t" << fFps << "\n";
		unsigned nPCMBitSize = 16;
		DWORD nMaxInputBytes = 1024 * fSubsession.numChannels() * nPCMBitSize / 8;
		DWORD nMaxOutputBytes = (6144/8) * fSubsession.numChannels();
		if (strcasecmp(fSubsession.codecName(), "MPEG4-GENERIC") == 0
				|| strcasecmp(fSubsession.codecName(), "MPA") == 0) {
			setBufferSize(nMaxInputBytes);
			fAACBuffer = new BYTE[nMaxOutputBytes];
		} else {
			switch(fSubsession.rtpPayloadFormat()) {
				case 0:	//PCMU
				case 8:	//PCMA
				case 2: //G726-32
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
						setBufferSize(nMaxInputBytes);
						fAACBuffer = new BYTE[nMaxOutputBytes];
					}
					break;
				default:
					break;
			}
		}
	}
}

DummySink::~DummySink() {
	delete[] fSps;
	delete[] fPps;
	delete[] fReceiveBuffer;
	delete[] fStreamId;
	if (aacEncHandle != NULL) {
		Easy_AACEncoder_Release(aacEncHandle);
		delete[] fAACBuffer;
	}
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
	if (fClient == NULL)
		goto RECONNECT;

	if (strcasecmp(fSubsession.mediumName(), "video") == 0) {
		if (strcasecmp(fSubsession.codecName(), "H264") == 0) {
			u_int8_t nal_unit_type = fReceiveBuffer[4] & 0x1F; //0xFF;

			if (fClient->fWaitFirstFrameFlag) {
				if (isSPS(nal_unit_type)) {
					parseSpsPacket(fReceiveBuffer+4, frameSize);
				} else if (isPPS(nal_unit_type)) {
					parsePpsPacket(fReceiveBuffer+4, frameSize);
				} else if (isIDR(nal_unit_type)) {
					if (!fClient->sendH264FramePacket(fSps, fSpsSize, 0))
						goto RECONNECT;

					if (!fClient->sendH264FramePacket(fPps, fPpsSize, 0))
						goto RECONNECT;

					checkComplexIDRFrame();

					if (!fClient->sendH264FramePacket(fReceiveBuffer+fIdrOffset, frameSize+4-fIdrOffset, 0))
						goto RECONNECT;

					fClient->fWaitFirstFrameFlag = False;
				}
				goto NEXT_FRAME;
			} else {
				videoPacket.sink = this;
				videoPacket.data = isIDR(nal_unit_type) ? fReceiveBuffer+fIdrOffset : fReceiveBuffer;
				videoPacket.size = isIDR(nal_unit_type) ? frameSize-fIdrOffset : frameSize;
				gettimeofday(&timeNow, NULL);
				videoPacket.pts = (u_int32_t(timeNow.tv_sec * 1000000 + timeNow.tv_usec) - fPtsOffset) / 1000;
				videoPacket.channels = 0;
				envir().taskScheduler().scheduleDelayedTask(1000 / fFps * 1000, (TaskFunc*)sendFramePacket, &videoPacket);
			}
		}
	} else if (strcasecmp(fSubsession.mediumName(), "audio") == 0 && !fClient->fWaitFirstFrameFlag) {
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
		audioPacket.pts = (u_int32_t(timeNow.tv_sec * 1000000 + timeNow.tv_usec) - fPtsOffset) / 1000;
		audioPacket.channels = fSubsession.numChannels();
		envir().taskScheduler().scheduleDelayedTask(fFps * 1000, (TaskFunc*)sendFramePacket, &audioPacket);
	}

	return;

RECONNECT:
	fClient = ourRTMPClient::createNew(envir(), fStreamId);
NEXT_FRAME:
	continuePlaying();
}

//Implementation of "ourRTMPClient":
ourRTMPClient* ourRTMPClient::createNew(UsageEnvironment& env, char const* rtmpUrl) {
	ourRTMPClient* instance = new ourRTMPClient(env, rtmpUrl);
	return instance->rtmp != NULL ? instance : NULL;
}

ourRTMPClient::ourRTMPClient(UsageEnvironment& env, char const* rtmpUrl)
	: Medium(env), fWaitFirstFrameFlag(True), rtmp(NULL), fSource(NULL), fUrl(rtmpUrl) {
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

Boolean ourRTMPClient::sendH264FramePacket(u_int8_t* data, unsigned size, u_int32_t pts) {
	do {
		if (NULL != data && size > 4) {
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
#ifdef DEBUG
			u_int8_t nut = data[4] & 0x1F;
			envir() << *this << "sent packet: type=video" << ", time=" << pts
			<< ", size=" << size << ", b[4]="
			<< (unsigned char*) data[4] << "("
			<< (isSPS(nut) ? "SPS" : (isPPS(nut) ? "PPS" : (isIDR(nut) ? "I" : (isNonIDR(nut) ? "P" : "Unknown"))))
			<< ")\n";
#endif
		}
		return True;
	} while (0);

	Medium::close(this);
	return False;
}

Boolean ourRTMPClient::sendAACFramePacket(u_int8_t* data, unsigned size, u_int32_t pts, u_int8_t sound_size, u_int8_t sound_type) {
	do {
		if (NULL != data && size > 0) {

			//sound_format:   0 = Linear PCM, platform endian
	        //				  1 = ADPCM
	        //                2 = MP3
	        //                7 = G.711 A-law logarithmic PCM
	        //                8 = G.711 mu-law logarithmic PCM
	        //                10 = AAC
	        //                11 = Speex
			char sound_format = 10;
			//sound_rate: 0 = 5.5kHz(?auto)  1 = 11kHz 2 = 22kHz 3 = 44kHz
			char sound_rate = 0;
			//sound_size: 0 = 8-bit samples  1 = 16-bit samples
			//sound_type: 0 = Mono sound  1 = Stereo sound
			int ret = srs_audio_write_raw_frame(rtmp, sound_format, sound_rate, sound_size, sound_type, (char*) data, size, pts);
			if (ret != 0) {
				envir() << *this << "send audio raw data failed. code=" << ret << "\n";
			}
#ifdef DEBUG
		envir() << *this <<"sent packet: type=audio" << ", time=" << pts
			<< ", size=" << size << ", codec=" << sound_format << ", rate=" << sound_rate
			<< ", sample=" << sound_size << ", channel=" << sound_type << "\n";
#endif
		}
		return True;
	} while (0);

	Medium::close(this);
	return False;
}

void sendFramePacket(void* clientData) {
	send_frame_packet_ptr pkt = (send_frame_packet_ptr)clientData;
	if (pkt->channels == 0) {
		u_int8_t nut = pkt->data[4] & 0x1F;
		if (isIDR(nut) || isNonIDR(nut)) {
			if (!pkt->sink->fClient->sendH264FramePacket(pkt->data, pkt->size, pkt->pts)) {

			}
		}
	} else {
		if (!pkt->sink->fClient->sendAACFramePacket(pkt->data, pkt->size, pkt->pts, 1, pkt->channels-1)) {

		}
	}
	pkt->sink->continuePlaying();
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
	env << " -s: set endpoint streamname default: demo" << "\n";
}
