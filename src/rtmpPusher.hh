
#ifndef RTMPPUSHER_HH_
#define RTMPPUSHER_HH_

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "GroupsockHelper.hh"
#include "srs_librtmp.h"
#include "EasyAACEncoderAPI.h"

#include "cJSON.h"
#include "ourMD5.hh"
#include "spsDecode.h"

#define CHECK_ALIVE_TASK_TIMER_INTERVAL 5*1000*1000

#define RECONNECT_WAIT_DELAY(n) if (n >= 3) { usleep(CHECK_ALIVE_TASK_TIMER_INTERVAL); n = 0; }

typedef struct {
	char const* srcStreamURL;	//rtsp url or file ptah
	char const* destStreamURL;	//rtmp url
	Authenticator* rtspAUTH;
	Boolean rtspUseTcp;
	Boolean audioTrack;
} conn_item_params_t, *conn_item_params_ptr;

//Only support nalu by rtmp protocol
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

UsageEnvironment* thatEnv;
char const* progName = NULL;
struct timeval timeNow;

class StreamClientState {
public:
	StreamClientState();
	virtual ~StreamClientState();
public:
	MediaSession* session;
	MediaSubsessionIterator* iter;
	MediaSubsession* subsession;
	struct timeval gettingLastFrameTime;
	TaskToken streamTimerTask;
	TaskToken checkAliveTimerTask;
	double duration;
};

class ourRTMPClient: public Medium {
public:
	static ourRTMPClient* createNew(UsageEnvironment& env, RTSPClient* rtspClient);
	static ourRTMPClient* createNew(UsageEnvironment& env, char const* rtmpUrl);
	char* url() const { return strDup(fUrl); }
protected:
	ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient);
	ourRTMPClient(UsageEnvironment& env, char const* rtmpUrl);
	virtual ~ourRTMPClient();
public:
	Boolean sendH264FramePacket(u_int8_t* data, unsigned size, u_int32_t pts) {
		do {
			if (NULL != data && size > 4) {
				int ret = srs_h264_write_raw_frames(rtmp, (char*) data, size, pts, pts);
				if (ret != 0) {
					if (srs_h264_is_dvbsp_error(ret)) {
						envir() << "[URL:\"" << fUrl << "\"]: " << "ignore drop video error, code=" << ret << "\n";
					} else if (srs_h264_is_duplicated_sps_error(ret)) {
						envir() << "[URL:\"" << fUrl << "\"]: " << "ignore duplicated sps, code=" << ret << "\n";
					} else if (srs_h264_is_duplicated_pps_error(ret)) {
						envir() << "[URL:\"" << fUrl << "\"]: " << "ignore duplicated pps, code=" << ret << "\n";
					} else {
						envir() << "[URL:\"" << fUrl << "\"]: " << "send h264 raw data failed. code=" << ret << "\n";
						break;
					}
				}
#ifdef DEBUG
				u_int8_t nut = data[4] & 0x1F;
				envir() << "[URL:\"" << fUrl << "\"]: " << "sent packet: type=video" << ", time=" << pts
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

	Boolean sendAACFramePacket(u_int8_t* data, unsigned size, u_int32_t pts, u_int8_t sound_size = 0, u_int8_t sound_type = 0) {
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
				int ret = srs_audio_write_raw_frame(rtmp, sound_format,
						sound_rate, sound_size, sound_type, (char*) data, size, pts);
				if (ret != 0) {
					envir() << "[URL:\"" << fUrl << "\"]: " << "send audio raw data failed. code=" << ret << "\n";
				}
#ifdef DEBUG
				envir() << "[URL:\"" << fUrl << "\"]: " <<"sent packet: type=audio" << ", time=" << pts
						<< ", size=" << size << ", codec=" << sound_format << ", rate=" << sound_rate
						<< ", sample=" << sound_size << ", channel=" << sound_type << "\n";
#endif
			}
			return True;
		} while (0);

		Medium::close(this);
		return False;
	}
private:
	srs_rtmp_t rtmp;
	RTSPClient* fSource;
	char const* fUrl;
};

class ourRTSPClient: public RTSPClient {
public:
	static ourRTSPClient* createNew(UsageEnvironment& env, conn_item_params_t& params);
protected:
	ourRTSPClient(UsageEnvironment& env, conn_item_params_t& params);
	virtual ~ourRTSPClient();
public:
	StreamClientState scs;
	ourRTMPClient* publisher;
	conn_item_params_t fParams;
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

	int getSamplingFrequencyIndex(unsigned samplingfrequeny) {
		switch(samplingfrequeny) {
			case 96000:
				return 0x0;
			case 88200:
				return 0x1;
			case 64000:
				return 0x2;
			case 48000:
				return 0x3;
			case 44100:
				return 0x4;
			case 32000:
				return 0x5;
			case 24000:
				return 0x6;
			case 22050:
				return 0x7;
			case 16000:
				return 0x8;
			case 12000:
				return 0x9;
			case 11025:
				return 0xa;
			case 8000:
				return 0xb;
			default:
				return -1;
		}
	}

	void checkComplexIDRFrame() {
		fIdrOffset = fSpsSize + fPpsSize;
		u_int8_t* idr = fReceiveBuffer + fIdrOffset;
		fIdrOffset = (idr[0] == 0 && idr[1] == 0 && idr[2] == 0 && idr[3] == 1 && isIDR(idr[4] & 0x1F)) ? fIdrOffset : 0;
	}

public:
	// redefined virtual functions:
	virtual Boolean continuePlaying();

	void parseSpsPacket(u_int8_t* data, unsigned size) {
		delete[] fSps;
		fSpsSize = size+4;
		fSps = new u_int8_t[fSpsSize];
		fSps[0] = 0; fSps[1] = 0;
		fSps[2] = 0; fSps[3] = 1;
		memmove(fSps+4, data, size);

		int width, height, fps;
		h264_decode_sps(data, size, width, height, fps);
		if (width >= fWidth || height >= fHeight) {
			fBufferSize = (fWidth = width) * (fHeight = height) * 2 / 8;
			delete[] fReceiveBuffer;
			fReceiveBuffer = new u_int8_t[fBufferSize];
			fFps = fps > 0 ? fps : 25;
		}
	}

	void parsePpsPacket(u_int8_t* data, unsigned size) {
		delete[] fPps;
		fPpsSize = size + 4;
		fPps = new u_int8_t[fPpsSize];
		fPps[0] = 0; fPps[1] = 0;
		fPps[2] = 0; fPps[3] = 1;
		memmove(fPps+4, data, size);
	}

	Boolean addADTStoPacket(u_int8_t* data, unsigned size) {
		int profile = 2;  //AAC LC
		int freqIdx = getSamplingFrequencyIndex(fSubsession.rtpTimestampFrequency());
		int chanCfg = fSubsession.numChannels();
		if (freqIdx == -1)
			return False;

		// fill in ADTS data
		data[0] = (u_int8_t)0xFF;
		data[1] = (u_int8_t)0xF9;
		data[2] = (u_int8_t)(((profile-1) << 6) + (freqIdx << 2) +(chanCfg >> 2));
		data[3] = (u_int8_t)(((chanCfg&3) << 6) + (size >> 11));
		data[4] = (u_int8_t)((size&0x7FF) >> 3);
		data[5] = (u_int8_t)(((size&7) << 5) + 0x1F);
		data[6] = (u_int8_t)0xFC;
		return True;
	}

private:
	u_int8_t* fSps;
	u_int8_t* fPps;
	unsigned fSpsSize;
	unsigned fPpsSize;
	u_int8_t* fReceiveBuffer;
	unsigned fBufferSize;
	char* fStreamId;
	MediaSubsession& fSubsession;
	int fWidth;
	int fHeight;
	int fFps;
	u_int64_t fPtsOffset;
	u_int8_t fIdrOffset;
	Boolean fWaitFirstFrameFlag;
private:
	EasyAACEncoder_Handle aacEncHandle;
	u_int8_t* fAACBuffer;
};

#endif /* RTMPPUSHER_HH_ */
