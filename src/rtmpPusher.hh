#ifndef RTMPPUSHER_HH_
#define RTMPPUSHER_HH_

#include <math.h>
#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "GroupsockHelper.hh"
#include "srs_librtmp.h"
#include "EasyAACEncoderAPI.h"

typedef unsigned int UINT;
typedef unsigned char BYTE;
typedef unsigned long DWORD;

#define CHECK_ALIVE_TASK_TIMER_INTERVAL 5*1000*1000

#define RECONNECT_WAIT_DELAY(n) if (n >= 3) { usleep(CHECK_ALIVE_TASK_TIMER_INTERVAL); n = 0; }

//forward
class DummySink;
int h264_decode_sps(BYTE* buf, unsigned int nLen, int &width, int &height, int &fps); //forward

typedef struct {
	char const* srcStreamURL;	//rtsp url or file ptah
	char const* destStreamURL;	//rtmp url
	Authenticator* rtspAUTH;
	Boolean rtspUseTcp;
	unsigned videoFps;
	Boolean audioTrack;
	char const* streamName;
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
protected:
	ourRTMPClient(UsageEnvironment& env, RTSPClient* rtspClient);
	ourRTMPClient(UsageEnvironment& env, char const* rtmpUrl);
	virtual ~ourRTMPClient();
public:
	Boolean sendH264FramePacket(u_int8_t* data, unsigned size, u_int32_t pts);
	Boolean sendAACFramePacket(u_int8_t* data, unsigned size, u_int32_t pts, u_int8_t sound_size = 0, u_int8_t sound_type = 0);
	char* url() const { return strDup(fUrl); }
	Boolean fWaitFirstFrameFlag;
private:
	srs_rtmp_t rtmp;
	RTSPClient* fSource;
	char const* fUrl;
};

UsageEnvironment& operator << (UsageEnvironment& env, const ourRTMPClient& rtmpClient) {
    return env << "[URL:\"" << rtmpClient.url() << "\"]: ";
}

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

	void setBufferSize(unsigned size) {
		fBufferSize = size;
		delete[] fReceiveBuffer;
		fReceiveBuffer = new u_int8_t[size];
	}

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
	ourRTMPClient* fClient;

	// redefined virtual functions:
	virtual Boolean continuePlaying();

	void setFps(unsigned fps) { fFps = fps; }

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
			fWidth = width;
			fHeight = height;
			fFps = fps;
			setBufferSize(width * height * 2 / 8);
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
	u_int32_t fPtsOffset;
	u_int8_t fIdrOffset;
private:
	EasyAACEncoder_Handle aacEncHandle;
	u_int8_t* fAACBuffer;
};

UINT Ue(BYTE *pBuff, UINT nLen, UINT &nStartBit) {
	UINT nZeroNum = 0;
	while (nStartBit < nLen * 8) {
		if (pBuff[nStartBit / 8] & (0x80 >> (nStartBit % 8)))
			break;
		nZeroNum++;
		nStartBit++;
	}
	nStartBit++;

	DWORD dwRet = 0;
	for (UINT i = 0; i < nZeroNum; i++) {
		dwRet <<= 1;
		if (pBuff[nStartBit / 8] & (0x80 >> (nStartBit % 8))) {
			dwRet += 1;
		}
		nStartBit++;
	}
	return (1 << nZeroNum) - 1 + dwRet;
}

int Se(BYTE *pBuff, UINT nLen, UINT &nStartBit) {
	int UeVal = Ue(pBuff, nLen, nStartBit);
	double k = UeVal;
	int nValue = ceil(k / 2);
	if (UeVal % 2 == 0)
		nValue = -nValue;
	return nValue;
}

DWORD u(UINT BitCount, BYTE * buf, UINT &nStartBit) {
	DWORD dwRet = 0;
	for (UINT i = 0; i < BitCount; i++) {
		dwRet <<= 1;
		if (buf[nStartBit / 8] & (0x80 >> (nStartBit % 8)))
			dwRet += 1;
		nStartBit++;
	}
	return dwRet;
}

void de_emulation_prevention(BYTE* buf, unsigned int* buf_size) {
	size_t i = 0, j = 0;
	BYTE* tmp_ptr = NULL;
	unsigned int tmp_buf_size = 0;
	int val = 0;
	tmp_ptr = buf;
	tmp_buf_size = *buf_size;
	for (i = 0; i < (tmp_buf_size - 2); i++) {
		val = (tmp_ptr[i] ^ 0x00) + (tmp_ptr[i + 1] ^ 0x00)
				+ (tmp_ptr[i + 2] ^ 0x03);
		if (val == 0) {
			for (j = i + 2; j < tmp_buf_size - 1; j++)
				tmp_ptr[j] = tmp_ptr[j + 1];
			(*buf_size)--;
		}
	}
}

int h264_decode_sps(BYTE * buf, unsigned int nLen, int &width, int &height, int &fps) {
	UINT StartBit = 0;
	int chroma_format_idc = 0;
	int frame_crop_left_offset;
	int frame_crop_right_offset;
	int frame_crop_top_offset;
	int frame_crop_bottom_offset;
	fps = 0;
	de_emulation_prevention(buf, &nLen);

	//forbidden_zero_bit =
	u(1, buf, StartBit);
	//nal_ref_idc =
	u(2, buf, StartBit);
	int nal_unit_type = u(5, buf, StartBit);
	if (nal_unit_type == 7) {
		int profile_idc = u(8, buf, StartBit);
		//constraint_set0_flag =
		u(1, buf, StartBit); //(buf[1] & 0x80)>>7;
		//constraint_set1_flag =
		u(1, buf, StartBit); //(buf[1] & 0x40)>>6;
		//constraint_set2_flag =
		u(1, buf, StartBit); //(buf[1] & 0x20)>>5;
		//constraint_set3_flag =
		u(1, buf, StartBit); //(buf[1] & 0x10)>>4;
		//reserved_zero_4bits =
		u(4, buf, StartBit);
		//level_idc =
		u(8, buf, StartBit);

		//seq_parameter_set_id =
		Ue(buf, nLen, StartBit);

		if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122
				|| profile_idc == 144) {
			chroma_format_idc = Ue(buf, nLen, StartBit);
			if (chroma_format_idc == 3)
				//residual_colour_transform_flag =
				u(1, buf, StartBit);
			//bit_depth_luma_minus8 =
			Ue(buf, nLen, StartBit);
			//bit_depth_chroma_minus8 =
			Ue(buf, nLen, StartBit);
			//qpprime_y_zero_transform_bypass_flag =
			u(1, buf, StartBit);

			int seq_scaling_matrix_present_flag = u(1, buf, StartBit);
			if (seq_scaling_matrix_present_flag) {
				for (int i = 0; i < 8; i++) {
					//seq_scaling_list_present_flag[i] =
					u(1, buf, StartBit);
				}
			}
		}
		//log2_max_frame_num_minus4 =
		Ue(buf, nLen, StartBit);
		int pic_order_cnt_type = Ue(buf, nLen, StartBit);
		if (pic_order_cnt_type == 0)
			//log2_max_pic_order_cnt_lsb_minus4 =
			Ue(buf, nLen, StartBit);
		else if (pic_order_cnt_type == 1) {
			//delta_pic_order_always_zero_flag =
			u(1, buf, StartBit);
			//offset_for_non_ref_pic =
			Se(buf, nLen, StartBit);
			//offset_for_top_to_bottom_field =
			Se(buf, nLen, StartBit);
			int num_ref_frames_in_pic_order_cnt_cycle = Ue(buf, nLen, StartBit);

			int *offset_for_ref_frame = new int[num_ref_frames_in_pic_order_cnt_cycle];
			for (int i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++)
				offset_for_ref_frame[i] = Se(buf, nLen, StartBit);
			delete[] offset_for_ref_frame;
		}

		//num_ref_frames =
		Ue(buf, nLen, StartBit);
		//gaps_in_frame_num_value_allowed_flag =
		u(1, buf, StartBit);
		int pic_width_in_mbs_minus1 = Ue(buf, nLen, StartBit);
		int pic_height_in_map_units_minus1 = Ue(buf, nLen, StartBit);

		int frame_mbs_only_flag = u(1, buf, StartBit);
		if (!frame_mbs_only_flag)
			//mb_adaptive_frame_field_flag =
			u(1, buf, StartBit);

		//direct_8x8_inference_flag =
		u(1, buf, StartBit);
		int frame_cropping_flag = u(1, buf, StartBit);
		if (frame_cropping_flag) {
			frame_crop_left_offset = Ue(buf, nLen, StartBit);
			frame_crop_right_offset = Ue(buf, nLen, StartBit);
			frame_crop_top_offset = Ue(buf, nLen, StartBit);
			frame_crop_bottom_offset = Ue(buf, nLen, StartBit);
		}

		width = (pic_width_in_mbs_minus1 + 1) * 16;
		height = (2 - frame_mbs_only_flag) * (pic_height_in_map_units_minus1 + 1) * 16;

		if (frame_cropping_flag) {
			unsigned int crop_unit_x;
			unsigned int crop_unit_y;
			if (0 == chroma_format_idc) {
				// monochrome
				crop_unit_x = 1;
				crop_unit_y = 2 - frame_mbs_only_flag;
			} else if (1 == chroma_format_idc) {
				// 4:2:0
				crop_unit_x = 2;
				crop_unit_y = 2 * (2 - frame_mbs_only_flag);
			} else if (2 == chroma_format_idc) {
				// 4:2:2
				crop_unit_x = 2;
				crop_unit_y = 2 - frame_mbs_only_flag;
			} else {
				// 3 == chroma_format_idc   // 4:4:4
				crop_unit_x = 1;
				crop_unit_y = 2 - frame_mbs_only_flag;
			}

			width -= crop_unit_x * (frame_crop_left_offset + frame_crop_right_offset);
			height -= crop_unit_y * (frame_crop_top_offset + frame_crop_bottom_offset);
		}

		int vui_parameter_present_flag = u(1, buf, StartBit);
		if (vui_parameter_present_flag) {
			int aspect_ratio_info_present_flag = u(1, buf, StartBit);
			if (aspect_ratio_info_present_flag) {
				int aspect_ratio_idc = u(8, buf, StartBit);
				if (aspect_ratio_idc == 255) {
					//sar_width =
					u(16, buf, StartBit);
					//sar_height =
					u(16, buf, StartBit);
				}
			}

			int overscan_info_present_flag = u(1, buf, StartBit);
			if (overscan_info_present_flag)
				//overscan_appropriate_flagu =
				u(1, buf, StartBit);

			int video_signal_type_present_flag = u(1, buf, StartBit);
			if (video_signal_type_present_flag) {
				//video_format =
				u(3, buf, StartBit);
				//video_full_range_flag =
				u(1, buf, StartBit);
				int colour_description_present_flag = u(1, buf, StartBit);
				if (colour_description_present_flag) {
					//colour_primaries =
					u(8, buf, StartBit);
					//transfer_characteristics =
					u(8, buf, StartBit);
					//matrix_coefficients =
					u(8, buf, StartBit);
				}
			}

			int chroma_loc_info_present_flag = u(1, buf, StartBit);
			if (chroma_loc_info_present_flag) {
				//chroma_sample_loc_type_top_field =
				Ue(buf, nLen, StartBit);
				//chroma_sample_loc_type_bottom_field =
				Ue(buf, nLen, StartBit);
			}

			int timing_info_present_flag = u(1, buf, StartBit);
			if (timing_info_present_flag) {
				int num_units_in_tick = u(32, buf, StartBit);
				int time_scale = u(32, buf, StartBit);
				fps = time_scale / (2 * num_units_in_tick);
			}
		}
		return true;
	}
	return false;
}
#endif /* RTMPPUSHER_HH_ */
