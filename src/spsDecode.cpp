
#include "spsDecode.h"

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

			int *offset_for_ref_frame =
					new int[num_ref_frames_in_pic_order_cnt_cycle];
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
		height = (2 - frame_mbs_only_flag)
				* (pic_height_in_map_units_minus1 + 1) * 16;

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

			width -= crop_unit_x
					* (frame_crop_left_offset + frame_crop_right_offset);
			height -= crop_unit_y
					* (frame_crop_top_offset + frame_crop_bottom_offset);
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

