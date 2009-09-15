/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Code Aurora nor
*       the names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior written
*       permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef QDSP5AUDPLAYMSG_H
#define QDSP5AUDPLAYMSG_H

#define AUDPLAY_MSG_DEC_NEEDS_DATA		0x0001
#define AUDPLAY_MSG_DEC_NEEDS_DATA_MSG_LEN	\
	sizeof(audplay_msg_dec_needs_data)

struct audplay_msg_dec_needs_data {
	/* reserved*/
	unsigned int dec_id;

	/*The read pointer offset of external memory till which bitstream
	has been dmed in*/
	unsigned int adecDataReadPtrOffset;

	/*The buffer size of external memory. */
	unsigned int adecDataBufSize;

	unsigned int 	bitstream_free_len;
	unsigned int	bitstream_write_ptr;
	unsigned int	bitstarem_buf_start;
	unsigned int	bitstream_buf_len;
} __attribute__((packed));

#define AUDPLAY_UP_STREAM_INFO 0x0003
#define AUDPLAY_UP_STREAM_INFO_LEN \
	sizeof(struct audplay_msg_stream_info)

struct audplay_msg_stream_info {
	unsigned int decoder_id;
	unsigned int channel_info;
	unsigned int sample_freq;
	unsigned int bitstream_info;
	unsigned int bit_rate;
} __attribute__((packed));

#define AUDPLAY_MSG_BUFFER_UPDATE 0x0004
#define AUDPLAY_MSG_BUFFER_UPDATE_LEN \
	sizeof(struct audplay_msg_buffer_update)

struct audplay_msg_buffer_update {
	unsigned int buffer_write_count;
	unsigned int num_of_buffer;
	unsigned int buf0_address;
	unsigned int buf0_length;
	unsigned int buf1_address;
	unsigned int buf1_length;
} __attribute__((packed));
#endif /* QDSP5AUDPLAYMSG_H */
