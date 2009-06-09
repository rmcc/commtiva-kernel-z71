/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ADSP_AUDIO_ERROR_H
#define __ADSP_AUDIO_ERROR_H


/* success, i.e. no error */
#define  ADSP_AUDIO_SUCCESS				0



/* ADSP Audio Error Codes */


/* object already loaded */
#define  ADSP_AUDIO_EALREADYLOADED			5

/* unable to unload object */
#define  ADSP_AUDIO_EUNABLETOUNLOAD			7

/* invalid time */
#define  ADSP_AUDIO_EINVALIDTIME			9

/* invalid metric specified */
#define  ADSP_AUDIO_EBADMETRIC				11

/* invalid state */
#define  ADSP_AUDIO_EBADSTATE				13

/* invalid parameter */
#define  ADSP_AUDIO_EBADPARM				14

/* invalid item */
#define  ADSP_AUDIO_EBADITEM				16

/* invalid format */
#define  ADSP_AUDIO_EINVALIDFORMAT			17

/* incomplete item */
#define  ADSP_AUDIO_EINCOMPLETEITEM			18

/* module left memory allocated when released */
#define  ADSP_AUDIO_EALLOCATED				25

/* operation is already in progress */
#define  ADSP_AUDIO_EALREADY				26

/* bad memory pointer */
#define  ADSP_AUDIO_EMEMPTR				29

/* context (system, interface, etc.) is idle */
#define  ADSP_AUDIO_EIDLE				31

/* context (system, interface, etc.) is busy */
#define  ADSP_AUDIO_EITEMBUSY				32

/* no type detected/found */
#define  ADSP_AUDIO_ENOTYPE				34

/* need more data/info */
#define  ADSP_AUDIO_ENEEDMORE				35

/* destination buffer given is too small */
#define  ADSP_AUDIO_EBUFFERTOOSMALL			38

/* no such name/port/socket/service exists or valid */
#define  ADSP_AUDIO_ENOSUCH				39

/* ACK pending on application */
#define  ADSP_AUDIO_EACKPENDING				40

/* current item is invalid */
#define  ADSP_AUDIO_EINVALIDITEM			42

/* invalid handle */
#define  ADSP_AUDIO_EBADHANDLE				44

/* waitable call is interrupted */
#define  ADSP_AUDIO_EINTERRUPTED			46

/* no more items available - reached end */
#define  ADSP_AUDIO_ENOMORE				47

/* Cannot change read-only object or parameter */
#define  ADSP_AUDIO_EREADONLY				49





/* Fatal, non recoverable errors */


/* general failure */
#define  ADSP_AUDIO_EFAILED				-1

/* insufficient RAM */
#define  ADSP_AUDIO_ENOMEMORY				2

/* specified class unsupported */
#define  ADSP_AUDIO_ECLASSNOTSUPPORT			3

/* version not supported */
#define  ADSP_AUDIO_EVERSIONNOTSUPPORT			4

/* unable to load object */
#define  ADSP_AUDIO_EUNABLETOLOAD			6

/* NULL class object */
#define  ADSP_AUDIO_EBADCLASS				10

/* component expired */
#define  ADSP_AUDIO_EEXPIRED				12

/* API is not supported */
#define  ADSP_AUDIO_EUNSUPPORTED			20

/* privileges are insufficient for this operation */
#define  ADSP_AUDIO_EPRIVLEVEL				21

/* unable to find specified resource */
#define  ADSP_AUDIO_ERESOURCENOTFOUND			22

/* non re-entrant API re-entered */
#define  ADSP_AUDIO_EREENTERED				23

/* API called in wrong task context */
#define  ADSP_AUDIO_EBADTASK				24

/* heap corruption */
#define  ADSP_AUDIO_EHEAP				30

/* driver failed to close properly */
#define  ADSP_AUDIO_EBADSHUTDOWN			37

/* not an owner authorized to perform the operation */
#define  ADSP_AUDIO_ENOTOWNER				41

/* not allowed to perform the operation */
#define  ADSP_AUDIO_ENOTALLOWED				43

/* out of handles */
#define  ADSP_AUDIO_EOUTOFHANDLES			45

/* a CPU exception occurred */
#define  ADSP_AUDIO_ECPUEXCEPTION			48

#endif


