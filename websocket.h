/*
 * Copyright (c) 2014 Putilov Andrey
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef WEBSOCKET_H
#define	WEBSOCKET_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <assert.h>
#include <stdint.h> /* uint8_t */
#include <stdlib.h> /* strtoul */
#include <netinet/in.h> /*htons*/
#include <string.h>
#include <stdio.h> /* sscanf */
#include <ctype.h> /* isdigit */

#define WS_MAX_URI_LENGTH 2048
#define WS_MAX_PARAM_LEN 16
#define WS_MAX_VALUE_LEN 32
#define WS_MAX_PARAM_NO   5

/*
 * OPTIONS /signalk/api/v2/vessels/self HTTP/1.1
 *   Host: localhost:2947
 *   User-Agent: Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:31.0) Gecko/20100101 Firefox/31.0
 *   Accept: text/html,application/xhtml+xml,application/xml
 *   Accept-Language: en-US,en;q=0.5\x0d\x0aAccept-Encoding: gzip, deflate\x0d\x0a
 *   Origin: http://localhost\x0d\x0a
 *   Access-Control-Request-Method: GET\x0d\x0a
 *   Access-Control-Request-Headers: content-type\x0d\x0a
 *   Connection: keep-alive\x0d\x0a\x0d\x0a
 */
static const char WS_HEADER_CONNECTION[] = "Connection: ";
static const char WS_HEADER_UPGRADE[]    = "Upgrade: ";

static const char hostField[]        = "Host: ";
static const char originField[]      = "Origin: ";
static const char keyField[]         = "Sec-WebSocket-Key: ";
static const char protocolField[]    = "Sec-WebSocket-Protocol: ";
static const char versionField[]     = "Sec-WebSocket-Version: ";
static const char version[]          = "13";
static const char secret[]           = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

enum wsFrameType { 
    WS_EMPTY_FRAME       = 0xF0,
    WS_ERROR_FRAME       = 0xF1,
    WS_INCOMPLETE_FRAME  = 0xF2,
    WS_TEXT_FRAME        = 0x01,
    WS_BINARY_FRAME      = 0x02,
    WS_PING_FRAME        = 0x09,
    WS_PONG_FRAME        = 0x0A,
    WS_OPENING_FRAME     = 0xF3,
    WS_GET_FRAME         = 0xF4,
    WS_CLOSING_FRAME     = 0x08,
    WS_PREFLIGHTED_FRAME = 0x0B
};
    
enum wsState {
    WS_STATE_OPENING,
    WS_STATE_NORMAL,
    WS_STATE_CLOSING
};

    struct ws_param_t {
        char param[WS_MAX_URI_LENGTH];
        char value[WS_MAX_URI_LENGTH];
    };

struct handshake {
    char host[WS_MAX_URI_LENGTH];
    char origin[WS_MAX_URI_LENGTH];
    char key[WS_MAX_URI_LENGTH];
    char protocol[WS_MAX_URI_LENGTH];
    char resource[WS_MAX_URI_LENGTH];
    struct ws_param_t params[WS_MAX_PARAM_NO];
    enum wsFrameType frameType;
};


    /**
     * @param inputFrame Pointer to input frame
     * @param inputLength Length of input frame
     * @param hs Cleared with nullHandshake() handshake structure
     * @return Type of parsed frame
     */
    enum wsFrameType wsParseHandshake(const uint8_t *inputFrame, size_t inputLength,
                                      struct handshake *hs);
	
    /**
     * @param hs Filled handshake structure
     * @param outFrame Pointer to frame buffer
     * @param outLength Length of frame buffer. Return length of out frame
     */
    void wsGetHandshakeAnswer(const struct handshake *hs, uint8_t *outFrame,
                              size_t *outLength);

    /**
     * @param data Pointer to input data array
     * @param dataLength Length of data array
     * @param outFrame Pointer to frame buffer
     * @param outLength Length of out frame buffer. Return length of out frame
     * @param frameType [WS_TEXT_FRAME] frame type to build
     */
    void wsMakeFrame(const char *data, size_t dataLength,
                     uint8_t *outFrame, size_t *outLength, enum wsFrameType frameType);

    /**
     *
     * @param inputFrame Pointer to input frame. Frame will be modified.
     * @param inputLen Length of input frame
     * @param outDataPtr Return pointer to extracted data in input frame
     * @param outLen Return length of extracted data
     * @return Type of parsed frame
     */
    enum wsFrameType wsParseInputFrame(const uint8_t *inputFrame, const size_t inputLength,
                                       uint8_t **dataPtr, size_t *dataLength);

    /**
     * @param hs NULL handshake structure
     */
    void nullHandshake(struct handshake *hs);

    /**
     * @param hs free and NULL handshake structure
     */
    void freeHandshake(struct handshake *hs);

#ifdef	__cplusplus
}
#endif

#endif	/* WEBSOCKET_H */
