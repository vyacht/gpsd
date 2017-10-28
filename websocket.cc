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

#include "websocket.h"
#include "bsd_base64.h"
#include "aw-sha1.h"

static char rn[] = "\r\n";

void nullHandshake(struct handshake *hs)
{
    uint32_t p = 0;

    hs->host[0] = '\0';
    hs->origin[0] = '\0';
    hs->key[0] = '\0';
    hs->protocol[0] = '\0';
    hs->resource[0] = '\0';
    hs->frameType = WS_EMPTY_FRAME;
    for(p = 0; p < WS_MAX_PARAM_NO; p++) {
        hs->params[p].param[0] = '\0';
        hs->params[p].value[0] = '\0';
    }
}

void freeHandshake(struct handshake *hs)
{
    nullHandshake(hs);
}

static char* copyToLinefeed(char * dest, const char *startFrom)
{
    uint16_t len = strstr(startFrom, rn) - startFrom;
    if(len > WS_MAX_URI_LENGTH - 1) 
        len = WS_MAX_URI_LENGTH - 1;
    memcpy(dest, startFrom, len);
    dest[ len ] = '\0';
    return dest;
}

/*
 * parses a resource string's parameters
 */
static void 
ws_parse_resource_uri(struct handshake *hs)
{
    uint32_t pcnt = 0;
    char * p= hs->resource;
    if(p[0] != '\/')
        return;

    // search for parameters
    // we know it starts with /, so p++ in do is safe
    do { p++; } while(('\0' != *p) && ('?' != *p));
    char * s = p;
    typedef enum {ps_param, ps_value} ps_t;
    ps_t ps = ps_param;
    while(('\0' != *p) && (pcnt < WS_MAX_PARAM_NO)) {
        if(('&' == *p) && (ps == ps_value)) {
            strncpy(hs->params[pcnt].value, s+1, p-s); 
            hs->params[pcnt].value[p-s-1] = '\0';
            s = p;
            ps = ps_param;
            pcnt++;
        } else if((ps == ps_param) && (('=' == *p) || ('&' == *p))) {
            strncpy(hs->params[pcnt].param, s+1, p-s); 
            hs->params[pcnt].param[p-s-1] = '\0';
            s = p;
            if('=' == *p) 
                ps = ps_value;
            else {
                ps = ps_param;
                pcnt++;
            }
        }
        p++;
    } 
    if((ps == ps_value) && (p>s)) {
        strncpy(hs->params[pcnt].value, s+1, p-s); 
    } else if((ps == ps_param) && (p>s)) {
        strncpy(hs->params[pcnt].param, s+1, p-s); 
    }
}
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
enum wsFrameType wsParseHandshake(const uint8_t *inputFrame, size_t inputLength,
                                  struct handshake *hs)
{
    enum wsFrameType tmpFT = WS_ERROR_FRAME;

    const char *inputPtr = (const char *)inputFrame;
    char *endPtr = (char *)inputFrame;
    while(*endPtr != '\0') {
      endPtr++;
    }

    if (!strstr((const char *)inputFrame, "\r\n\r\n"))
        return WS_INCOMPLETE_FRAME;
	
    if(memcmp(inputFrame, "GET ", 4) == 0)
        tmpFT = WS_OPENING_FRAME;
    else if(memcmp(inputFrame, "OPTIONS ", 8) == 0)
        tmpFT = WS_PREFLIGHTED_FRAME;
    else 
        return WS_ERROR_FRAME;

    // measure resource size
    char *first = strchr((const char *)inputFrame, ' ');
    if (!first)
        return WS_ERROR_FRAME;
    first++;
    char *second = strchr(first, ' ');
    if (!second) 
        return WS_ERROR_FRAME;
    if(second - first + 1 > WS_MAX_URI_LENGTH)
        return WS_ERROR_FRAME;

    if(sscanf(first, "%s HTTP/1.1\r\n", hs->resource) != 1)
        return WS_ERROR_FRAME;

    ws_parse_resource_uri(hs);

    inputPtr = strstr(inputPtr, rn) + 2;

    /*
        parse next lines
     */
    #define prepare(x) do {if (x) { x[0] = '\0'; }} while(0)
    #define strtolower(x) do { int i; for (i = 0; x[i]; i++) x[i] = tolower(x[i]); } while(0)

    uint8_t connectionFlag  = 0;
    uint8_t upgradeFlag     = 0;
    uint8_t subprotocolFlag = 0;
    uint8_t versionMismatch = 0;

    while (inputPtr < endPtr && inputPtr[0] != '\r' && inputPtr[1] != '\n') {

        if (memcmp(inputPtr, WS_HEADER_UPGRADE, strlen(WS_HEADER_UPGRADE)) == 0) {

            inputPtr += strlen(WS_HEADER_UPGRADE);
            char compare[WS_MAX_URI_LENGTH];
            copyToLinefeed(compare, inputPtr);
            strtolower(compare);
            assert(compare);
            if (memcmp(compare, "websocket", strlen("websocket")) == 0)
                upgradeFlag = 1;

        } else if (memcmp(inputPtr, WS_HEADER_CONNECTION, strlen(WS_HEADER_CONNECTION)) == 0) {

            inputPtr += strlen(WS_HEADER_CONNECTION);
            char connectionValue[WS_MAX_URI_LENGTH];
            copyToLinefeed(connectionValue, inputPtr);
            strtolower(connectionValue);
            assert(connectionValue);
            if (strstr(connectionValue, "upgrade") != NULL)
                connectionFlag = 1;

        } else if (memcmp(inputPtr, hostField, strlen(hostField)) == 0) {

            inputPtr += strlen(hostField);
            prepare(hs->host);
            copyToLinefeed(hs->host, inputPtr);

        } else if (memcmp(inputPtr, originField, strlen(originField)) == 0) {

            inputPtr += strlen(originField);
            prepare(hs->origin);
            copyToLinefeed(hs->origin, inputPtr);

        } else if (memcmp(inputPtr, protocolField, strlen(protocolField)) == 0) {

            inputPtr += strlen(protocolField);
            subprotocolFlag = 1;
            copyToLinefeed(hs->protocol, inputPtr);

        } else if (memcmp(inputPtr, keyField, strlen(keyField)) == 0) {

            inputPtr += strlen(keyField);
            prepare(hs->key);
            copyToLinefeed(hs->key, inputPtr);

        } else if (memcmp(inputPtr, versionField, strlen(versionField)) == 0) {

            inputPtr += strlen(versionField);
            char versionString[WS_MAX_URI_LENGTH];
            copyToLinefeed(versionString, inputPtr);
            if (memcmp(versionString, version, strlen(version)) != 0)
                versionMismatch = 1;

        } 

        inputPtr = strstr(inputPtr, rn) + 2;
    }

    // we have read all data, so check them
    if(hs->host[0] == '\0')
        printf("no host!\n");
    if(hs->key[0] == '\0')
        printf("no key!\n");
    else
      printf("key: %s\n", hs->key);
    if(!connectionFlag)
        printf("no connectionFlag!\n");
    if(!upgradeFlag)
        printf("no upgrade flag!\n");
    if(subprotocolFlag)
        printf("subprotocol: %s\n", protocolField);
    if(versionMismatch)
        printf("version mismatch!\n");

    // we have read all data, so check them
    if(tmpFT == WS_OPENING_FRAME) {
        if(!hs->host || !hs->key || !connectionFlag || !upgradeFlag || versionMismatch)  {
            hs->frameType = WS_GET_FRAME;
        } else {
            hs->frameType = WS_OPENING_FRAME;
        }
    } else if(tmpFT == WS_PREFLIGHTED_FRAME) {
            hs->frameType = WS_PREFLIGHTED_FRAME;
    } 
    
    return hs->frameType;
}

static const char encode[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
			     "abcdefghijklmnopqrstuvwxyz0123456789+/";
static const char decode[] = "|$$$}rstuvwxyz{$$$$$$$>?@ABCDEFGHIJKLMNOPQRSTUVW"
			     "$$$$$$XYZ[\\]^_`abcdefghijklmnopq";

int
b64_encode_string(const char *in, int in_len, char *out, int out_size)
{
	unsigned char triple[3];
	int i;
	int len;
	int line = 0;
	int done = 0;

	while (in_len) {
		len = 0;
		for (i = 0; i < 3; i++) {
			if (in_len) {
				triple[i] = *in++;
				len++;
				in_len--;
			} else
				triple[i] = 0;
		}

		if (done + 4 >= out_size)
			return -1;

		*out++ = encode[triple[0] >> 2];
		*out++ = encode[((triple[0] & 0x03) << 4) |
					     ((triple[1] & 0xf0) >> 4)];
		*out++ = (len > 1 ? encode[((triple[1] & 0x0f) << 2) |
					     ((triple[2] & 0xc0) >> 6)] : '=');
		*out++ = (len > 2 ? encode[triple[2] & 0x3f] : '=');

		done += 4;
		line += 4;
	}

	if (done + 1 >= out_size)
		return -1;

	*out++ = '\0';

	return done;
}

void wsGetHandshakeAnswer(const struct handshake *hs, uint8_t *outFrame, 
                          size_t *outLength)
{
    assert(outFrame);
    assert(*outLength);
    assert(hs->frameType == WS_OPENING_FRAME);
    assert(hs && hs->key);

    printf("handshake answer in key: %s\n", hs->key);

    char *responseKey = NULL;
    uint8_t length = strlen(hs->key)+strlen(secret);
    responseKey = malloc(length);
    memcpy(responseKey, hs->key, strlen(hs->key));
    memcpy(&(responseKey[strlen(hs->key)]), secret, strlen(secret));
    unsigned char shaHash[20];
    memset(shaHash, 0, sizeof(shaHash));
    sha1(shaHash, responseKey, length);

    size_t base64Length = b64_encode_string(shaHash, 20, responseKey, length);

    // size_t base64Length = base64(responseKey, length, shaHash, 20);
    responseKey[base64Length] = '\0';

    printf("handshake answer out key: %s (target length = %d, len = %d)\n", 
	   responseKey, length, base64Length);

    (void)snprintf((char *)outFrame, outLength,
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n");

    if(hs->protocol[0] != '\0')
        (void)snprintf((char *)outFrame + strlen(outFrame), 
        outLength - strlen(outFrame),
        "Sec-WebSocket-Protocol: %s\r\n", hs->protocol);

    (void)snprintf((char *)outFrame + strlen(outFrame), 
        outLength - strlen(outFrame),
        "Sec-WebSocket-Accept: %s\r\n\r\n", responseKey);

    free(responseKey);

    // if assert fail, that means, that we corrupt memory
    *outLength = strlen(outFrame);
}

void wsMakeFrame(const char *data, size_t dataLength,
                 uint8_t *outFrame, size_t *outLength, enum wsFrameType frameType)
{
  if(frameType != WS_CLOSING_FRAME) 
    assert(outFrame && *outLength);
  assert(frameType < 0x10);

  if (dataLength > 0)
    assert(data);
	
  outFrame[0] = 0x80 | frameType;
    
  if (dataLength <= 125) {
    outFrame[1] = dataLength;
    *outLength = 2;
  } else if (dataLength <= 0xFFFF) {
    outFrame[1] = 126;
    uint16_t payloadLength16b = htons(dataLength);
    memcpy(&outFrame[2], &payloadLength16b, 2);
    *outLength = 4;
  } else {
    assert(dataLength <= 0xFFFF);
    
        /* implementation for 64bit systems
        outFrame[1] = 127;
        dataLength = htonll(dataLength);
        memcpy(&outFrame[2], &dataLength, 8);
        *outLength = 10;
        */
  }
  memcpy(&outFrame[*outLength], data, dataLength);
  *outLength+= dataLength;
}

static size_t getPayloadLength(const uint8_t *inputFrame, size_t inputLength,
                               uint8_t *payloadFieldExtraBytes, enum wsFrameType *frameType) 
{
    size_t payloadLength = inputFrame[1] & 0x7F;
    *payloadFieldExtraBytes = 0;
    if ((payloadLength == 0x7E && inputLength < 4) || (payloadLength == 0x7F && inputLength < 10)) {
        *frameType = WS_INCOMPLETE_FRAME;
        return 0;
    }
    if (payloadLength == 0x7F && (inputFrame[3] & 0x80) != 0x0) {
        *frameType = WS_ERROR_FRAME;
        return 0;
    }

    if (payloadLength == 0x7E) {
        uint16_t payloadLength16b = 0;
        *payloadFieldExtraBytes = 2;
        memcpy(&payloadLength16b, &inputFrame[2], *payloadFieldExtraBytes);
        payloadLength = ntohs(payloadLength16b);
    } else if (payloadLength == 0x7F) {
        *frameType = WS_ERROR_FRAME;
        return 0;
        
        /* // implementation for 64bit systems
        uint64_t payloadLength64b = 0;
        *payloadFieldExtraBytes = 8;
        memcpy(&payloadLength64b, &inputFrame[2], *payloadFieldExtraBytes);
        if (payloadLength64b > SIZE_MAX) {
            *frameType = WS_ERROR_FRAME;
            return 0;
        }
        payloadLength = (size_t)ntohll(payloadLength64b);
        */
    }

    return payloadLength;
}

enum wsFrameType wsParseInputFrame(const uint8_t *inputFrame, const size_t inputLength,
                                   uint8_t **dataPtr, size_t *dataLength)
{
    uint32_t calc_length = 0;
    assert(inputFrame);
    uint8_t * end = inputFrame;
    while(*end != '\0') end++;

    if(end < inputFrame) {
        printf("end < inputFrame\n");
        return WS_ERROR_FRAME;
    }
    calc_length = (uint32_t)(end - inputFrame);

    //    if (inputLength < 2)
    //    return WS_INCOMPLETE_FRAME;
	
    if ((inputFrame[0] & 0x70) != 0x0) {// checks extensions off
        printf("extensions off\n");
        return WS_ERROR_FRAME;
    }
    if ((inputFrame[0] & 0x80) != 0x80) {// we haven't continuation frames support so, fin flag must be set
        printf("we haven't continuation frames support\n");
        return WS_ERROR_FRAME;
    }
    if ((inputFrame[1] & 0x80) != 0x80) { // checks masking bit
        printf("masking bit set\n");
        return WS_ERROR_FRAME;
    }

    uint8_t opcode = inputFrame[0] & 0x0F;
    if (opcode == WS_TEXT_FRAME ||
            opcode == WS_BINARY_FRAME ||
            opcode == WS_CLOSING_FRAME ||
            opcode == WS_PING_FRAME ||
            opcode == WS_PONG_FRAME
    ){
        enum wsFrameType frameType = opcode;

        uint8_t payloadFieldExtraBytes = 0;
        size_t payloadLength = getPayloadLength(inputFrame, calc_length - 1,
                                                &payloadFieldExtraBytes, &frameType);
        printf("received payload length of %lu and %u extra bytes and %u length (%d)\n",
               payloadLength, payloadFieldExtraBytes, calc_length, frameType);
        if (payloadLength > 0) {
            if (payloadLength + 6 + payloadFieldExtraBytes > calc_length - 1) {// 4-maskingKey, 2-header 
                printf("received incomplete frame of %lu > %u\n", 
                       payloadLength + 6 + payloadFieldExtraBytes, calc_length - 1);
                return WS_INCOMPLETE_FRAME;
            }
            uint8_t *maskingKey = &inputFrame[2 + payloadFieldExtraBytes];

            assert(payloadLength == calc_length - 1 - 6 - payloadFieldExtraBytes);

            *dataPtr = &inputFrame[2 + payloadFieldExtraBytes + 4];
            *dataLength = payloadLength;
		
            size_t i;
            for (i = 0; i < *dataLength; i++) {
                (*dataPtr)[i] = (*dataPtr)[i] ^ maskingKey[i%4];
            }
        }
        return frameType;
    }

    return WS_ERROR_FRAME;
}
