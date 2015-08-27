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

static char rn[] PROGMEM = "\r\n";

void nullHandshake(struct handshake *hs)
{
    hs->host = NULL;
    hs->origin = NULL;
    hs->resource = NULL;
    hs->key = NULL;
    hs->protocol = NULL;
    hs->frameType = WS_EMPTY_FRAME;
}

void freeHandshake(struct handshake *hs)
{
    if (hs->host) {
        free(hs->host);
    }
    if (hs->origin) {
        free(hs->origin);
    }
    if (hs->resource) {
        free(hs->resource);
    }
    if (hs->key) {
        free(hs->key);
    }
    if (hs->protocol) {
        free(hs->protocol);
    }
    nullHandshake(hs);
}

static char* getUptoLinefeed(const char *startFrom)
{
    char *writeTo = NULL;
    uint8_t newLength = strstr_P(startFrom, rn) - startFrom;
    assert(newLength);
    writeTo = (char *)malloc(newLength+1); //+1 for '\x00'
    assert(writeTo);
    memcpy(writeTo, startFrom, newLength);
    writeTo[ newLength ] = 0;

    return writeTo;
}

enum wsFrameType wsParseHandshake(const uint8_t *inputFrame, size_t inputLength,
                                  struct handshake *hs)
{
    const char *inputPtr = (const char *)inputFrame;
    char *endPtr = (const char *)inputFrame;
    while(*endPtr != '\0') {
      endPtr++;
    }

    if (!strstr((const char *)inputFrame, "\r\n\r\n"))
        return WS_INCOMPLETE_FRAME;
	
    if (memcmp_P(inputFrame, PSTR("GET "), 4) != 0)
        return WS_ERROR_FRAME;
    // measure resource size
    char *first = strchr((const char *)inputFrame, ' ');
    if (!first)
        return WS_ERROR_FRAME;
    first++;
    char *second = strchr(first, ' ');
    if (!second)
        return WS_ERROR_FRAME;

    if (hs->resource) {
        free(hs->resource);
        hs->resource = NULL;
    }
    hs->resource = (char *)malloc(second - first + 1); // +1 is for \x00 symbol
    assert(hs->resource);

    if (sscanf_P(inputPtr, PSTR("GET %s HTTP/1.1\r\n"), hs->resource) != 1)
        return WS_ERROR_FRAME;
    inputPtr = strstr_P(inputPtr, rn) + 2;

    /*
        parse next lines
     */
    #define prepare(x) do {if (x) { free(x); x = NULL; }} while(0)
    #define strtolower(x) do { int i; for (i = 0; x[i]; i++) x[i] = tolower(x[i]); } while(0)
    uint8_t connectionFlag = FALSE;
    uint8_t upgradeFlag = FALSE;
    uint8_t subprotocolFlag = FALSE;
    uint8_t versionMismatch = FALSE;
    while (inputPtr < endPtr && inputPtr[0] != '\r' && inputPtr[1] != '\n') {
        if (memcmp_P(inputPtr, hostField, strlen_P(hostField)) == 0) {
            inputPtr += strlen_P(hostField);
            prepare(hs->host);
            hs->host = getUptoLinefeed(inputPtr);
        } else
        if (memcmp_P(inputPtr, originField, strlen_P(originField)) == 0) {
            inputPtr += strlen_P(originField);
            prepare(hs->origin);
            hs->origin = getUptoLinefeed(inputPtr);
        } else
        if (memcmp_P(inputPtr, protocolField, strlen_P(protocolField)) == 0) {
            inputPtr += strlen_P(protocolField);
            subprotocolFlag = TRUE;
            hs->protocol = getUptoLinefeed(inputPtr);
        } else
        if (memcmp_P(inputPtr, keyField, strlen_P(keyField)) == 0) {
            inputPtr += strlen_P(keyField);
            prepare(hs->key);
            hs->key = getUptoLinefeed(inputPtr);
        } else
        if (memcmp_P(inputPtr, versionField, strlen_P(versionField)) == 0) {
            inputPtr += strlen_P(versionField);
            char *versionString = NULL;
            versionString = getUptoLinefeed(inputPtr);
            if (memcmp_P(versionString, version, strlen_P(version)) != 0)
                versionMismatch = TRUE;
            free(versionString);
        } else
        if (memcmp_P(inputPtr, connectionField, strlen_P(connectionField)) == 0) {
            inputPtr += strlen_P(connectionField);
            char *connectionValue = NULL;
            connectionValue = getUptoLinefeed(inputPtr);
            strtolower(connectionValue);
            assert(connectionValue);
            if (strstr_P(connectionValue, upgrade) != NULL)
                connectionFlag = TRUE;
            free(connectionValue);
        } else
        if (memcmp_P(inputPtr, upgradeField, strlen_P(upgradeField)) == 0) {
            inputPtr += strlen_P(upgradeField);
            char *compare = NULL;
            compare = getUptoLinefeed(inputPtr);
            strtolower(compare);
            assert(compare);
            if (memcmp_P(compare, websocket, strlen_P(websocket)) == 0)
                upgradeFlag = TRUE;
            free(compare);
        };

        inputPtr = strstr_P(inputPtr, rn) + 2;
    }

    // we have read all data, so check them
    if(!hs->host)
        printf("no host!\n");
    if(!hs->key)
        printf("no key!\n");
    else
      printf("key: %s\n", hs->key);
    if(!connectionFlag)
        printf("no connectionFlag!\n");
    if(!upgradeFlag)
        printf("no upgrade flag!\n");
    if(subprotocolFlag)
        printf("subprotocol!\n");
    if(versionMismatch)
        printf("version mismatch!\n");

    // we have read all data, so check them
    if (!hs->host || !hs->key || !connectionFlag || !upgradeFlag || versionMismatch)
    {
        hs->frameType = WS_ERROR_FRAME;
    } else {
        hs->frameType = WS_OPENING_FRAME;
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
    uint8_t length = strlen(hs->key)+strlen_P(secret);
    responseKey = malloc(length);
    memcpy(responseKey, hs->key, strlen(hs->key));
    memcpy_P(&(responseKey[strlen(hs->key)]), secret, strlen_P(secret));
    unsigned char shaHash[20];
    memset(shaHash, 0, sizeof(shaHash));
    sha1(shaHash, responseKey, length);

    size_t base64Length = b64_encode_string(shaHash, 20, responseKey, length);

    // size_t base64Length = base64(responseKey, length, shaHash, 20);
    responseKey[base64Length] = '\0';

    printf("handshake answer out key: %s (target length = %d, len = %d)\n", 
	   responseKey, length, base64Length);

    int written = sprintf_P((char *)outFrame,
                            PSTR("HTTP/1.1 101 Switching Protocols\r\n"
                                 "%s%s\r\n"
                                 "%s%s\r\n"
                                 "Sec-WebSocket-Protocol: %s\r\n"
                                 "Sec-WebSocket-Accept: %s\r\n\r\n"),
                            upgradeField, websocket,
                            connectionField, upgrade2,
			    hs->protocol,
                            responseKey);
	
    free(responseKey);
    // if assert fail, that means, that we corrupt memory
    assert(written <= *outLength);
    *outLength = written;
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

enum wsFrameType wsParseInputFrame(uint8_t *inputFrame, size_t inputLength,
                                   uint8_t **dataPtr, size_t *dataLength)
{
    assert(inputFrame);
    uint8_t * end = inputFrame;
    while(*end != '\0') end++; 

    //    if (inputLength < 2)
    //    return WS_INCOMPLETE_FRAME;
	
    if ((inputFrame[0] & 0x70) != 0x0) // checks extensions off
        return WS_ERROR_FRAME;
    if ((inputFrame[0] & 0x80) != 0x80) // we haven't continuation frames support
        return WS_ERROR_FRAME; // so, fin flag must be set
    if ((inputFrame[1] & 0x80) != 0x80) // checks masking bit
        return WS_ERROR_FRAME;

    uint8_t opcode = inputFrame[0] & 0x0F;
    if (opcode == WS_TEXT_FRAME ||
            opcode == WS_BINARY_FRAME ||
            opcode == WS_CLOSING_FRAME ||
            opcode == WS_PING_FRAME ||
            opcode == WS_PONG_FRAME
    ){
        enum wsFrameType frameType = opcode;

        uint8_t payloadFieldExtraBytes = 0;
        size_t payloadLength = getPayloadLength(inputFrame, end - inputFrame - 1,
                                                &payloadFieldExtraBytes, &frameType);
        if (payloadLength > 0) {
            if (payloadLength + 6 + payloadFieldExtraBytes > end - inputFrame - 1) // 4-maskingKey, 2-header
                return WS_INCOMPLETE_FRAME;
            uint8_t *maskingKey = &inputFrame[2 + payloadFieldExtraBytes];

            assert(payloadLength == end - inputFrame - 1 - 6 - payloadFieldExtraBytes);

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
