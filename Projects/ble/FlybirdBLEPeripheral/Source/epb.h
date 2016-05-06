//  epb.h
//  MicroMessenger
//
//  Created by harlliu@tencent.com on 14-02-15.
//  Copyright 2014 Tencent. All rights reserved.
//

//  Version : 1.0.2

#ifndef __EPB_H__
#define __EPB_H__

//#include <stdint.h>
//#include <stdbool.h>
#include "hal_types.h"

typedef struct
{
    uint8 *data;
    int len;
} Bytes;

typedef struct
{
    const uint8 *data;
    int len;
} CBytes;

typedef struct
{
    char *str;
    int len;
} String;

typedef struct
{
    const char *str;
    int len;
} CString;

typedef uint8 Message;

typedef struct 
{
    const uint8 *unpack_buf;
    uint8 *pack_buf;
    int buf_len;
    int buf_offset;
} Epb;

/*
 * embeded protobuf unpack functions
 */

void epb_unpack_init(Epb *e, const uint8 *buf, int len);
bool epb_has_tag(Epb *e, uint16 tag);

//Varint
int32 epb_get_int32(Epb *e, uint16 tag);
uint32 epb_get_uint32(Epb *e, uint16 tag);
int32 epb_get_sint32(Epb *e, uint16 tag);
bool epb_get_bool(Epb *e, uint16 tag);
int epb_get_enum(Epb *e, uint16 tag);

//Length Delimited
const char *epb_get_string(Epb *e, uint16 tag, int *len);
const uint8 *epb_get_bytes(Epb *e, uint16 tag, int *len);
const Message *epb_get_message(Epb *e, uint16 tag, int *len);

//Length Delimited Packed Repeadted Field
//TODO

//Fixed32
uint32 epb_get_fixed32(Epb *e, uint16 tag);
int32 epb_get_sfixed32(Epb *e, uint16 tag);
float epb_get_float(Epb *e, uint16 tag);

/*
 * embeded protobuf pack functions
 */

void epb_pack_init(Epb *e, uint8 *buf, int len);
int epb_get_packed_size(Epb *e);

//Varint
int epb_set_int32(Epb *e, uint16 tag, int32 value);
int epb_set_uint32(Epb *e, uint16 tag, uint32 value);
int epb_set_sint32(Epb *e, uint16 tag, int32 value);
int epb_set_bool(Epb *e, uint16 tag, bool value);
int epb_set_enum(Epb *e, uint16 tag, int value);

//Length Delimited
int epb_set_string(Epb *e, uint16 tag, const char *data, int len);
int epb_set_bytes(Epb *e, uint16 tag, const uint8 *data, int len);
int epb_set_message(Epb *e, uint16 tag, const Message *data, int len);

//Length Delimited Packed Repeadted Field
//TODO

//Fixed32
int epb_set_fixed32(Epb *e, uint16 tag, uint32 value);
int epb_set_sfixed32(Epb *e, uint16 tag, int32 value);
int epb_set_float(Epb *e, uint16 tag, float value);

//Pack size
int epb_varint32_pack_size(uint16 tag, uint32 value, bool is_signed);
int epb_fixed32_pack_size(uint16 tag);
int epb_length_delimited_pack_size(uint16 tag, int len);

#endif
