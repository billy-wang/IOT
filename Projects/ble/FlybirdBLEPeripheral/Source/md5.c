/*
 * modified  by anqiren  2014/12/10  V1.0bat
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

//#include <stdlib.h>
//#include <stdio.h>
//#include <math.h>
#include "md5.h"

const uint32 X[4][2] = {{0, 1}, {1, 5}, {5, 3}, {0, 7}};
const uint32 S[4][4] = {{ 7, 12, 17, 22 },{ 5, 9 , 14, 20 },{ 4, 11, 16, 23 },{ 6, 10, 15, 21 }};

uint32 F( uint32 X, uint32 Y, uint32 Z )
{
	return ( X & Y ) | ( ~X & Z );
}

uint32 G( uint32 X, uint32 Y, uint32 Z )
 {
	return ( X & Z ) | ( Y & ~Z );
 }

uint32 H( uint32 X, uint32 Y, uint32 Z )
{
	return X ^ Y ^ Z;
}

uint32 I( uint32 X, uint32 Y, uint32 Z )
{
	return Y ^ ( X | ~Z );
}

// rotates x left s bits.
uint32 rotate_left( uint32 x, uint32 s )
{
	return ( x << s ) | ( x >> ( 32 - s ) );
}

// Pre-processin
uint32 count_padding_bits ( uint32 length )
{
//	uint32 div = length * BITS / BLOCK_SIZE;
	uint32 mod = length * BITS % BLOCK_SIZE;
	uint32 c_bits;
	if ( mod == 0 )
	{
			c_bits = MOD_SIZE;
	}
	else
	{
		c_bits = ( MOD_SIZE + BLOCK_SIZE - mod ) % BLOCK_SIZE;
	}
	return c_bits / BITS;
}

MD5String append_padding_bits ( char * argv )
{
	//uint32 msg_length = strlen( argv );
	uint32 msg_length = osal_strlen( argv );
	uint32 bit_length = count_padding_bits( msg_length );
	uint64 app_length = msg_length * BITS;
	MD5String string;
	
	//string.str = (char *)malloc(msg_length + bit_length + APP_SIZE / BITS);
	string.str = (char *)osal_mem_alloc(msg_length + bit_length + APP_SIZE / BITS);
	strncpy( string.str, argv, msg_length );
	memset ( string.str + msg_length, 0, bit_length );
	string.str [ msg_length ] = SINGLE_ONE_BIT;
	memmove ( string.str + msg_length + bit_length, (char *)&app_length, sizeof( uint64 ) );
	string.len = msg_length + bit_length + sizeof( uint64 );
	return string;
}

int32 md5 (char *argv, uint8 *md5_32)
{
	MD5String string;
	uint32 w[16];
	uint32 chain[4];
	uint32 state[4];
//	uint8_t r[16];
	uint32 ( *auxi[ 4 ])( uint32, uint32, uint32 ) = { F, G, H, I };
	int sIdx;
	int wIdx;
	
	string = append_padding_bits ( argv );
	chain[0] = A;
	chain[1] = B;
	chain[2] = C;
	chain[3] = D;
	
	for (uint32 j = 0; j < string.len; j += BLOCK_SIZE / BITS)
	{
		memmove ( (char *)w, string.str + j, BLOCK_SIZE / BITS );
		memmove ( state, chain, sizeof(chain) );
		
		for ( uint8 roundIdx = 0; roundIdx < 4; roundIdx++ )
		{
			wIdx = X[ roundIdx ][ 0 ];
			sIdx = 0;
			for (uint8 i = 0; i < 16; i++ )
			{	
				state[sIdx] = state [(sIdx + 1)%4] + rotate_left( state[sIdx] +(*auxi[ roundIdx])( state[(sIdx+1) % 4],
				state[(sIdx+2) % 4], 
				state[(sIdx+3) % 4]) + w[ wIdx ] + (uint32)floor((1ULL << 32) * fabs(sin( roundIdx * 16 + i + 1 )) ),
				S[ roundIdx ][ i % 4 ]);
				sIdx = ( sIdx + 3 ) % 4;
				wIdx = ( wIdx + X[ roundIdx ][ 1 ] ) & 0xF;
			}
		}
		chain[ 0 ] += state[ 0 ];
		chain[ 1 ] += state[ 1 ];
		chain[ 2 ] += state[ 2 ];
		chain[ 3 ] += state[ 3 ];
	}
	memmove ( md5_32 + 0, (char *)&chain[0], sizeof(uint32) );
	memmove ( md5_32 + 4, (char *)&chain[1], sizeof(uint32) );
	memmove ( md5_32 + 8, (char *)&chain[2], sizeof(uint32) );
	memmove ( md5_32 + 12, (char *)&chain[3], sizeof(uint32) );
	//free(string.str);
	osal_mem_free(string.str);
	string.str = NULL;
	
  return SUCCESS;
}

