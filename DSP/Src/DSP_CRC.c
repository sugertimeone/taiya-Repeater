#include "DSP_CRC.h"

unsigned int CRC_OutData(uint8_t *data,char size)
{
	unsigned int C=0;
	
	for(int i=0;i<size;i++)
	{
		C^=data[i];
	}
	return C;
}
#define crc_mul 0x1021  //生成多项式

unsigned int cal_crc(unsigned char *ptr, unsigned char len)
{
    unsigned char i;
    unsigned int crc=0;
    while(len-- != 0)
    {
        for(i=0x80; i!=0; i>>=1)
        {
            if((crc&0x8000)!=0)
            {
               crc<<=1;
               crc^=(crc_mul);
            }else{
               crc<<=1;
            }
            if((*ptr&i)!=0)
            {
               crc ^= (crc_mul);
            }
        }
        ptr ++;
    }
    return (crc);
}
