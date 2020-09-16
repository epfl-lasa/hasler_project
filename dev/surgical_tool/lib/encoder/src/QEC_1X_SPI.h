#ifndef __QEC_1X_SPI_H__
#define __QEC_1X_SPI_H__

#include <SPI.h>
#include <mbed.h>
#include <rtos.h>


class QEC_1X
{
	public:
		QEC_1X(PinName CS);
		~QEC_1X(); 
		 
		void QEC_init(int id_, float scale_, int sign_, SPI* spi);
		float outDimension;  
		void QEC_read(SPI* spi);
		float QEC_getPosition(SPI* spi);
		void QEC_config(SPI* spi);
		void QEC_clear(SPI* spi);
		//void QEC_offset();
		
	private:
		long _encoderCount;
		DigitalOut* _cs;
		int _id;
		float _encoderScale;
		int _sign;
		long _encoderOffset;
		static QEC_1X *me;
};

#endif /*__QEC_1X_SPI_H__*/