#include "QEC_1X_SPI.h"

QEC_1X *QEC_1X::me = NULL;

QEC_1X::QEC_1X(PinName CS)
{
    me=this;
    _cs = new DigitalOut(CS);
}

QEC_1X::~QEC_1X()
{
    delete(_cs);
    delete(me);
}

void QEC_1X::QEC_init(int id_, float scale_, int sign_, SPI *spi) {
  _id = id_; _encoderScale = scale_ ; _sign = sign_;
  _encoderCount=0; _encoderOffset=0;
  outDimension=0.0f;
  *_cs = 1 ;
  QEC_config(spi);
}

void QEC_1X::QEC_read(SPI *spi){
  long buff[4];
  *_cs=0;
  spi->write(0x60); // Request count , Read Counter
  buff[0] = spi->write(0x00); // most significant byte - 4 byte mode (32 bits)
  buff[1] = spi->write(0x00);
  buff[2] = spi->write(0x00);
  buff[3] = spi->write(0x00); // least significant byte
  *_cs=1;
  _encoderCount = (long)(buff[0]<<24) + (long)(buff[1]<<16) + (long)(buff[2]<<8) + (long)buff[3] - _encoderOffset;
}

float QEC_1X::QEC_getPosition(SPI *spi)

{
  QEC_read(spi);
  outDimension = _sign * (_encoderCount-_encoderOffset) * _encoderScale;
  return outDimension;
}

void QEC_1X::QEC_config(SPI *spi)
{
  //spi.begin();
  ThisThread::sleep_for(10);
  *_cs=0;
  spi->write(0x88); //! WRITE_MDR0 
  spi->write(0x03); //! X4 quadrature mode
  *_cs=1;
  ThisThread::sleep_for(10);
  *_cs=0;
  spi->write(0x20); //! CLR_COUNTER
  *_cs=1;
}

void QEC_1X::QEC_clear(SPI *spi) //To restart the counter.
//void QEC_1X::QEC_offset()

{
 //_encoderOffset=_encoderCount; 
 *_cs=0;
  spi->write(0x20); /// Clear Counter
 *_cs=1;
}
