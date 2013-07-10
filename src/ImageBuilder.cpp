#include "ImageBuilder.h"
#include "limits.h"
#include "IronPalette.h"
#include "BlackWhitePalette.h"

namespace optris
{

/**
 * Helper method to change the scale for displaying purposes
 * @param[in] val Temperature value
 * @param[in] min Minimum value of temperature
 * @param[in] max Maximum value of temperature
 * @param[in] scal_min Lower limit for scaled value
 * @param[in] scal_max Upper limit for scaled value
 */
unsigned char changeScale(float val, float min, float max, float scal_min, float scal_max)
{
  unsigned char ret = 0;
  if (val > max)
  {
    return scal_max;
  }
  else if (val < min)
  {
    return scal_min;
  }
  else
  {
    ret = (scal_max - scal_min) * ((val - min) / (max - min)) + scal_min;
    return ret;
  }
}

ImageBuilder::ImageBuilder()
{
	_dynamicScale = false;
	_min = 800;
	_max = 1200;
}

ImageBuilder::~ImageBuilder()
{

}

void ImageBuilder::setTemperatureRange(float min, float max)
{
	_min = (unsigned short)(min*10.0f) + 1000;
	_max = (unsigned short)(max*10.0f) + 1000;
}

void ImageBuilder::setDynamicScaling(bool dynamicScale)
{
    _dynamicScale = dynamicScale;
}

void ImageBuilder::setSize(unsigned int width, unsigned int height)
{
	_width = width;
	_height = height;
	_size = width * height;
	_stride = width;
	while(_stride % 4)
	  _stride++;
}

unsigned int ImageBuilder::getStride(void)
{
	return _stride;
}

void ImageBuilder::convertTemperatureToPalette(unsigned short* src, unsigned char* dst, EOptrisColoringPalette palette)
{
    unsigned int i=0;
    unsigned int j=0;
    int dIdx=0;
    int sIdx=0;

    if(_dynamicScale==true)
    {
        _min = USHRT_MAX;
        _max = 0;
        for(i = 0; i < _size; i++)
        {
            unsigned short pixel = src[i];
            if(pixel > _max) _max = pixel;
            if(pixel < _min) _min = pixel;
        }
    }

    unsigned int offs = (_stride - _width) * 3;
    for(j = 0; j < _height; j++)
    {
        for(i = 0; i < _width; i++)
        {
            unsigned int idx = changeScale(src[sIdx++], _min, _max, 0, 239);

            switch (palette)
            {
            default:
            case eIron:
                dst[dIdx++] = Iron[idx][0];
                dst[dIdx++] = Iron[idx][1];
                dst[dIdx++] = Iron[idx][2];
                break;

            case eBlackWhite:
                dst[dIdx++] = BlackWhite[idx][0];
                dst[dIdx++] = BlackWhite[idx][1];
                dst[dIdx++] = BlackWhite[idx][2];
                break;
            };

        }
        dIdx += offs;
    }
}

void ImageBuilder::drawCrosshair(unsigned char* rgb, unsigned int x, unsigned int y)
{
	int idxPoint = (x + _stride*y);
	int radius = 3;
	for(int i=-radius; i<=radius; i++)
	{
		int idx = (idxPoint + i) * 3;
		rgb[idx]   = 0;
		rgb[idx+1] = 255;
		rgb[idx+2] = 0;
		idx = (idxPoint + i*_stride) * 3;
		rgb[idx]   = 0;
		rgb[idx+1] = 255;
		rgb[idx+2] = 0;
	}
}

}
