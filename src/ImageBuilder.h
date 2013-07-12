#ifndef OPTRISIMAGEBUILDER_H
#define OPTRISIMAGEBUILDER_H

namespace optris
{

enum EOptrisColoringPalette{
    eIron,
    eBlackWhite
};

/**
 * Image creation module for displaying purposes
 * @author Stefan May
 */
class ImageBuilder
{
public:
  /**
   * Standard constructor
   */
	ImageBuilder();

  /**
   * Standard destructor
   */
  ~ImageBuilder();

  /**
   * Set fixed temperature range
   * @param[in] min Lower limit in °C
   * @param[in] max Upper limit in °C
   */
  void setTemperatureRange(float min, float max);

  /**
   * Activate/Deactivate dynamic scaling of temperature range
   * @param[in] dynamicScale Dynamic scaling flag
   */
  void setDynamicScaling(bool dynamicScale);

  /**
   * Set the size of Imager-Matrix
   * @param[in] src Source image (short-Format)
   * @param[in] size Size of src image, buffer of dst image must be 3*size
   * @param[out] dst Destination image
   */
  void setSize(unsigned int width, unsigned int height);
    unsigned int width(void) const { return _width; }
    unsigned int height(void) const { return _height; }
  unsigned int getStride(void);

  /**
   * Image conversion to rgb
   * @param[in] src Source image (short-Format)
   * @param[in] size Size of src image, buffer of dst image must be 3*size
   * @param[out] dst Destination image
   * @param[in] palette coloring palette
   */
  void convertTemperatureToPalette(unsigned short* src, unsigned char* dst, EOptrisColoringPalette palette);

  /**
   * Draw crosshair to the center of image
   * @param[in/out] rgb Image in RGB-format
   * @param[in] x x-position in image
   * @param[out] y y-position in image
   */
  void drawCrosshair(unsigned char* rgb, unsigned int x, unsigned int y);

private:

  /**
   * Flag indicating a dynamic temperature range scaling for display
   */
  bool _dynamicScale;

  /**
   * Minimum temperature of display range
   */
  unsigned short _min;

  /**
   * Maximum temperature of display range
   */
  unsigned short _max;

  /**
   * Real width of Imager-Matrix
   */
  unsigned int _width;
  /**
   * Real height of Imager-Matrix
   */
  unsigned int _height;
  /**
   * Width including additional pixels in every row of Imager-Matrix to fit the modulo-4-criteria
   */
  unsigned int  _stride;
  /**
   * Real size if Imager-Matrix
   */
  unsigned int _size;

};

}

#endif //OPTRISIMAGEBUILDER_H
