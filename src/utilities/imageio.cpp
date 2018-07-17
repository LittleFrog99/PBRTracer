#include "imageio.h"
#include "file.h"
#include "log.h"
#include "ext/lodepng.h"
#include "ext/targa.h"
#include <OpenEXR/ImfRgba.h>
#include <OpenEXR/ImfRgbaFile.h>

namespace ImageIO {

using namespace File;

static void writeImageEXR(const string &name, const Float *pixels, int xRes, int yRes,
                          int totalXRes, int totalYRes, int xOffset, int yOffset);
static void writeImageTGA(const string &name, const uint8_t *pixels, int xRes, int yRes,
                          int totalXRes, int totalYRes, int xOffset, int yOffset);
static RGBSpectrum *readImageTGA(const string &name, int *w, int *h);
static RGBSpectrum *readImagePNG(const string &name, int *w, int *h);
static bool writeImagePFM(const string &filename, const Float *rgb, int xres, int yres);
static RGBSpectrum *readImagePFM(const string &filename, int *xres, int *yres);

inline Float gammaCorrect(Float value) {
    if (value <= 0.0031308f) return 12.92f * value;
    return 1.055f * std::pow(value, (Float)(1.f / 2.4f)) - 0.055f;
}

inline Float inverseGammaCorrect(Float value) {
    if (value <= 0.04045f) return value * 1.f / 12.92f;
    return std::pow((value + 0.055f) * 1.f / 1.055f, (Float)2.4f);
}

// ImageIO Function Definitions
unique_ptr<RGBSpectrum[]> readImage(const string &name,
                                         Point2i *resolution) {
    if (hasExtension(name, ".exr"))
        return unique_ptr<RGBSpectrum[]>(
            readImageEXR(name, &resolution->x, &resolution->y));
    else if (hasExtension(name, ".tga"))
        return unique_ptr<RGBSpectrum[]>(
            readImageTGA(name, &resolution->x, &resolution->y));
    else if (hasExtension(name, ".png"))
        return unique_ptr<RGBSpectrum[]>(
            readImagePNG(name, &resolution->x, &resolution->y));
    else if (hasExtension(name, ".pfm"))
        return unique_ptr<RGBSpectrum[]>(
            readImagePFM(name, &resolution->x, &resolution->y));
    ERROR("Unable to load image stored in format \"%s\" for filename \"%s\".",
          strrchr(name.c_str(), '.') ? (strrchr(name.c_str(), '.') + 1)
                                     : "(unknown)",
          name.c_str());
    return nullptr;
}

void writeImage(const string &name, const Float *rgb,
                const Bounds2i &outputBounds, const Point2i &totalResolution) {
    Vector2i resolution = outputBounds.diagonal();
    if (hasExtension(name, ".exr")) {
        writeImageEXR(name, rgb, resolution.x, resolution.y, totalResolution.x,
                      totalResolution.y, outputBounds.pMin.x,
                      outputBounds.pMin.y);
    } else if (hasExtension(name, ".pfm")) {
        writeImagePFM(name, rgb, resolution.x, resolution.y);
    } else if (hasExtension(name, ".tga") || hasExtension(name, ".png")) {
        // 8-bit formats; apply gamma
        Vector2i resolution = outputBounds.diagonal();
        unique_ptr<uint8_t[]> rgb8(new uint8_t[3 * resolution.x * resolution.y]);
        uint8_t *dst = rgb8.get();
        for (int y = 0; y < resolution.y; ++y) {
            for (int x = 0; x < resolution.x; ++x) {
#define TO_BYTE(v) (uint8_t) clamp(255.f * gammaCorrect(v) + 0.5f, 0.f, 255.f)
                dst[0] = TO_BYTE(rgb[3 * (y * resolution.x + x) + 0]);
                dst[1] = TO_BYTE(rgb[3 * (y * resolution.x + x) + 1]);
                dst[2] = TO_BYTE(rgb[3 * (y * resolution.x + x) + 2]);
#undef TO_BYTE
                dst += 3;
            }
        }

        if (hasExtension(name, ".tga"))
            writeImageTGA(name, rgb8.get(), resolution.x, resolution.y,
                          totalResolution.x, totalResolution.y,
                          outputBounds.pMin.x, outputBounds.pMin.y);
        else {
            unsigned int error = lodepng_encode24_file(
                name.c_str(), rgb8.get(), resolution.x, resolution.y);
            if (error != 0)
                ERROR("Error writing PNG \"%s\": %s", name.c_str(), lodepng_error_text(error));
        }
    } else {
        ERROR("Can't determine image file type from suffix of filename \"%s\"",
              name.c_str());
    }
}

RGBSpectrum *readImageEXR(const string &name, int *width, int *height,
                          Bounds2i *dataWindow, Bounds2i *displayWindow) {
    using namespace Imf;
    using namespace Imath;
    try {
        RgbaInputFile file(name.c_str());
        Box2i dw = file.dataWindow();

        // OpenEXR uses inclusive pixel bounds; adjust to non-inclusive
        // (the convention pbrt uses) in the values returned.
        if (dataWindow)
            *dataWindow = {{dw.min.x, dw.min.y}, {dw.max.x + 1, dw.max.y + 1}};
        if (displayWindow) {
            Box2i dispw = file.displayWindow();
            *displayWindow = {{dispw.min.x, dispw.min.y},
                              {dispw.max.x + 1, dispw.max.y + 1}};
        }
        *width = dw.max.x - dw.min.x + 1;
        *height = dw.max.y - dw.min.y + 1;

        vector<Rgba> pixels(*width * *height);
        file.setFrameBuffer(&pixels[0] - dw.min.x - dw.min.y * *width, 1,
                            *width);
        file.readPixels(dw.min.y, dw.max.y);

        RGBSpectrum *ret = new RGBSpectrum[*width * *height];
        for (int i = 0; i < *width * *height; ++i) {
            Float frgb[3] = {pixels[i].r, pixels[i].g, pixels[i].b};
            ret[i] = RGBSpectrum::fromRGB(frgb);
        }
        LOG(INFO) << STRING_PRINTF("Read EXR image %s (%d x %d)", name.c_str(), *width, *height);
        return ret;
    } catch (const exception &e) {
        ERROR("Unable to read image file \"%s\": %s", name.c_str(), e.what());
    }

    return nullptr;
}

static void writeImageEXR(const string &name, const Float *pixels,
                          int xRes, int yRes, int totalXRes, int totalYRes,
                          int xOffset, int yOffset) {
    using namespace Imf;
    using namespace Imath;

    Rgba *hrgba = new Rgba[xRes * yRes];
    for (int i = 0; i < xRes * yRes; ++i)
        hrgba[i] = Rgba(pixels[3 * i], pixels[3 * i + 1], pixels[3 * i + 2]);

    // OpenEXR uses inclusive pixel bounds.
    Box2i displayWindow(V2i(0, 0), V2i(totalXRes - 1, totalYRes - 1));
    Box2i dataWindow(V2i(xOffset, yOffset),
                     V2i(xOffset + xRes - 1, yOffset + yRes - 1));

    try {
        RgbaOutputFile file(name.c_str(), displayWindow, dataWindow,
                            WRITE_RGBA);
        file.setFrameBuffer(hrgba - xOffset - yOffset * xRes, 1, xRes);
        file.writePixels(yRes);
    } catch (const exception &exc) {
        ERROR("Error writing \"%s\": %s", name.c_str(), exc.what());
    }

    delete[] hrgba;
}

// TGA Function Definitions
void writeImageTGA(const string &name, const uint8_t *pixels, int xRes,
                   int yRes, int totalXRes, int totalYRes, int xOffset,
                   int yOffset) {
    // Reformat to BGR layout.
    unique_ptr<uint8_t[]> outBuf(new uint8_t[3 * xRes * yRes]);
    uint8_t *dst = outBuf.get();
    const uint8_t *src = pixels;
    for (int y = 0; y < yRes; ++y) {
        for (int x = 0; x < xRes; ++x) {
            dst[0] = src[2];
            dst[1] = src[1];
            dst[2] = src[0];
            dst += 3;
            src += 3;
        }
    }

    tga_result result;
    if ((result = tga_write_bgr(name.c_str(), outBuf.get(), xRes, yRes, 24)) !=
        TGA_NOERR)
        ERROR("Unable to write output file \"%s\" (%s)",
              name.c_str(), tga_error(result));
}

static RGBSpectrum *readImageTGA(const string &name, int *width,
                                 int *height) {
    tga_image img;
    tga_result result;
    if ((result = tga_read(&img, name.c_str())) != TGA_NOERR) {
        ERROR("Unable to read from TGA file \"%s\" (%s)",
              name.c_str(), tga_error(result));
        return nullptr;
    }

    if (tga_is_right_to_left(&img)) tga_flip_horiz(&img);
    if (!tga_is_top_to_bottom(&img)) tga_flip_vert(&img);
    if (tga_is_colormapped(&img)) tga_color_unmap(&img);

    *width = img.width;
    *height = img.height;

    // "Unpack" the pixels (origin in the lower left corner).
    // TGA pixels are in BGRA format.
    RGBSpectrum *ret = new RGBSpectrum[*width * *height];
    RGBSpectrum *dst = ret;
    for (int y = 0; y < *height; y++)
        for (int x = 0; x < *width; x++) {
            uint8_t *src = tga_find_pixel(&img, x, y);
            if (tga_is_mono(&img))
                *dst++ = RGBSpectrum(*src / 255.f);
            else {
                Float c[3];
                c[2] = src[0] / 255.f;
                c[1] = src[1] / 255.f;
                c[0] = src[2] / 255.f;
                *dst++ = RGBSpectrum::fromRGB(c);
            }
        }

    tga_free_buffers(&img);
    LOG(INFO) << STRING_PRINTF("Read TGA image %s (%d x %d)", name.c_str(), *width, *height);

    return ret;
}

static RGBSpectrum *readImagePNG(const string &name, int *width,
                                 int *height) {
    unsigned char *rgb;
    unsigned w, h;
    unsigned int error = lodepng_decode24_file(&rgb, &w, &h, name.c_str());
    if (error != 0) {
        ERROR("Error reading PNG \"%s\": %s", name.c_str(),
              lodepng_error_text(error));
        return nullptr;
    }
    *width = w;
    *height = h;

    RGBSpectrum *ret = new RGBSpectrum[*width * *height];
    unsigned char *src = rgb;
    for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x, src += 3) {
            Float c[3];
            c[0] = src[0] / 255.f;
            c[1] = src[1] / 255.f;
            c[2] = src[2] / 255.f;
            ret[y * *width + x] = RGBSpectrum::fromRGB(c);
        }
    }

    free(rgb);
    LOG(INFO) << STRING_PRINTF("Read PNG image %s (%d x %d)", name.c_str(), *width, *height);
    return ret;
}

// PFM Function Definitions
/*
 * PFM reader/writer code courtesy Jiawen "Kevin" Chen
 * (http://people.csail.mit.edu/jiawen/)
 */

static constexpr bool hostLittleEndian =
#if defined(__BYTE_ORDER__)
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    true
  #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    false
  #else
    #error "__BYTE_ORDER__ defined but has unexpected value"
  #endif
#else
  #if defined(__LITTLE_ENDIAN__) || defined(__i386__) || defined(__x86_64__) || \
      defined(WIN32)
    true
  #elif defined(__BIG_ENDIAN__)
    false
  #elif defined(__sparc) || defined(__sparc__)
    false
  #else
    #error "Can't detect machine endian-ness at compile-time."
  #endif
#endif
    ;

#define BUFFER_SIZE 80

static inline int isWhitespace(char c) {
    return c == ' ' || c == '\n' || c == '\t';
}

// Reads a "word" from the fp and puts it into buffer and adds a null
// terminator.  i.e. it keeps reading until whitespace is reached.  Returns
// the number of characters read *not* including the whitespace, and
// returns -1 on an error.
static int readWord(FILE *fp, char *buffer, int bufferLength) {
    int n;
    int c;

    if (bufferLength < 1) return -1;

    n = 0;
    c = fgetc(fp);
    while (c != EOF && !isWhitespace(c) && n < bufferLength) {
        buffer[n] = c;
        ++n;
        c = fgetc(fp);
    }

    if (n < bufferLength) {
        buffer[n] = '\0';
        return n;
    }

    return -1;
}

static RGBSpectrum *readImagePFM(const string &filename, int *xres, int *yres) {
    float *data = nullptr;
    RGBSpectrum *rgb = nullptr;
    char buffer[BUFFER_SIZE];
    unsigned int nFloats;
    int nChannels, width, height;
    float scale;
    bool fileLittleEndian;

    FILE *fp = fopen(filename.c_str(), "rb");
    if (!fp) goto fail;

    // read either "Pf" or "PF"
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;

    if (strcmp(buffer, "Pf") == 0)
        nChannels = 1;
    else if (strcmp(buffer, "PF") == 0)
        nChannels = 3;
    else
        goto fail;

    // read the rest of the header
    // read width
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    width = atoi(buffer);
    *xres = width;

    // read height
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    height = atoi(buffer);
    *yres = height;

    // read scale
    if (readWord(fp, buffer, BUFFER_SIZE) == -1) goto fail;
    sscanf(buffer, "%f", &scale);

    // read the data
    nFloats = nChannels * width * height;
    data = new float[nFloats];
    // Flip in Y, as P*M has the origin at the lower left.
    for (int y = height - 1; y >= 0; --y) {
        if (fread(&data[y * nChannels * width], sizeof(float),
                  nChannels * width, fp) != nChannels * width)
            goto fail;
    }

    // apply endian conversian and scale if appropriate
    fileLittleEndian = (scale < 0.f);
    if (hostLittleEndian ^ fileLittleEndian) {
        uint8_t bytes[4];
        for (unsigned int i = 0; i < nFloats; ++i) {
            memcpy(bytes, &data[i], 4);
            swap(bytes[0], bytes[3]);
            swap(bytes[1], bytes[2]);
            memcpy(&data[i], bytes, 4);
        }
    }
    if (abs(scale) != 1.f)
        for (unsigned int i = 0; i < nFloats; ++i) data[i] *= abs(scale);

    // create RGBs...
    rgb = new RGBSpectrum[width * height];
    if (nChannels == 1) {
        for (int i = 0; i < width * height; ++i) rgb[i] = RGBSpectrum(data[i]);
    } else {
        for (int i = 0; i < width * height; ++i) {
            Float frgb[3] = {data[3 * i], data[3 * i + 1], data[3 * i + 2]};
            rgb[i] = RGBSpectrum::fromRGB(frgb);
        }
    }

    delete[] data;
    fclose(fp);
    LOG(INFO) << STRING_PRINTF("Read PFM image %s (%d x %d)",
                              filename.c_str(), *xres, *yres);
    return rgb;

fail:
    ERROR("Error reading PFM file \"%s\"", filename.c_str());
    if (fp) fclose(fp);
    delete[] data;
    delete[] rgb;
    return nullptr;
}

static bool writeImagePFM(const string &filename, const Float *rgb,
                          int width, int height) {
    FILE *fp;
    float scale;

    fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        ERROR("Unable to open output PFM file \"%s\"", filename.c_str());
        return false;
    }

    unique_ptr<float[]> scanline(new float[3 * width]);

    // only write 3 channel PFMs here...
    if (fprintf(fp, "PF\n") < 0) goto fail;

    // write the width and height, which must be positive
    if (fprintf(fp, "%d %d\n", width, height) < 0) goto fail;

    // write the scale, which encodes endianness
    scale = hostLittleEndian ? -1.f : 1.f;
    if (fprintf(fp, "%f\n", scale) < 0) goto fail;

    // write the data from bottom left to upper right as specified by
    // http://netpbm.sourceforge.net/doc/pfm.html
    // The raster is a sequence of pixels, packed one after another, with no
    // delimiters of any kind. They are grouped by row, with the pixels in each
    // row ordered left to right and the rows ordered bottom to top.
    for (int y = height - 1; y >= 0; y--) {
        // in case Float is 'double', copy into a staging buffer that's
        // definitely a 32-bit float...
        for (int x = 0; x < 3 * width; ++x)
            scanline[x] = rgb[y * width * 3 + x];
        if (fwrite(&scanline[0], sizeof(float), width * 3, fp) <
            (size_t)(width * 3))
            goto fail;
    }

    fclose(fp);
    return true;

fail:
    ERROR("Error writing PFM file \"%s\"", filename.c_str());
    fclose(fp);
    return false;
}

};
