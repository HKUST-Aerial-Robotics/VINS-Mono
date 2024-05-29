#include "camodocal/gpl/gpl.h"

#include <set>
#ifdef _WIN32
#include <winsock.h>
#else
#include <time.h>
#endif


// source: https://stackoverflow.com/questions/5167269/clock-gettime-alternative-in-mac-os-x
#ifdef __APPLE__
#include <mach/mach_time.h>
#define ORWL_NANO (+1.0E-9)
#define ORWL_GIGA UINT64_C(1000000000)

static double orwl_timebase = 0.0;
static uint64_t orwl_timestart = 0;

struct timespec orwl_gettime(void) {
  // be more careful in a multithreaded environement
  if (!orwl_timestart) {
    mach_timebase_info_data_t tb = { 0 };
    mach_timebase_info(&tb);
    orwl_timebase = tb.numer;
    orwl_timebase /= tb.denom;
    orwl_timestart = mach_absolute_time();
  }
  struct timespec t;
  double diff = (mach_absolute_time() - orwl_timestart) * orwl_timebase;
  t.tv_sec = diff * ORWL_NANO;
  t.tv_nsec = diff - (t.tv_sec * ORWL_GIGA);
  return t;
}
#endif // __APPLE__


const double WGS84_A = 6378137.0;
const double WGS84_ECCSQ = 0.00669437999013;

// Windows lacks fminf
#ifndef fminf
#define fminf(x, y) (((x) < (y)) ? (x) : (y))
#endif

namespace camodocal
{

double hypot3(double x, double y, double z)
{
	return sqrt(square(x) + square(y) + square(z));
}

float hypot3f(float x, float y, float z)
{
	return sqrtf(square(x) + square(y) + square(z));
}

double d2r(double deg)
{
	return deg / 180.0 * M_PI;
}

float d2r(float deg)
{
	return deg / 180.0f * M_PI;
}

double r2d(double rad)
{
	return rad / M_PI * 180.0;
}

float r2d(float rad)
{
	return rad / M_PI * 180.0f;
}

double sinc(double theta)
{
    return sin(theta) / theta;
}

#ifdef _WIN32
#include <sys/timeb.h>
#include <sys/types.h>
#include <winsock.h>
LARGE_INTEGER
getFILETIMEoffset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

int
clock_gettime(int X, struct timespec *tp)
{
    LARGE_INTEGER           t;
    FILETIME            f;
    double                  microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 0;

    if (!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter) {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        } else {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) QueryPerformanceCounter(&t);
    else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (double)t.QuadPart / frequencyToMicroseconds;
    t.QuadPart = microseconds;
    tp->tv_sec = t.QuadPart / 1000000;
    tp->tv_nsec = (t.QuadPart % 1000000) * 1000;
    return (0);
}
#endif

unsigned long long timeInMicroseconds(void)
{
	 struct timespec tp;
#ifdef __APPLE__
     tp = orwl_gettime();
#else
	 clock_gettime(CLOCK_REALTIME, &tp);
#endif

	 return tp.tv_sec * 1000000 + tp.tv_nsec / 1000;
}

double timeInSeconds(void)
{
    struct timespec tp;
#ifdef __APPLE__
     tp = orwl_gettime();
#else
	 clock_gettime(CLOCK_REALTIME, &tp);
#endif

	 return static_cast<double>(tp.tv_sec) +
			 static_cast<double>(tp.tv_nsec) / 1000000000.0;
}

float colormapAutumn[128][3] =
{
		{1.0f,0.f,0.f},
		{1.0f,0.007874f,0.f},
		{1.0f,0.015748f,0.f},
		{1.0f,0.023622f,0.f},
		{1.0f,0.031496f,0.f},
		{1.0f,0.03937f,0.f},
		{1.0f,0.047244f,0.f},
		{1.0f,0.055118f,0.f},
		{1.0f,0.062992f,0.f},
		{1.0f,0.070866f,0.f},
		{1.0f,0.07874f,0.f},
		{1.0f,0.086614f,0.f},
		{1.0f,0.094488f,0.f},
		{1.0f,0.10236f,0.f},
		{1.0f,0.11024f,0.f},
		{1.0f,0.11811f,0.f},
		{1.0f,0.12598f,0.f},
		{1.0f,0.13386f,0.f},
		{1.0f,0.14173f,0.f},
		{1.0f,0.14961f,0.f},
		{1.0f,0.15748f,0.f},
		{1.0f,0.16535f,0.f},
		{1.0f,0.17323f,0.f},
		{1.0f,0.1811f,0.f},
		{1.0f,0.18898f,0.f},
		{1.0f,0.19685f,0.f},
		{1.0f,0.20472f,0.f},
		{1.0f,0.2126f,0.f},
		{1.0f,0.22047f,0.f},
		{1.0f,0.22835f,0.f},
		{1.0f,0.23622f,0.f},
		{1.0f,0.24409f,0.f},
		{1.0f,0.25197f,0.f},
		{1.0f,0.25984f,0.f},
		{1.0f,0.26772f,0.f},
		{1.0f,0.27559f,0.f},
		{1.0f,0.28346f,0.f},
		{1.0f,0.29134f,0.f},
		{1.0f,0.29921f,0.f},
		{1.0f,0.30709f,0.f},
		{1.0f,0.31496f,0.f},
		{1.0f,0.32283f,0.f},
		{1.0f,0.33071f,0.f},
		{1.0f,0.33858f,0.f},
		{1.0f,0.34646f,0.f},
		{1.0f,0.35433f,0.f},
		{1.0f,0.3622f,0.f},
		{1.0f,0.37008f,0.f},
		{1.0f,0.37795f,0.f},
		{1.0f,0.38583f,0.f},
		{1.0f,0.3937f,0.f},
		{1.0f,0.40157f,0.f},
		{1.0f,0.40945f,0.f},
		{1.0f,0.41732f,0.f},
		{1.0f,0.4252f,0.f},
		{1.0f,0.43307f,0.f},
		{1.0f,0.44094f,0.f},
		{1.0f,0.44882f,0.f},
		{1.0f,0.45669f,0.f},
		{1.0f,0.46457f,0.f},
		{1.0f,0.47244f,0.f},
		{1.0f,0.48031f,0.f},
		{1.0f,0.48819f,0.f},
		{1.0f,0.49606f,0.f},
		{1.0f,0.50394f,0.f},
		{1.0f,0.51181f,0.f},
		{1.0f,0.51969f,0.f},
		{1.0f,0.52756f,0.f},
		{1.0f,0.53543f,0.f},
		{1.0f,0.54331f,0.f},
		{1.0f,0.55118f,0.f},
		{1.0f,0.55906f,0.f},
		{1.0f,0.56693f,0.f},
		{1.0f,0.5748f,0.f},
		{1.0f,0.58268f,0.f},
		{1.0f,0.59055f,0.f},
		{1.0f,0.59843f,0.f},
		{1.0f,0.6063f,0.f},
		{1.0f,0.61417f,0.f},
		{1.0f,0.62205f,0.f},
		{1.0f,0.62992f,0.f},
		{1.0f,0.6378f,0.f},
		{1.0f,0.64567f,0.f},
		{1.0f,0.65354f,0.f},
		{1.0f,0.66142f,0.f},
		{1.0f,0.66929f,0.f},
		{1.0f,0.67717f,0.f},
		{1.0f,0.68504f,0.f},
		{1.0f,0.69291f,0.f},
		{1.0f,0.70079f,0.f},
		{1.0f,0.70866f,0.f},
		{1.0f,0.71654f,0.f},
		{1.0f,0.72441f,0.f},
		{1.0f,0.73228f,0.f},
		{1.0f,0.74016f,0.f},
		{1.0f,0.74803f,0.f},
		{1.0f,0.75591f,0.f},
		{1.0f,0.76378f,0.f},
		{1.0f,0.77165f,0.f},
		{1.0f,0.77953f,0.f},
		{1.0f,0.7874f,0.f},
		{1.0f,0.79528f,0.f},
		{1.0f,0.80315f,0.f},
		{1.0f,0.81102f,0.f},
		{1.0f,0.8189f,0.f},
		{1.0f,0.82677f,0.f},
		{1.0f,0.83465f,0.f},
		{1.0f,0.84252f,0.f},
		{1.0f,0.85039f,0.f},
		{1.0f,0.85827f,0.f},
		{1.0f,0.86614f,0.f},
		{1.0f,0.87402f,0.f},
		{1.0f,0.88189f,0.f},
		{1.0f,0.88976f,0.f},
		{1.0f,0.89764f,0.f},
		{1.0f,0.90551f,0.f},
		{1.0f,0.91339f,0.f},
		{1.0f,0.92126f,0.f},
		{1.0f,0.92913f,0.f},
		{1.0f,0.93701f,0.f},
		{1.0f,0.94488f,0.f},
		{1.0f,0.95276f,0.f},
		{1.0f,0.96063f,0.f},
		{1.0f,0.9685f,0.f},
		{1.0f,0.97638f,0.f},
		{1.0f,0.98425f,0.f},
		{1.0f,0.99213f,0.f},
		{1.0f,1.0f,0.0f}
};


float colormapJet[128][3] =
{
		{0.0f,0.0f,0.53125f},
		{0.0f,0.0f,0.5625f},
		{0.0f,0.0f,0.59375f},
		{0.0f,0.0f,0.625f},
		{0.0f,0.0f,0.65625f},
		{0.0f,0.0f,0.6875f},
		{0.0f,0.0f,0.71875f},
		{0.0f,0.0f,0.75f},
		{0.0f,0.0f,0.78125f},
		{0.0f,0.0f,0.8125f},
		{0.0f,0.0f,0.84375f},
		{0.0f,0.0f,0.875f},
		{0.0f,0.0f,0.90625f},
		{0.0f,0.0f,0.9375f},
		{0.0f,0.0f,0.96875f},
		{0.0f,0.0f,1.0f},
		{0.0f,0.03125f,1.0f},
		{0.0f,0.0625f,1.0f},
		{0.0f,0.09375f,1.0f},
		{0.0f,0.125f,1.0f},
		{0.0f,0.15625f,1.0f},
		{0.0f,0.1875f,1.0f},
		{0.0f,0.21875f,1.0f},
		{0.0f,0.25f,1.0f},
		{0.0f,0.28125f,1.0f},
		{0.0f,0.3125f,1.0f},
		{0.0f,0.34375f,1.0f},
		{0.0f,0.375f,1.0f},
		{0.0f,0.40625f,1.0f},
		{0.0f,0.4375f,1.0f},
		{0.0f,0.46875f,1.0f},
		{0.0f,0.5f,1.0f},
		{0.0f,0.53125f,1.0f},
		{0.0f,0.5625f,1.0f},
		{0.0f,0.59375f,1.0f},
		{0.0f,0.625f,1.0f},
		{0.0f,0.65625f,1.0f},
		{0.0f,0.6875f,1.0f},
		{0.0f,0.71875f,1.0f},
		{0.0f,0.75f,1.0f},
		{0.0f,0.78125f,1.0f},
		{0.0f,0.8125f,1.0f},
		{0.0f,0.84375f,1.0f},
		{0.0f,0.875f,1.0f},
		{0.0f,0.90625f,1.0f},
		{0.0f,0.9375f,1.0f},
		{0.0f,0.96875f,1.0f},
		{0.0f,1.0f,1.0f},
		{0.03125f,1.0f,0.96875f},
		{0.0625f,1.0f,0.9375f},
		{0.09375f,1.0f,0.90625f},
		{0.125f,1.0f,0.875f},
		{0.15625f,1.0f,0.84375f},
		{0.1875f,1.0f,0.8125f},
		{0.21875f,1.0f,0.78125f},
		{0.25f,1.0f,0.75f},
		{0.28125f,1.0f,0.71875f},
		{0.3125f,1.0f,0.6875f},
		{0.34375f,1.0f,0.65625f},
		{0.375f,1.0f,0.625f},
		{0.40625f,1.0f,0.59375f},
		{0.4375f,1.0f,0.5625f},
		{0.46875f,1.0f,0.53125f},
		{0.5f,1.0f,0.5f},
		{0.53125f,1.0f,0.46875f},
		{0.5625f,1.0f,0.4375f},
		{0.59375f,1.0f,0.40625f},
		{0.625f,1.0f,0.375f},
		{0.65625f,1.0f,0.34375f},
		{0.6875f,1.0f,0.3125f},
		{0.71875f,1.0f,0.28125f},
		{0.75f,1.0f,0.25f},
		{0.78125f,1.0f,0.21875f},
		{0.8125f,1.0f,0.1875f},
		{0.84375f,1.0f,0.15625f},
		{0.875f,1.0f,0.125f},
		{0.90625f,1.0f,0.09375f},
		{0.9375f,1.0f,0.0625f},
		{0.96875f,1.0f,0.03125f},
		{1.0f,1.0f,0.0f},
		{1.0f,0.96875f,0.0f},
		{1.0f,0.9375f,0.0f},
		{1.0f,0.90625f,0.0f},
		{1.0f,0.875f,0.0f},
		{1.0f,0.84375f,0.0f},
		{1.0f,0.8125f,0.0f},
		{1.0f,0.78125f,0.0f},
		{1.0f,0.75f,0.0f},
		{1.0f,0.71875f,0.0f},
		{1.0f,0.6875f,0.0f},
		{1.0f,0.65625f,0.0f},
		{1.0f,0.625f,0.0f},
		{1.0f,0.59375f,0.0f},
		{1.0f,0.5625f,0.0f},
		{1.0f,0.53125f,0.0f},
		{1.0f,0.5f,0.0f},
		{1.0f,0.46875f,0.0f},
		{1.0f,0.4375f,0.0f},
		{1.0f,0.40625f,0.0f},
		{1.0f,0.375f,0.0f},
		{1.0f,0.34375f,0.0f},
		{1.0f,0.3125f,0.0f},
		{1.0f,0.28125f,0.0f},
		{1.0f,0.25f,0.0f},
		{1.0f,0.21875f,0.0f},
		{1.0f,0.1875f,0.0f},
		{1.0f,0.15625f,0.0f},
		{1.0f,0.125f,0.0f},
		{1.0f,0.09375f,0.0f},
		{1.0f,0.0625f,0.0f},
		{1.0f,0.03125f,0.0f},
		{1.0f,0.0f,0.0f},
		{0.96875f,0.0f,0.0f},
		{0.9375f,0.0f,0.0f},
		{0.90625f,0.0f,0.0f},
		{0.875f,0.0f,0.0f},
		{0.84375f,0.0f,0.0f},
		{0.8125f,0.0f,0.0f},
		{0.78125f,0.0f,0.0f},
		{0.75f,0.0f,0.0f},
		{0.71875f,0.0f,0.0f},
		{0.6875f,0.0f,0.0f},
		{0.65625f,0.0f,0.0f},
		{0.625f,0.0f,0.0f},
		{0.59375f,0.0f,0.0f},
		{0.5625f,0.0f,0.0f},
		{0.53125f,0.0f,0.0f},
		{0.5f,0.0f,0.0f}
};

void colorDepthImage(cv::Mat& imgDepth, cv::Mat& imgColoredDepth,
					 float minRange, float maxRange)
{
	imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

	for (int i = 0; i < imgColoredDepth.rows; ++i)
	{
		const float* depth = imgDepth.ptr<float>(i);
		unsigned char* pixel = imgColoredDepth.ptr<unsigned char>(i);
		for (int j = 0; j < imgColoredDepth.cols; ++j)
		{
			if (depth[j] != 0)
			{
				int idx = fminf(depth[j] - minRange, maxRange - minRange) / (maxRange - minRange) * 127.0f;
				idx = 127 - idx;

				pixel[0] = colormapJet[idx][2] * 255.0f;
				pixel[1] = colormapJet[idx][1] * 255.0f;
				pixel[2] = colormapJet[idx][0] * 255.0f;
			}

			pixel += 3;
		}
	}
}

bool colormap(const std::string& name, unsigned char idx,
			  float& r, float& g, float& b)
{
	if (name.compare("jet") == 0)
	{
		float* color = colormapJet[idx];

		r = color[0];
		g = color[1];
		b = color[2];

		return true;
	}
	else if (name.compare("autumn") == 0)
	{
		float* color = colormapAutumn[idx];

		r = color[0];
		g = color[1];
		b = color[2];

		return true;
	}

	return false;
}

std::vector<cv::Point2i> bresLine(int x0, int y0, int x1, int y1)
{
	// Bresenham's line algorithm
	// Find cells intersected by line between (x0,y0) and (x1,y1)

	std::vector<cv::Point2i> cells;

	int dx = std::abs(x1 - x0);
	int dy = std::abs(y1 - y0);

	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;

	int err = dx - dy;

	while (1)
	{
		cells.push_back(cv::Point2i(x0, y0));

		if (x0 == x1 && y0 == y1)
		{
			break;
		}

		int e2 = 2 * err;
		if (e2 > -dy)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx)
		{
			err += dx;
			y0 += sy;
		}
	}

	return cells;
}

std::vector<cv::Point2i> bresCircle(int x0, int y0, int r)
{
	// Bresenham's circle algorithm
	// Find cells intersected by circle with center (x0,y0) and radius r

	std::vector< std::vector<bool> > mask(2 * r + 1);
	
	for (int i = 0; i < 2 * r + 1; ++i)
	{
		mask[i].resize(2 * r + 1);
		for (int j = 0; j < 2 * r + 1; ++j)
		{
			mask[i][j] = false;
		}
	}

	int f = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;

	std::vector<cv::Point2i> line;

	line = bresLine(x0, y0 - r, x0, y0 + r);
	for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
	{
		mask[it->x - x0 + r][it->y - y0 + r] = true;
	}

	line = bresLine(x0 - r, y0, x0 + r, y0);
	for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
	{
		mask[it->x - x0 + r][it->y - y0 + r] = true;
	}

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		line = bresLine(x0 - x, y0 + y, x0 + x, y0 + y);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}

		line = bresLine(x0 - x, y0 - y, x0 + x, y0 - y);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}

		line = bresLine(x0 - y, y0 + x, x0 + y, y0 + x);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}

		line = bresLine(x0 - y, y0 - x, x0 + y, y0 - x);
		for (std::vector<cv::Point2i>::iterator it = line.begin(); it != line.end(); ++it)
		{
			mask[it->x - x0 + r][it->y - y0 + r] = true;
		}
	}

	std::vector<cv::Point2i> cells;
	for (int i = 0; i < 2 * r + 1; ++i)
	{
		for (int j = 0; j < 2 * r + 1; ++j)
		{
			if (mask[i][j])
			{
				cells.push_back(cv::Point2i(i - r + x0, j - r + y0));
			}
		}
	}

	return cells;
}

void
fitCircle(const std::vector<cv::Point2d>& points,
          double& centerX, double& centerY, double& radius)
{
    // D. Umbach, and K. Jones, A Few Methods for Fitting Circles to Data,
    // IEEE Transactions on Instrumentation and Measurement, 2000
    // We use the modified least squares method.
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double sum_xxx = 0.0;
    double sum_xxy = 0.0;
    double sum_xyy = 0.0;
    double sum_yyy = 0.0;

    int n = points.size();
    for (int i = 0; i < n; ++i)
    {
        double x = points.at(i).x;
        double y = points.at(i).y;

        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
        sum_xxx += x * x * x;
        sum_xxy += x * x * y;
        sum_xyy += x * y * y;
        sum_yyy += y * y * y;
    }

    double A = n * sum_xx - square(sum_x);
    double B = n * sum_xy - sum_x * sum_y;
    double C = n * sum_yy - square(sum_y);
    double D = 0.5 * (n * sum_xyy - sum_x * sum_yy + n * sum_xxx - sum_x * sum_xx);
    double E = 0.5 * (n * sum_xxy - sum_y * sum_xx + n * sum_yyy - sum_y * sum_yy);

    centerX = (D * C - B * E) / (A * C - square(B));
    centerY = (A * E - B * D) / (A * C - square(B));

    double sum_r = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double x = points.at(i).x;
        double y = points.at(i).y;

        sum_r += hypot(x - centerX, y - centerY);
    }

    radius = sum_r / n;
}

std::vector<cv::Point2d>
intersectCircles(double x1, double y1, double r1,
                 double x2, double y2, double r2)
{
    std::vector<cv::Point2d> ipts;

    double d = hypot(x1 - x2, y1 - y2);
    if (d > r1 + r2)
    {
        // circles are separate
        return ipts;
    }
    if (d < fabs(r1 - r2))
    {
        // one circle is contained within the other
        return ipts;
    }

    double a = (square(r1) - square(r2) + square(d)) / (2.0 * d);
    double h = sqrt(square(r1) - square(a));

    double x3 = x1 + a * (x2 - x1) / d;
    double y3 = y1 + a * (y2 - y1) / d;

    if (h < 1e-10)
    {
        // two circles touch at one point
        ipts.push_back(cv::Point2d(x3, y3));
        return ipts;
    }

    ipts.push_back(cv::Point2d(x3 + h * (y2 - y1) / d,
                               y3 - h * (x2 - x1) / d));
    ipts.push_back(cv::Point2d(x3 - h * (y2 - y1) / d,
                               y3 + h * (x2 - x1) / d));
    return ipts;
}

char
UTMLetterDesignator(double latitude)
{
    // This routine determines the correct UTM letter designator for the given latitude
    // returns 'Z' if latitude is outside the UTM limits of 84N to 80S
    // Written by Chuck Gantz- chuck.gantz@globalstar.com
    char letterDesignator;

    if ((84.0 >= latitude) && (latitude >= 72.0)) letterDesignator = 'X';
    else if ((72.0 > latitude) && (latitude >= 64.0)) letterDesignator = 'W';
    else if ((64.0 > latitude) && (latitude >= 56.0)) letterDesignator = 'V';
    else if ((56.0 > latitude) && (latitude >= 48.0)) letterDesignator = 'U';
    else if ((48.0 > latitude) && (latitude >= 40.0)) letterDesignator = 'T';
    else if ((40.0 > latitude) && (latitude >= 32.0)) letterDesignator = 'S';
    else if ((32.0 > latitude) && (latitude >= 24.0)) letterDesignator = 'R';
    else if ((24.0 > latitude) && (latitude >= 16.0)) letterDesignator = 'Q';
    else if ((16.0 > latitude) && (latitude >= 8.0)) letterDesignator = 'P';
    else if (( 8.0 > latitude) && (latitude >= 0.0)) letterDesignator = 'N';
    else if (( 0.0 > latitude) && (latitude >= -8.0)) letterDesignator = 'M';
    else if ((-8.0 > latitude) && (latitude >= -16.0)) letterDesignator = 'L';
    else if ((-16.0 > latitude) && (latitude >= -24.0)) letterDesignator = 'K';
    else if ((-24.0 > latitude) && (latitude >= -32.0)) letterDesignator = 'J';
    else if ((-32.0 > latitude) && (latitude >= -40.0)) letterDesignator = 'H';
    else if ((-40.0 > latitude) && (latitude >= -48.0)) letterDesignator = 'G';
    else if ((-48.0 > latitude) && (latitude >= -56.0)) letterDesignator = 'F';
    else if ((-56.0 > latitude) && (latitude >= -64.0)) letterDesignator = 'E';
    else if ((-64.0 > latitude) && (latitude >= -72.0)) letterDesignator = 'D';
    else if ((-72.0 > latitude) && (latitude >= -80.0)) letterDesignator = 'C';
    else letterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

    return letterDesignator;
}

void
LLtoUTM(double latitude, double longitude,
        double& utmNorthing, double& utmEasting, std::string& utmZone)
{
    // converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
    // East Longitudes are positive, West longitudes are negative.
    // North latitudes are positive, South latitudes are negative
    // Lat and Long are in decimal degrees
    // Written by Chuck Gantz- chuck.gantz@globalstar.com

    double k0 = 0.9996;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    double LatRad = latitude * M_PI / 180.0;
    double LongRad = longitude * M_PI / 180.0;
    double LongOriginRad;

    int ZoneNumber = static_cast<int>((longitude + 180.0) / 6.0) + 1;

    if (latitude >= 56.0 && latitude < 64.0 &&
            longitude >= 3.0 && longitude < 12.0) {
        ZoneNumber = 32;
    }

    // Special zones for Svalbard
    if (latitude >= 72.0 && latitude < 84.0) {
        if (     longitude >= 0.0  && longitude <  9.0) ZoneNumber = 31;
        else if (longitude >= 9.0  && longitude < 21.0) ZoneNumber = 33;
        else if (longitude >= 21.0 && longitude < 33.0) ZoneNumber = 35;
        else if (longitude >= 33.0 && longitude < 42.0) ZoneNumber = 37;
    }
    LongOrigin = static_cast<double>((ZoneNumber - 1) * 6 - 180 + 3);  //+3 puts origin in middle of zone
    LongOriginRad = LongOrigin * M_PI / 180.0;

    // compute the UTM Zone from the latitude and longitude
    std::ostringstream oss;
    oss << ZoneNumber << UTMLetterDesignator(latitude);
    utmZone = oss.str();

    eccPrimeSquared = WGS84_ECCSQ / (1.0 - WGS84_ECCSQ);

    N = WGS84_A / sqrt(1.0 - WGS84_ECCSQ * sin(LatRad) * sin(LatRad));
    T = tan(LatRad) * tan(LatRad);
    C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
    A = cos(LatRad) * (LongRad - LongOriginRad);

    M = WGS84_A * ((1.0 - WGS84_ECCSQ / 4.0
                    - 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 64.0
                    - 5.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256.0)
                   * LatRad
                   - (3.0 * WGS84_ECCSQ / 8.0
                      + 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 32.0
                      + 45.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024.0)
                   * sin(2.0 * LatRad)
                   + (15.0 * WGS84_ECCSQ * WGS84_ECCSQ / 256.0
                      + 45.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024.0)
                   * sin(4.0 * LatRad)
                   - (35.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072.0)
                   * sin(6.0 * LatRad));

    utmEasting = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0
                           + (5.0 - 18.0 * T + T * T + 72.0 * C
                              - 58.0 * eccPrimeSquared)
                           * A * A * A * A * A / 120.0)
                 + 500000.0;

    utmNorthing = k0 * (M + N * tan(LatRad) *
                        (A * A / 2.0 +
                         (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0
                         + (61.0 - 58.0 * T + T * T + 600.0 * C
                            - 330.0 * eccPrimeSquared)
                         * A * A * A * A * A * A / 720.0));
    if (latitude < 0.0) {
        utmNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
    }
}

void
UTMtoLL(double utmNorthing, double utmEasting, const std::string& utmZone,
        double& latitude, double& longitude)
{
    // converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
    // East Longitudes are positive, West longitudes are negative.
    // North latitudes are positive, South latitudes are negative
    // Lat and Long are in decimal degrees.
    // Written by Chuck Gantz- chuck.gantz@globalstar.com

    double k0 = 0.9996;
    double eccPrimeSquared;
    double e1 = (1.0 - sqrt(1.0 - WGS84_ECCSQ)) / (1.0 + sqrt(1.0 - WGS84_ECCSQ));
    double N1, T1, C1, R1, D, M;
    double LongOrigin;
    double mu, phi1, phi1Rad;
    double x, y;
    int ZoneNumber;
    char ZoneLetter;
    bool NorthernHemisphere;

    x = utmEasting - 500000.0; //remove 500,000 meter offset for longitude
    y = utmNorthing;

    std::istringstream iss(utmZone);
    iss >> ZoneNumber >> ZoneLetter;
    if ((static_cast<int>(ZoneLetter) - static_cast<int>('N')) >= 0) {
        NorthernHemisphere = true;//point is in northern hemisphere
    } else {
        NorthernHemisphere = false;//point is in southern hemisphere
        y -= 10000000.0;//remove 10,000,000 meter offset used for southern hemisphere
    }

    LongOrigin = (ZoneNumber - 1.0) * 6.0 - 180.0 + 3.0;  //+3 puts origin in middle of zone

    eccPrimeSquared = WGS84_ECCSQ / (1.0 - WGS84_ECCSQ);

    M = y / k0;
    mu = M / (WGS84_A * (1.0 - WGS84_ECCSQ / 4.0
                         - 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 64.0
                         - 5.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256.0));

    phi1Rad = mu + (3.0 * e1 / 2.0 - 27.0 * e1 * e1 * e1 / 32.0) * sin(2.0 * mu)
              + (21.0 * e1 * e1 / 16.0 - 55.0 * e1 * e1 * e1 * e1 / 32.0)
              * sin(4.0 * mu)
              + (151.0 * e1 * e1 * e1 / 96.0) * sin(6.0 * mu);
    phi1 = phi1Rad / M_PI * 180.0;

    N1 = WGS84_A / sqrt(1.0 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad));
    T1 = tan(phi1Rad) * tan(phi1Rad);
    C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
    R1 = WGS84_A * (1.0 - WGS84_ECCSQ) /
         pow(1.0 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad), 1.5);
    D = x / (N1 * k0);

    latitude = phi1Rad - (N1 * tan(phi1Rad) / R1)
               * (D * D / 2.0 - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1
                                 - 9.0 * eccPrimeSquared) * D * D * D * D / 24.0
                  + (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1
                     - 252.0 * eccPrimeSquared - 3.0 * C1 * C1)
                  * D * D * D * D * D * D / 720.0);
    latitude *= 180.0 / M_PI;

    longitude = (D - (1.0 + 2.0 * T1 + C1) * D * D * D / 6.0
                 + (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1
                    + 8.0 * eccPrimeSquared + 24.0 * T1 * T1)
                 * D * D * D * D * D / 120.0) / cos(phi1Rad);
    longitude = LongOrigin + longitude / M_PI * 180.0;
}

long int
timestampDiff(uint64_t t1, uint64_t t2)
{
    if (t2 > t1)
    {
        uint64_t d = t2 - t1;

        if (d > std::numeric_limits<long int>::max())
        {
            return std::numeric_limits<long int>::max();
        }
        else
        {
            return d;
        }
    }
    else
    {
        uint64_t d = t1 - t2;

        if (d > std::numeric_limits<long int>::max())
        {
            return std::numeric_limits<long int>::min();
        }
        else
        {
            return - static_cast<long int>(d);
        }
    }
}

}
