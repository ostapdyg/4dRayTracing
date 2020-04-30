#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

double x1 = 50;
double x2 = 150;
double y1 = 0;
double y2 = 150;
double dx1 = 10;
double dx2 = 20;
double dy1 = 0;
double dy2 = 50;

double r1 = 5;
double r2 = 20;

double g = 100;

class Example : public olc::PixelGameEngine
{
public:
	Example()
	{
		// Name you application
		sAppName = "Example";
	}

public:
	bool OnUserCreate() override
	{
		// Called once at the start, so create things here
		for (int x = 0; x < ScreenWidth(); x++)
			for (int y = 0; y < ScreenHeight(); y++)
				Draw(x, y, olc::BLACK);
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		// called once per frame

		DrawLine(olc::vi2d(x1, y1), olc::vi2d(x1 + dx1 * fElapsedTime, y1 + dy1 * fElapsedTime), olc::RED);
		DrawLine(olc::vi2d(x2, y2), olc::vi2d(x2 + dx2 * fElapsedTime, y2 + dy2 * fElapsedTime), olc::BLUE);

		x1 += dx1 * fElapsedTime;
		x2 += dx2 * fElapsedTime;
		y1 += dy1 * fElapsedTime;
		y2 += dy2 * fElapsedTime;

		if (y1 < 0 && dy1 < 0)
		{
			dy1 *= -1;
		}
		if (y2 < 0 && dy2 < 0)
		{
			dy2 *= -1;
		}
		if (x1 < 0 && dx1 < 0)
		{
			x1 = ScreenWidth() - 1;
		}
		if (x2 < 0 && dx2 < 0)
		{
			x2 = ScreenWidth() - 1;
		}

		dy1 += g * fElapsedTime;
		dy2 += g * fElapsedTime;

		if (y1 >= ScreenHeight() - 1 && dy1 > 0)
		{
			dy1 *= -0.95;
		}
		if (y2 >= ScreenHeight() - 1 && dy2 > 0)
		{
			dy2 *= -0.95;
		}
		if (x1 >= ScreenWidth() - 1 && dx1 > 0)
		{
			x1 = 0;
		}
		if (x2 >= ScreenWidth() - 1 && dx2 > 0)
		{
			x2 = 0;
		}

		// DrawCircle(olc::vi2d{x1, y1}, r1, olc::RED);
		// DrawCircle(olc::vi2d{x2, y2}, r2, olc::BLUE);

		// DrawCircle(olc::vi2d{x1, y1}, r1, olc::BLACK);
		// DrawCircle(olc::vi2d{x2, y2}, r2, olc::BLACK);
		// Draw(x1-r1, y1, olc::RED);
		// Draw(x2-r1, y2, olc::BLUE);

		return true;
	}
};

int main()
{
	Example demo;
	if (demo.Construct(1280, 720, 1, 1, 0, 1))
		demo.Start();

	return 0;
}