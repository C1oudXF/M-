#pragma once


class Location
{public:
	int x;
	int y;
	float z = 2010.0f;

	Location() {};
	Location(int x_, int y_) {
		x = x_;
		y = y_;
	}

	bool operator==(const Location obj) const
	{
		if (x == obj.x && y == obj.y)
		{
			return true;
		}
		return false;
	}
	bool operator!=(const Location obj) const {
		if (x != obj.x || y != obj.y)
		{
			return true;
		}
		return false;

	}
};

