#pragma once

#include <cmath>

namespace crushedpixel {

struct Vec2 {
	Vec2() :
			x(0), y(0) {}

	Vec2(const float x, const float y) :
			x(x), y(y) {}

	float x, y;

	Vec2 withLength(float len) const {
		auto mag = magnitude();
		auto factor = mag / len;
		return *this / factor;
	}

	Vec2 normalized() const {
		return withLength(1);
	}

	float magnitude() const {
		return std::sqrt(x * x + y * y);
	}

	Vec2 operator*(const Vec2 &other) const {
		return {x * other.x, y * other.y};
	}

	Vec2 operator+(const Vec2 &other) const {
		return {x + other.x, y + other.y};
	}

	Vec2 operator-(const Vec2 &other) const {
		return {x - other.x, y - other.y};
	}

	Vec2 operator*(float factor) const {
		return {x * factor, y * factor};
	}

	Vec2 operator/(float factor) const {
		return {x / factor, y / factor};
	}

	/**
	 * Calculates the dot product of two vectors.
	 */
	static float dot(const Vec2 &a, const Vec2 &b) {
		return a.x * b.x + a.y * b.y;
	}

	/**
	 * Calculates the cross product of two vectors.
	 */
	static float cross(const Vec2 &a, const Vec2 &b) {
		return a.x * b.y - a.y * b.x;
	}

	/**
	 * Calculates the angle between two vectors.
	 */
	static float angle(const Vec2 &a, const Vec2 &b) {
		return std::acos(dot(a, b) / (a.magnitude() * b.magnitude()));
	}
};

}