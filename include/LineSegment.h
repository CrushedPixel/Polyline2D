#pragma once

#include "Vec2.h"
#include <optional>

namespace crushedpixel {

struct LineSegment {
	LineSegment(const Vec2 &a, const Vec2 &b) :
			a(a), b(b) {}

	Vec2 a, b;

	/**
	 * @return A copy of the line segment, offset by the given vector.
	 */
	LineSegment operator+(const Vec2 &toAdd) const {
		return {a + toAdd, b + toAdd};
	}

	/**
	 * @return A copy of the line segment, offset by the given vector.
	 */
	LineSegment operator-(const Vec2 &toRemove) const {
		return {a - toRemove, b - toRemove};
	}

	/**
	 * @return The line segment's normal vector.
	 */
	Vec2 normal() const {
		auto dir = direction();

		// return the direction vector
		// rotated by 90 degrees counter-clockwise
		return Vec2(-dir.y, dir.x);
	}

	/**
	 * @return The line segment's direction vector.
	 */
	Vec2 direction() const {
		auto dX = b.x - a.x;
		auto dY = b.y - a.y;

		return Vec2(dX, dY).normalized();
	}

	static std::optional<Vec2> intersection(const LineSegment &a, const LineSegment &b, bool infiniteLines) {
		auto r = a.b - a.a;
		auto s = b.b - b.a;

		auto originDist = b.a - a.a;

		auto uNumerator = Vec2::cross(originDist, r);
		auto denominator = Vec2::cross(r, s);

		if (std::abs(denominator) < 0.0001) {
			// The lines are parallel
			return std::nullopt;
		}

		auto u = uNumerator / denominator;
		auto t = Vec2::cross(originDist, s) / denominator;

		if (!infiniteLines && (t < 0 || t > 1 || u < 0 || u > 1)) {
			// the intersection lies outside of the line segments
			return std::nullopt;
		}

		return a.a + (r * t);
	}
};


} // namespace crushedpixel