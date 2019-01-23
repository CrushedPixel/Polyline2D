#pragma once

#include "LineSegment.h"

namespace crushedpixel {

class Polyline2D {
public:
	enum class JointStyle {
		/**
		 * Corners are drawn with sharp joints.
		 * If the joint's outer angle is too large,
		 * the joint is drawn as beveled instead,
		 * to avoid the mitre extending too far out.
		 */
		MITER,
		/**
		 * Corners are flattened.
		 */
		BEVEL,
		/**
		 * Corners are rounded off.
		 */
		ROUND
	};

	enum class EndCapStyle {
		/**
		 * Path ends are drawn flat,
		 * and don't exceed the actual end point.
		 */
		BUTT, // lol
		/**
		 * Path ends are drawn flat,
		 * but extended beyond the end point
		 * by half the line thickness.
		 */
		SQUARE,
		/**
		 * Path ends are rounded off.
		 */
		ROUND
	};

	/**
	 * Creates a vector of vertices describing a solid path through the input points.
	 * @param points The points of the path.
	 * @param thickness The path's thickness.
	 * @param jointStyle The path's joint style.
	 * @param endCapStyle The path's end cap style.
	 * @return The vertices describing the path.
	 */
	static std::vector<Vec2> create(const std::vector<Vec2> &points, float thickness,
	                                JointStyle jointStyle = JointStyle::MITER,
	                                EndCapStyle endCapStyle = EndCapStyle::BUTT) {
		return create(points, {thickness, thickness}, jointStyle, endCapStyle);
	}

	static std::vector<Vec2> create(const std::vector<Vec2> &points, Vec2 thickness,
	                                JointStyle jointStyle = JointStyle::MITER,
	                                EndCapStyle endCapStyle = EndCapStyle::BUTT) {

		// operate on half the thickness to make our lives easier
		thickness = thickness / 2;

		// create poly segments from the points
		std::vector<PolySegment> segments;
		for (size_t i = 0; i + 1 < points.size(); i++) {
			segments.emplace_back(LineSegment(points[i], points[i + 1]), thickness);
		}

		std::vector<Vec2> vertices;

		Vec2 nextStart1, nextStart2;
		Vec2 start1, start2, end1, end2;

		for (size_t i = 0; i < segments.size(); i++) {
			auto &segment = segments[i];

			// calculate start
			if (i == 0) {
				// this is the first segment
				start1 = segment.edge1.a;
				start2 = segment.edge2.a;

				if (endCapStyle == EndCapStyle::SQUARE) {
					// expand the line backwards by half the thickness
					start1 = start1 - (segment.edge1.direction() * thickness);
					start2 = start2 - (segment.edge2.direction() * thickness);

				} else if (endCapStyle == EndCapStyle::ROUND) {
					// create half circle end cap
					createTriangleFan(vertices, segment.center.a, segment.center.a,
					                  segment.edge1.a, segment.edge2.a, false);
				}
			}

			if (i + 1 == segments.size()) {
				// this is the last segment
				end1 = segment.edge1.b;
				end2 = segment.edge2.b;

				if (endCapStyle == EndCapStyle::SQUARE) {
					// expand the line by half the thickness
					end1 = end1 + (segment.edge1.direction() * thickness);
					end2 = end2 + (segment.edge2.direction() * thickness);

				} else if (endCapStyle == EndCapStyle::ROUND) {
					// create half circle end cap
					createTriangleFan(vertices, segment.center.b, segment.center.b,
					                  segment.edge1.b, segment.edge2.b, true);
				}

			} else {
				auto &nextSegment = segments[i + 1];

				auto _jointStyle = jointStyle;

				// calculate the angle between the two line segments
				auto dir1 = segment.center.direction();
				auto dir2 = nextSegment.center.direction();

				auto angle = Vec2::angle(dir1, dir2);

				// wrap the angle around the 180° mark if it exceeds 90°
				// for minimum angle detection
				auto wrappedAngle = angle;
				if (wrappedAngle > pi / 2) {
					wrappedAngle = pi - wrappedAngle;
				}

				if (_jointStyle == JointStyle::MITER && wrappedAngle < miterMinAngle) {
					// the minimum angle for mitered joints wasn't exceeded.
					// to avoid the intersection point being extremely far out,
					// thus producing an enormous joint like a rasta on 4/20,
					// we render the joint beveled instead.
					_jointStyle = JointStyle::BEVEL;
				}

				if (_jointStyle == JointStyle::MITER) {
					// calculate each edge's intersection point
					// with the next segment's central line
					auto sec1 = LineSegment::intersection(segment.edge1, nextSegment.edge1, true);
					auto sec2 = LineSegment::intersection(segment.edge2, nextSegment.edge2, true);

					// there is always an intersection point,
					// as we require a minimum angle for mitered joints
					end1 = *sec1;
					end2 = *sec2;

					nextStart1 = end1;
					nextStart2 = end2;

				} else {
					// joint style is either BEVEL or ROUND

					// find out which are the inner edges for this joint
					auto x1 = dir1.x;
					auto x2 = dir2.x;
					auto y1 = dir1.y;
					auto y2 = dir2.y;

					auto clockwise = x1 * y2 - x2 * y1 < 0;

					LineSegment *inner1, *inner2, *outer1, *outer2;

					// as the normal vector is rotated counter-clockwise,
					// the first edge lies to the left
					// from the central line's perspective,
					// and the second one to the right.
					if (clockwise) {
						outer1 = &segment.edge1;
						outer2 = &nextSegment.edge1;
						inner1 = &segment.edge2;
						inner2 = &nextSegment.edge2;
					} else {
						outer1 = &segment.edge2;
						outer2 = &nextSegment.edge2;
						inner1 = &segment.edge1;
						inner2 = &nextSegment.edge1;
					}

					// calculate the intersection point of the inner edges
					auto innerSecOpt = LineSegment::intersection(*inner1, *inner2, false);

					auto innerSec = innerSecOpt
					                ? *innerSecOpt
					                // for parallel lines, simply connect them directly
					                : inner1->b;

					// if there's no inner intersection, flip
					// the next start position for near-180° turns
					Vec2 innerStart;
					if (innerSecOpt) {
						innerStart = innerSec;
					} else if (angle > pi / 2) {
						innerStart = outer1->b;
					} else {
						innerStart = inner1->b;
					}

					if (clockwise) {
						end1 = outer1->b;
						end2 = innerSec;

						nextStart1 = outer2->a;
						nextStart2 = innerStart;

					} else {
						end1 = innerSec;
						end2 = outer1->b;

						nextStart1 = innerStart;
						nextStart2 = outer2->a;
					}

					// connect the intersection points according to the joint style

					if (_jointStyle == JointStyle::BEVEL) {
						// simply connect the intersection points
						vertices.push_back(outer1->b);
						vertices.push_back(outer2->a);
						vertices.push_back(innerSec);

					} else if (_jointStyle == JointStyle::ROUND) {
						// draw a circle between the ends of the outer edges,
						// centered at the actual point
						// with half the line thickness as the radius
						createTriangleFan(vertices, innerSec, segment.center.b, outer1->b, outer2->a, clockwise);
					} else {
						assert(false);
					}
				}
			}

			// emit vertices
			vertices.push_back(start1);
			vertices.push_back(start2);
			vertices.push_back(end1);

			vertices.push_back(end1);
			vertices.push_back(start2);
			vertices.push_back(end2);

			start1 = nextStart1;
			start2 = nextStart2;
		}

		return vertices;
	}

private:
	static constexpr float pi = 3.14159265358979323846f;

	/**
	 * The threshold for mitered joints.
	 * If the joint's angle is smaller than this angle,
	 * the joint will be drawn beveled instead.
	 */
	static constexpr float miterMinAngle = 0.349066; // ~20 degrees

	/**
	 * The minimum angle of a round joint's triangles.
	 */
	static constexpr float roundMinAngle = 0.174533; // ~10 degrees

	struct PolySegment {
		PolySegment(const LineSegment &center, const Vec2 &thickness) :
				center(center),
				// calculate the segment's outer edges by offsetting
				// the central line by the normal vector
				// multiplied with the thickness
				edge1(center + center.normal() * thickness),
				edge2(center - center.normal() * thickness) {}

		LineSegment center, edge1, edge2;
	};

	/**
	 * Creates a partial circle between two points.
	 * The points must be equally far away from the origin.
	 * @param vertices The vector to add vertices to.
	 * @param connectTo The position to connect the triangles to.
	 * @param origin The circle's origin.
	 * @param start The circle's starting point.
	 * @param end The circle's ending point.
	 * @param clockwise Whether the circle's rotation is clockwise.
	 */
	static void createTriangleFan(std::vector<Vec2> &vertices, Vec2 connectTo, Vec2 origin,
	                              Vec2 start, Vec2 end, bool clockwise) {

		auto point1 = start - origin;
		auto point2 = end - origin;

		// calculate the angle between the two points
		auto angle1 = atan2(point1.y, point1.x);
		auto angle2 = atan2(point2.y, point2.x);

		// ensure the outer angle is calculated
		if (clockwise) {
			if (angle2 > angle1) {
				angle2 = angle2 - 2 * pi;
			}
		} else {
			if (angle1 > angle2) {
				angle1 = angle1 - 2 * pi;
			}
		}

		auto jointAngle = angle2 - angle1;

		// calculate the amount of triangles to use for the joint
		auto numTriangles = (int) std::floor(std::abs(jointAngle) / roundMinAngle);

		// calculate the angle of each triangle
		auto triAngle = jointAngle / numTriangles;

		Vec2 startPoint = start;
		Vec2 endPoint;
		for (int t = 0; t < numTriangles; t++) {
			if (t + 1 == numTriangles) {
				// it's the last triangle - ensure it perfectly
				// connects to the next line
				endPoint = end;
			} else {
				auto rot = (t + 1) * triAngle;

				// rotate the original point around the origin
				endPoint.x = std::cos(rot) * point1.x - std::sin(rot) * point1.y;
				endPoint.y = std::sin(rot) * point1.x + std::cos(rot) * point1.y;

				// re-add the rotation origin to the target point
				endPoint = endPoint + origin;
			}

			// emit the triangle
			vertices.push_back(startPoint);
			vertices.push_back(endPoint);
			vertices.push_back(connectTo);

			startPoint = endPoint;
		}
	}
};

} // namespace crushedpixel