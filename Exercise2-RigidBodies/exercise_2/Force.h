#ifndef FORCE_H
#define FORCE_H
#include "util/vectorbase.h"

using GamePhysics::Vec3;

class Force {
public:
	Vec3 m_Location;
	Vec3 m_Size;
	Force(const Vec3& location, const Vec3& size)
		: m_Location(location)
		, m_Size(size)
	{}
};
#endif
