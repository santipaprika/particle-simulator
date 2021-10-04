#pragma once
#include <glm\glm.hpp>

class Plane
{
public:
	Plane();
	~Plane();
	glm::vec3 point;
	glm::vec3 direc1;
	glm::vec3 direc2;
	glm::vec3 normal = glm::cross(direc1, direc2);
	float d = -glm::dot(point, normal);
	//setter
	glm::vec3 setPlanePoint(const float& x, const float& y, const float& z);
	glm::vec3 setPlaneNormal(const float& x, const float& y, const float& z);
	glm::vec3 setPlaneDirect1(const float& x, const float& y, const float& z);
	glm::vec3 setPlaneDirect2(const float& x, const float& y, const float& z);
//	void setPlaneFrom3Points(Point p1, Point p2, Point p3);
	//functions
	float distPlaneToPoint(glm::vec3 q);
	void computePlaneNormal();
	glm::vec3 closestPointInPlane(glm::vec3 q);
	glm::vec3 entryPointSegmentPlane(glm::vec3 p, glm::vec3 q);
};

