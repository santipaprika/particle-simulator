#include "Plane.h"

Plane::Plane() : normal(0, 1, 0), d(0), point(0,0,0)
{
}

Plane::~Plane()
{
}

glm::vec3 Plane::setPlanePoint(const float& x, const float& y, const float& z){
	return glm::vec3(x, y, z);
}

glm::vec3 Plane::setPlaneDirect1(const float& x, const float& y, const float& z){
	return direc1 = glm::vec3(x, y, z);
}

glm::vec3 Plane::setPlaneDirect2(const float& x, const float& y, const float& z){
	return direc2 = glm::vec3(x, y, z);
}

glm::vec3 Plane::setPlaneNormal(const float& x, const float& y, const float& z){
	return normal = glm::vec3(x, y, z);
}

//void Plane::setPlaneFrom3Points(Point p1, Point p2, Point p3){
//	point = p1;
//	direc1 = p2.coord - p1.coord;
//	direc2 = p3.coord - p1.coord;
//	normal = glm::cross(direc1, direc2);
//	d = -glm::dot(point.coord, normal);
//}


	
void Plane::computePlaneNormal(){
	normal = glm::cross(direc1, direc2);
}

glm::vec3 Plane::closestPointInPlane(glm::vec3 q){
	float dist = distPlaneToPoint(q);
		return q - normal*dist;
}

float Plane::distPlaneToPoint(glm::vec3 q){
	float dist = glm::dot(normal, q) - glm::dot(normal, point);
	return dist = dist / glm::length(normal);
}


glm::vec3 Plane::entryPointSegmentPlane(glm::vec3 p, glm::vec3 q){
	float alfa;
	glm::vec3 r;
	alfa = (-glm::dot(normal, p) + glm::dot(normal, q - p) + glm::dot(normal, point)) / glm::dot(normal, q - p);
	r = p + (alfa-1)*(q - p);
	return r;
}
