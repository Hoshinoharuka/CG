#pragma once
#ifndef __ONB_HPP__
#define __ONB_HPP__

#include "geometry/vec.hpp"

namespace LYPathTracer
{
    using namespace NRenderer;
    class Onb
    {
    private:
        Vec3 u;
        Vec3 v;
        Vec3 w;
        Mat3x3 matrix;
    public:
        Onb(const Vec3& normal) {
            w = normal;
            Vec3 a = (fabs(w.x) > 0.9) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
            v = glm::normalize(glm::cross(w, a));
            u = glm::cross(w, v);
            Vec3 a1 = Vec3{ u.x, u.y, u.z };
            Vec3 b = Vec3{ v.x, v.y, v.z };
            Vec3 c = Vec3{ w.x, w.y, w.z };
            Mat3x3 matrix_temp = {
                u.x, u.y, u.z,
                v.x, v.y, v.z,
                w.x, w.y, w.z
            };// = { a1, b, c };
            //matrix_temp[0] = a1;
           // matrix_temp[1] = b;
            //matrix_temp[2] = c;
            Mat3x3 temp = matrix_temp;
            matrix = glm::transpose(temp);
        }
        ~Onb() = default;

        Vec3 local(const Vec3& v) const {
            return v.x*this->u + v.y*this->v + v.z * this->w;
        }

        Vec3 un_local(const Vec3& input, const Vec3& normal) const {
            //return glm::normalize(glm::reflect(input, normal));
            return matrix * input;
            //return v.x * x + v.y * y + v.z * z;
        }
    };
}


#endif