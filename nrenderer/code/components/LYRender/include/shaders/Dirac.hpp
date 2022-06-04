#pragma once
#ifndef __DIRAC_HPP__
#define __DIRAC_HPP__

#include "Shader.hpp"

namespace LYPathTracer
{
    class Dirac : public Shader
    {
    private:
        Vec3 albedo;
    public:
        Dirac(Material& material, vector<Texture>& textures);
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
    };
}

#endif