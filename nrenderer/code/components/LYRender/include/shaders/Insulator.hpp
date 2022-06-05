#pragma once
#ifndef __INSULATOR_HPP__
#define __INSULATOR_HPP__

#include "Shader.hpp"

namespace LYPathTracer
{
    class Insulator : public Shader
    {
    private:
        Vec3 albedo;
    public:
        Insulator(Material& material, vector<Texture>& textures);
        float F_schlick(const Vec3& incident, const Vec3& normal, float eta) const;
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
        Scattered shade_another(const Ray& ray, const Vec3& hitPoint, const Vec3& normal, float eta) const;
    };
}

#endif