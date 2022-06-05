#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Dirac.hpp"
#include "Insulator.hpp"

namespace LYPathTracer
{
    class ShaderCreator
    {
    public:
        ShaderCreator() = default;
        SharedShader create(Material& material, vector<Texture>& t) {
            SharedShader shader{nullptr};
            switch (material.type)
            {
            case 0:
                shader = make_shared<Lambertian>(material, t);
                break;
            case 2:
                shader = make_shared<Insulator>(material, t);
                break;
            case 3:
                shader = make_shared<Dirac>(material, t);
                break;
            default:
                shader = make_shared<Insulator>(material, t);
                break;
            }
            return shader;
        }
    };
}

#endif