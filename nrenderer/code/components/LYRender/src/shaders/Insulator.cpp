#include "shaders/Insulator.hpp"
#include "samplers/SamplerInstance.hpp"

#include "Onb.hpp"

namespace LYPathTracer
{
    Insulator::Insulator(Material& material, vector<Texture>& textures)
        : Shader(material, textures)
    {
        auto diffuseColor = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        if (diffuseColor) albedo = (*diffuseColor).value;
        else albedo = { 1, 1, 1 };
        //albedo = { 1, 1, 1 };
    }
    float Insulator::F_schlick(const Vec3& incident, const Vec3& normal, float eta) const {
        // args are normalized vector
        float cos_i = -glm::dot(incident, normal);
        float temp = (eta - 1.f) / (eta + 1.f);
        float F0 = temp * temp;
        // may total internal reflection and F0 doesn't change in eta>1 case
        //cout << F0 << endl;
        //cout << cos_i << endl;
        if (eta > 1.f) {
            float cos_big = 1.f - eta * eta * (1.f - cos_i * cos_i);
            //cout << cos_big << endl;
            if (cos_big < 0.f) {
                return 1.f;
            }
            else {
                //std::cout << cos_big << std::endl;
                cos_i = std::sqrt(cos_big);
                //cout << cos_i << endl;
            }
        }
        //cout << cos_i << endl;
        float weight = 1.f - cos_i;
        float temp_2 = weight * weight;
        float temp_5 = temp_2 * temp_2 * weight;
        //cout << temp_5 << endl;
        return F0 + (1.f - F0) * temp_5;
    }
    Scattered Insulator::shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const {
        Vec3 origin = hitPoint;
        //Vec3 random = defaultSamplerInstance<HemiSphere>().sample3d();
        Vec3 direction = glm::normalize(ray.direction - 2 * glm::dot(ray.direction, normal) * normal);
        float pdf = 1 / (2 * PI);
        auto attenuation = albedo / PI;
        Ray temp{ origin, direction };
        return {
            Ray{temp.at(0.01f), direction},
            attenuation,
            Vec3{0},
            pdf
        };
    }
    Scattered Insulator::shade_another(const Ray& ray, const Vec3& hitPoint, const Vec3& normal, float eta) const {
        Vec3 origin = hitPoint;
        //Vec3 random = defaultSamplerInstance<HemiSphere>().sample3d();
        float cos_in = glm::dot(ray.direction, normal);
        float sin_in2 = 1.f - cos_in * cos_in;
        float sin_tn2 = eta * eta * sin_in2;
        float cos_tn2 = 1.f - sin_tn2;
        if (cos_tn2 < 0) {
            Vec3 direction = Vec3{ 0.f, 0.f, 0.f };
            float pdf = 1 / (2 * PI);
            auto attenuation = albedo / PI;
            Ray temp{ origin, direction };
            return {
            Ray{temp.at(0.1f), direction},
            attenuation,
            Vec3{0},
            pdf
            };
        }
        float cos_tn = sqrt(cos_tn2);
        Vec3 direction = glm::normalize(eta * ray.direction - (eta * cos_in + cos_tn) * normal);
        float pdf = 1 / (2 * PI);
        auto attenuation = albedo / PI;
        Ray temp{ origin, direction };
        return {
            Ray{temp.at(0.1f), direction},
            attenuation,
            Vec3{0},
            pdf
        };
    }
}