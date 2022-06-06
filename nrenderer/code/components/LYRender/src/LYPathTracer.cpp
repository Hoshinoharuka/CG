#include "server/Server.hpp"

#include "LYPathTracer.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include <iostream>

namespace LYPathTracer
{
    RGB LYPathTracerRenderer::gamma(const RGB& rgb) {
        return glm::sqrt(rgb);
    }

    void LYPathTracerRenderer::renderTask(RGBA* pixels, int width, int height, int off, int step) {
        for(int i=off; i<height; i+=step) {
            for (int j=0; j<width; j++) {
                Vec3 color{0, 0, 0};
                for (int k=0; k < samples; k++) {
                    auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                    float rx = r.x;
                    float ry = r.y;
                    float x = (float(j)+rx)/float(width);
                    float y = (float(i)+ry)/float(height);
                    auto ray = camera.shoot(x, y);
                    color += trace(ray, 0, false);
                }
                color /= samples;
                color = gamma(color);
                pixels[(height-i-1)*width+j] = {color, 1};
            }
        }
    }

    auto LYPathTracerRenderer::render() -> RenderResult {
        // shaders
        shaderPrograms.clear();
        ShaderCreator shaderCreator{};
        for (auto& m : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
        }

        RGBA* pixels = new RGBA[width*height]{};

        // 局部坐标转换成世界坐标
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        const auto taskNums = 8;
        thread t[taskNums];
        for (int i=0; i < taskNums; i++) {
            t[i] = thread(&LYPathTracerRenderer::renderTask,
                this, pixels, width, height, i, taskNums);
        }
        for(int i=0; i < taskNums; i++) {
            t[i].join();
        }
        getServer().logger.log("Done...");
        return {pixels, width, height};
    }

    void LYPathTracerRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }

    HitRecord LYPathTracerRenderer::closestHitObject(const Ray& r) {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        for (auto& s : scene.sphereBuffer) {
            auto hitRecord = Intersection::xSphere(r, s, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& t : scene.triangleBuffer) {
            auto hitRecord = Intersection::xTriangle(r, t, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& p : scene.planeBuffer) {
            auto hitRecord = Intersection::xPlane(r, p, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit; 
    }
    
    tuple<float, Vec3> LYPathTracerRenderer::closestHitLight(const Ray& r) {
        Vec3 v = {};
        HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});
        for (auto& a : scene.areaLightBuffer) {
            auto hitRecord = Intersection::xAreaLight(r, a, 0.000001, closest->t);
            if (hitRecord && closest->t > hitRecord->t) {
                closest = hitRecord;
                v = a.radiance;
            }
        }
        return { closest->t, v };
    }


    RGB LYPathTracerRenderer::trace(const Ray& r, int currDepth, bool in) {
        if (currDepth == depth) return scene.ambient.constant;
        auto hitObject = closestHitObject(r);
        auto [t, emitted] = closestHitLight(r);
        // hit object
        if (hitObject && hitObject->t < t) {
            auto mtlHandle = hitObject->material;
            //auto 
            if (dynamic_pointer_cast<Insulator>(shaderPrograms[mtlHandle.index()])) {
                float F = (dynamic_pointer_cast<Insulator>(shaderPrograms[mtlHandle.index()]))
                    ->F_schlick(r.direction, in ? -hitObject->normal : hitObject->normal, in ? 1.5 : 0.66666666);
                //std::cout << F << std::endl;
                auto scattered = (dynamic_pointer_cast<Insulator>(shaderPrograms[mtlHandle.index()]))
                    ->shade(r, hitObject->hitPoint, in ? -hitObject->normal : hitObject->normal);
                auto scatteredRay = scattered.ray;
                auto attenuation = scattered.attenuation;
                auto emitted = scattered.emitted;
                auto next = trace(scatteredRay, currDepth + 1, in);
                float n_dot_in = glm::dot(in ? -hitObject->normal : hitObject->normal, scatteredRay.direction);
                //cout << n_dot_in << endl;
                float pdf = scattered.pdf;
                auto result1 = emitted + attenuation * next * n_dot_in / pdf;
                //F = 0.5;
                //cout << F << endl;
                if (F == 1.f) {
                    return result1;
                }
                else
                {
                    auto refract = (dynamic_pointer_cast<Insulator>(shaderPrograms[mtlHandle.index()]))
                        ->shade_another(r, hitObject->hitPoint, in ? -hitObject->normal : hitObject->normal, in ? 1.5 : 0.66666666);
                    auto fractRay = refract.ray;
                    auto fractattenuation = refract.attenuation;
                    auto fractemitted = refract.emitted;
                    auto fractnext = trace(fractRay, currDepth + 1, in ? false : true);
                    float fract_n_dot_in = glm::dot(in ? hitObject->normal : -hitObject->normal, fractRay.direction);
                    //cout << fract_n_dot_in << endl;
                    float fractpdf = refract.pdf;
                    auto result2 = fractemitted + fractattenuation * fractnext * fract_n_dot_in / fractpdf;
                    //return result2;
                    return F * result1 + (1 - F) * result2;
                }
            }
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            auto scatteredRay = scattered.ray;
            auto attenuation = scattered.attenuation;
            auto emitted = scattered.emitted;
            auto next = trace(scatteredRay, currDepth + 1, in);
            float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
            //cout << n_dot_in << endl;
            float pdf = scattered.pdf;
            /**
             * emitted      - Le(p, w_0)
             * next         - Li(p, w_i)
             * n_dot_in     - cos<n, w_i>
             * atteunation  - BRDF
             * pdf          - p(w)
             **/
             //attenuation = Vec3{ 1,1,1 };
            return emitted + attenuation * next * n_dot_in / pdf;
        }
        // 
        else if (t != FLOAT_INF) {
            return emitted;
        }
        else {
            return Vec3{ 0 };
        }
    }




    RGB LYPathTracerRenderer::trace(const Ray& r, int currDepth) {
        if (currDepth == depth) return scene.ambient.constant;
        auto hitObject = closestHitObject(r);
        auto [ t, emitted ] = closestHitLight(r);
        // hit object
        if (hitObject && hitObject->t < t) {
            auto mtlHandle = hitObject->material;
            if (mtlHandle.index() == 2) {
                auto refract = (dynamic_pointer_cast<Insulator>(shaderPrograms[mtlHandle.index()]))
                    ->shade_another(r, hitObject->hitPoint, hitObject->normal, 1.5);

            }
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            auto scatteredRay = scattered.ray;
            auto attenuation = scattered.attenuation;
            auto emitted = scattered.emitted;
            auto next = trace(scatteredRay, currDepth+1);
            float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
            float pdf = scattered.pdf;
            /**
             * emitted      - Le(p, w_0)
             * next         - Li(p, w_i)
             * n_dot_in     - cos<n, w_i>
             * atteunation  - BRDF
             * pdf          - p(w)
             **/
            //attenuation = Vec3{ 1,1,1 };
            return emitted + attenuation * next * n_dot_in / pdf;
        }
        // 
        else if (t != FLOAT_INF) {
            return emitted;
        }
        else {
            return Vec3{0};
        }
    }
}