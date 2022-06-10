#include "server/Server.hpp"

#include "LYPathTracer.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include "Onb.hpp"
#include <iostream>

namespace LYPathTracer
{
    using namespace std;
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

    void LYPathTracerRenderer::photonTask(RGBA* pixels, int width, int height, int off, int step) {
        // not really understand ,directly copy, should be updated!!
        for (int i = 0; i < photonNum; i += step) {
            auto r1 = defaultSamplerInstance<UniformInSquare>().sample2d();
            auto r2 = defaultSamplerInstance<HemiSphere>().sample3d();
            Vec3 norm{ 0,0,-1 };
            Onb onb{ norm };
            Vec3 rDir = glm::normalize(onb.local(r2));
            Vec3 rPos = scene.areaLightBuffer.begin()->position;
            rPos.x = rPos.x - 60.f + r1.x * 60.f;
            rPos.y = rPos.y - 60.f + r1.y * 60.f;
            Vec3 color = scene.areaLightBuffer.begin()->radiance;
            Ray ray = Ray(rPos, rDir);
            photonTracing(ray, color, 0);
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

        const auto taskNums = 12;
        energy = 1.f / log(samples);

        for (int k = 0; k < samples; k++) {
            Vec3 v{ 0,0,0 };
            for (int i = 0; i < height * width; i++) {
                pic[i] = v;
                sampleCount[i] = 0;
            }
            thread* t = new thread[taskNums];
            for (int i = 0; i < taskNums; i++) {
                t[i] = thread(&LYPathTracerRenderer::renderTask,
                    this, pixels, width, height, i, taskNums);
            }
            for (int i = 0; i < taskNums; i++) {
                t[i].join();
            }
            delete[] t;

            buildTree(root, viewPoints);
            t = new thread[taskNums];
            for (int i = 0; i < taskNums; i++) {
                t[i].join();
            }
            delete[] t;

            energy /= 0.9f;
            findR *= 0.9f;
            releaseTree(root);
            // not really understand but copy
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    Vec3 c = gamma(pic[i * width + j]);
                    Vec4 ori = pixels[(height - 1 - i) * width + j];
                    c = { c.x + ori.x,c.y + ori.y,c.z + ori.z };
                    pixels[(height - 1 - i) * width + j] = { c,1 };
                }
            }

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

    tuple<HitRecord, Vec3> LYPathTracerRenderer::closestHitLightpm(const Ray& r) {
        Vec3 v = {};
        HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});
        for (auto& a : scene.areaLightBuffer) {
            auto hitRecord = Intersection::xAreaLight(r, a, 0.000001, closest->t);
            if (hitRecord && closest->t > hitRecord->t) {
                closest = hitRecord;
                v = a.radiance;
            }
        }
        return { closest, v };
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
                //bool flag = glm::dot(r.direction, hitObject->normal) >= 0 ? true : false;
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
                    if (fractRay.direction.x == 0.f && fractRay.direction.y == 0.f && fractRay.direction.z == 0.f) {
                        return result1;
                    }
                    auto fractattenuation = refract.attenuation;
                    auto fractemitted = refract.emitted;
                    auto fractnext = trace(fractRay, currDepth + 1, in ? false : true);
                    float fract_n_dot_in = glm::dot(in ? hitObject->normal : -hitObject->normal, fractRay.direction);
                    //fract_n_dot_in = 1.f;
                    //cout << fract_n_dot_in << endl;
                    fract_n_dot_in *= 0.5f;
                    float fractpdf = refract.pdf;
                    auto result2 = fractemitted + fractattenuation * fractnext * fract_n_dot_in / fractpdf;
                    //return result2;
                    //cout << 1 - F << endl;
                    return F * result1 + (1 - F) * result2;
                    //return 0.5f * result1 + 0.5f * result2;
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

    void LYPathTracerRenderer::rayTracing(const Ray& r, int currDepth, float lambda, int x, int y) {
        if (lambda<1e-9 || currDepth>=depth) {
            return;
        }
        auto hitObject = closestHitObject(r);
        auto [hitLight, emitted] = closestHitLightpm(r);
        if (hitObject && hitObject->t < hitLight->t) {
            auto mtlHandle = hitObject->material;
            if (mtlHandle.index() == 2) {
                auto refract = (dynamic_pointer_cast<Insulator>(shaderPrograms[mtlHandle.index()]))
                    ->shade_another(r, hitObject->hitPoint, hitObject->normal, 1.5);
            }
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            mtx.lock();
            viewPoints.push_back(ViewPoint(hitObject->hitPoint,    
                hitObject->normal,      
                scattered.attenuation,  // color
                lambda * scattered.pdf,    // strength
                x, y));
            mtx.unlock();
        }
        else if (hitLight->t != FLOAT_INF) {
            mtx.lock();
            viewPoints.push_back(ViewPoint(hitLight->hitPoint,
                hitLight->normal,
                emitted,
                lambda,
                x,
                y));
            mtx.unlock();
        }
    }

    void LYPathTracerRenderer::photonTracing(const Ray& r, Vec3 rColor, int currDepth) {
        if (currDepth >= depth) {
            return;
        }
        auto hitObject = closestHitObject(r);
        auto [hitLight, emitted] = closestHitLightpm(r);
        if (hitObject && hitObject->t < hitLight->t) {
            auto mtlHandle = hitObject->material;
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            if (1) {
                // Diffuse reflection
            }
            else {
                //refraction?
                return;
            }

        }
        
    }

    Vec3 LYPathTracerRenderer::getMax(const Vec3& v1, const Vec3& v2) {
        return Vec3(max(v1.x, v2.x), max(v1.y, v2.y), max(v1.z, v2.z));
    }

    Vec3 LYPathTracerRenderer::getMin(const Vec3& v1, const Vec3& v2) {
        return Vec3(min(v1.x, v2.x), min(v1.y, v2.y), min(v1.z, v2.z));
    }

    void LYPathTracerRenderer::buildTree(KdTreeNode*& node, vector<ViewPoint>& list, int l, int r, int dim) {
        if (l == -1 && r == -1) {
            l = 0;
            r = list.size();
        }
        if (l > r) {
            return;
        }
        int mid = (l + r) >> 1;
        switch (dim) {
        case 0:
            nth_element(list.begin() + l, list.begin() + mid, list.begin() + r, ViewPointCompare<0>());
            break;
        case 1:
            nth_element(list.begin() + l, list.begin() + mid, list.begin() + r, ViewPointCompare<1>());
            break;
        case 2:
            nth_element(list.begin() + l, list.begin() + mid, list.begin() + r, ViewPointCompare<2>());
            break;
        }

        node = new KdTreeNode();
        node->viewpoint = list[mid];
        node->left = node->right = NULL;
        node->dim = dim;
        node->bdmax = node->viewpoint.pos;
        node->bdmin = node->viewpoint.pos;
        buildTree(node->left, list, l, mid, (dim + 1) % 3);
        if (node->left) {
            node->bdmax = getMax(node->bdmax, node->left->bdmax);
            node->bdmin = getMin(node->bdmin, node->left->bdmin);
        }
        buildTree(node->right, list, mid + 1, r, (dim + 1) % 3);
        if (node->right) {
            node->bdmax = getMax(node->bdmax, node->right->bdmax);
            node->bdmin = getMin(node->bdmin, node->right->bdmin);
        }

    }

    void LYPathTracerRenderer::releaseTree(KdTreeNode*& node) {
        if (!node)return;
        releaseTree(node->left);
        releaseTree(node->right);
        delete node;
    }
    
    void LYPathTracerRenderer::findTree(KdTreeNode* node, vector<const ViewPoint*>& result, const Vec3& pos, double r) {
        // not add max nearest num!
        double dx, dy, dz;
        if (pos.x <= node->bdmax.x && pos.x >= node->bdmin.x) {
            dx = 0;
        }
        else {
            dx = min(abs(pos.x - node->bdmax.x), abs(pos.x - node->bdmin.x));
        }
        if (pos.y <= node->bdmax.y && pos.y >= node->bdmin.y) {
            dy = 0;
        }
        else {
            dy = min(abs(pos.y - node->bdmax.y), abs(pos.y - node->bdmin.y));
        }
        if (pos.z <= node->bdmax.z && pos.z >= node->bdmin.z) {
            dz = 0;
        }
        else {
            dz = min(abs(pos.z - node->bdmax.z), abs(pos.z - node->bdmin.z));
        }
        if (dx * dx + dy * dy + dz * dz > r * r) {
            return;
        }
        if ((node->viewpoint.pos - pos).length() <= r) {
            result.push_back(&(node->viewpoint));
        }
        if (node->left) {
            findTree(node->left, result, pos, r);
        }
        if (node->right) {
            findTree(node->right, result, pos, r);
        }

    }
}