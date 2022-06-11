#pragma once
#ifndef __LY_PATH_TRACER_HPP__
#define __LY_PATH_TRACER_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"

#include "shaders/ShaderCreator.hpp"

#include "ViewPoint.hpp"
#include <mutex>
#include <tuple>
#include <vector>
namespace LYPathTracer
{
    using namespace NRenderer;
    using namespace std;

    class LYPathTracerRenderer
    {
    public:
    private:
        struct KdTreeNode {
        public:
            ViewPoint viewpoint;
            KdTreeNode* left;
            KdTreeNode* right;
            int dim;
            Vec3 bdmax;
            Vec3 bdmin;
        };
        SharedScene spScene;
        Scene& scene;

        unsigned int width;
        unsigned int height;
        unsigned int depth;
        unsigned int samples;

        using SCam = LYPathTracer::Camera;
        SCam camera;

        vector<SharedShader> shaderPrograms;


        //---photon mapping
        mutex mtx;
        KdTreeNode* root;
        vector<ViewPoint> viewPoints;
        int photonNum = 10000;
        Vec3* pic;
        int* sampleCount;
        int round = 10;
        double energy;
        double lightStrength;
        float photonR = 4;
        float findR = 3.f;  // find near photon r(?

    public:
        LYPathTracerRenderer(SharedScene spScene)
            : spScene               (spScene)
            , scene                 (*spScene)
            , camera                (spScene->camera)
        {
            width = scene.renderOption.width;
            height = scene.renderOption.height;
            depth = scene.renderOption.depth;
            samples = scene.renderOption.samplesPerPixel;

            root = new KdTreeNode();

            lightStrength == 1.0 / log(samples);
            Vec3 v{ 0,0,0 };
            pic = new Vec3[height * width]{};
            sampleCount = new int[height * width];
            for (int i = 0; i < height * width; i++) {
                pic[i] = v;
                sampleCount[i] = 0;
            }
        }
        ~LYPathTracerRenderer() = default;

        using RenderResult = tuple<RGBA*, unsigned int, unsigned int>;
        RenderResult render();
        void release(const RenderResult& r);

    private:
        void renderTask(RGBA* pixels, int width, int height, int off, int step);
        void photonTask(RGBA* pixels, int width, int height, int off, int step);

        RGB gamma(const RGB& rgb);
        RGB trace(const Ray& ray, int currDepth, bool in);
        RGB trace(const Ray& ray, int currDepth);
        HitRecord closestHitObject(const Ray& r);
        tuple<float, Vec3> closestHitLight(const Ray& r);

        //----photon mapping
        void rayTracing(const Ray& r, int currDepth, float lambda, int x, int y);
        tuple<HitRecord, Vec3> closestHitLightpm(const Ray& r);
        Vec3 getMax(const Vec3& v1, const Vec3& v2);
        Vec3 getMin(const Vec3& v1, const Vec3& v2);
        void photonTracing(const Ray& r, Vec3 rColor, int currDepth);
        void buildTree(KdTreeNode*& node, vector<ViewPoint>& list, int l = -1, int r = -1, int dim = 0);
        void releaseTree(KdTreeNode* &node);
        void findTree(KdTreeNode* node, vector<const ViewPoint*>& result, const Vec3& pos, double r);

        template<int dim>
        class ViewPointCompare {
        public:
            bool operator()(const ViewPoint& p1, const ViewPoint& p2) {
                switch (dim) {
                case 0:
                    return p1.pos.x < p2.pos.x;
                case 1:
                    return p1.pos.y < p2.pos.y;
                case 2:
                    return p1.pos.z < p2.pos.z;
                }
            }
        };
    };
}

#endif