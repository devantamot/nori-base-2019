#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/mesh.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/dpdf.h>
#include <nori/warp.h>
#include <stdlib.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:

    PathMatsIntegrator(const PropertyList& props) {
        // Do nothing
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

        Color3f Le(0.0f), alpha(1.0f);
        float eta = 1.f;
        bool specularBounce = false;

        std::vector<Mesh*> emitterMeshes = std::vector<Mesh*>(); // meshes that have emitters
        DiscretePDF dpdf = DiscretePDF();
        dpdf.clear();

        for (Mesh* m : scene->getMeshes()) {
            if (m->isEmitter()) {
                dpdf.append(1.0f);
                emitterMeshes.push_back(m);
            }
        }

        dpdf.normalize();

        Intersection hitRecord;
        Ray3f pathRay(ray.o, ray.d);

        while(true)
        {
            if (!scene->rayIntersect(pathRay, hitRecord))
                return Le;

            const Emitter* em = hitRecord.mesh->getEmitter();

            // Checks to see if we hit a light source
            if (em != nullptr)
               Le += alpha * em->getEmission();

            // This will get the query a bsdf based on the wi
            BSDFQueryRecord query(hitRecord.toLocal((pathRay.o - hitRecord.p).normalized()));
            
            float prob = fmin(fmax(fmax(alpha[0], alpha[1]), alpha[2]) * pow(eta, 2), 0.99);
        
            if (sampler->next1D() > prob)
                return Le;

            alpha /= prob; // decreases the amount of contribution as multiple bounces occur

            Color3f bsdf = hitRecord.mesh->getBSDF()->sample(query, sampler->next2D());
            // After calculating the bsdf, the wo gets updated in query.wo so this will be our new direction
            pathRay = Ray3f(hitRecord.p,hitRecord.toWorld(query.wo));

            alpha *= bsdf;
        }
    }

    std::string toString() const {
        return "PathMatsIntegrator[]";
    }

};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END