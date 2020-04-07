#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/mesh.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/dpdf.h>
#include <nori/warp.h>
#include <stdlib.h>

NORI_NAMESPACE_BEGIN

class PathEmsIntegrator : public Integrator {
public:

    PathEmsIntegrator(const PropertyList& props) {
        // Do nothing
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

        Color3f c(0.0f), alpha(1.0f);
        float eta = 1.f;

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

        if (!scene->rayIntersect(pathRay, hitRecord))
            return c;


        for (int k = 0; ; ++k) {

            const BSDF* bsdf = hitRecord.mesh->getBSDF();
            const Emitter* em = hitRecord.mesh->getEmitter();

            BSDFQueryRecord bRec(Vector3f(0.0f, 0.0f, 0.0f));

            // If it hits light source, it adds it
            if (em != nullptr) {
                EmitQueryRecord eRec(hitRecord.shFrame.toLocal(-pathRay.d), hitRecord.p, hitRecord.shFrame.n);
                c = em->getEmission(); // add or reassign?
                bRec = BSDFQueryRecord(hitRecord.shFrame.toLocal(-pathRay.d));
            }
            else {
                float emitterPDF;
                Mesh* eMesh = emitterMeshes.at(dpdf.sample(sampler->next1D(), emitterPDF)); // chooses a random emitter to sample from

                Emitter* x = eMesh->getEmitter();

                EmitQueryRecord eqr = EmitQueryRecord();
                x->sample(eqr, sampler->next2D());

                Vector3f nextDir = eqr.m_p - hitRecord.p;
                nextDir.normalize();
                
                bRec = BSDFQueryRecord(hitRecord.shFrame.toLocal(-pathRay.d), hitRecord.shFrame.toLocal(nextDir), ESolidAngle);
            

            }

            // This is probably wrong
            Color3f xcolor = bsdf->sample(bRec, sampler->next2D());
            c += alpha * xcolor /* Frame::cosTheta(nextDir)*/;// / bsdf->pdf(bRec);

            // Breaks with probability
            float prob = fmin(fmax(fmax(alpha[0], alpha[1]), alpha[2]) * pow(eta, 2), 0.99);
            if (sampler->next1D() < prob) {
                break;
            }

            pathRay = Ray3f(hitRecord.p, hitRecord.shFrame.toWorld(bRec.wo));
            if (!scene->rayIntersect(pathRay, hitRecord))
                break;
            //*= bsdf->eval(BSDFQueryRecord(bRec.wi, bRec.wo, ESolidAngle))
            alpha /= (1 - prob);
        }

        return c;
    }

    std::string toString() const {
        return "PathEmsIntegrator[]";
    }

};

NORI_REGISTER_CLASS(PathEmsIntegrator, "path_ems");
NORI_NAMESPACE_END