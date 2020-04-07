#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/mesh.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/dpdf.h>
#include <nori/warp.h>
#include <stdlib.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
public:

    WhittedIntegrator(const PropertyList& props) {
        //cout << "\tWHITEY CONSTRUCTOR\n";
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        //cout << "WHITEY Li\n";
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Point3f x = its.p; // Intersection point, x
        
        const Emitter* em = its.mesh->getEmitter();

        Color3f og = Color3f(0.0f);

        if (its.mesh->getEmitter() != nullptr)
            og = em->getEmission();

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

        if (emitterMeshes.size() > 0) {
            //cout << "\t\tMany meshies!\n";
            // pick a random emission mesh
            float emitterPDF;
            Mesh* eMesh = emitterMeshes.at(dpdf.sample(((float)rand())/RAND_MAX, emitterPDF)); // chooses a random emitter to sample from

            // Now pick a random point on that emission mesh
            Emitter* e = eMesh->getEmitter();
            EmitQueryRecord eqr = EmitQueryRecord();
            e->sample(eqr, Point2f(((float)rand()) / RAND_MAX, ((float)rand()) / RAND_MAX));

            Point3f p = eqr.m_p;
            Vector3f np = eqr.m_np;

            Point3f distP = p - its.p;
            float distVal = distP.norm();
            Vector3f distN = distP.normalized();
            Normal3f nn = its.shFrame.n;


            // Calculate if x and eqr.p are mutally visible.
            // This is done by shooting a ray from p to x and see if it intersects with anything
            float intersectVal = !scene->rayIntersect(Ray3f(x,distN, 1e-5f, distVal - 1e-5f));
            //shadowRay.maxt = ray.maxt - 0.01;

            // Test to see if the point and the mesh point are mutally visible

            //cout << "\t\tMutally visible\n";
            // Computing the fr value
            BSDFQueryRecord query(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(distN), ESolidAngle);
            const BSDF* bsdfVal = its.mesh->getBSDF();
            Color3f fr = bsdfVal->eval(query);//sample(query,Point2f(((float)rand()) / RAND_MAX, ((float)rand()) / RAND_MAX));


            float g = fmax(0.0f,-np.dot(distN)) * fabs(nn.dot(distN))
                / (distVal * distVal) * intersectVal / emitterPDF / (1.0f/e->getDPDF().normalize());
            
            // Getting the Le value
            Color3f emission = e->getEmission();

            Color3f Lr = (fr * emission * g); // calculating the Lr value

            Color3f result = Lr + og;

            if (((float)rand()) / RAND_MAX < 0.95) {
                if (!bsdfVal->isDiffuse()) {
                    Color3f c = bsdfVal->sample(query, Point2f(((float)rand()) / RAND_MAX, ((float)rand()) / RAND_MAX));
                    result += c  / 0.95f * Li(scene, sampler, Ray3f(its.p,its.shFrame.toWorld(query.wo)));
                }
            }

            return result;
        }

        return og;

    }

    std::string toString() const {
        return "SimpleIntegrator[]";
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END