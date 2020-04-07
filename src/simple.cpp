#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:

    Point3f m_position;
    Color3f m_energy;

    SimpleIntegrator(const PropertyList& props) {
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Point3f x = its.p; // Intersection point, x

        // Calculate if x and p are mutally visible.
        // This is done by shooting a ray from p to x and see if it intersects with anything
        Ray3f shadowRay = Ray3f(x, (m_position - x)/((m_position - x).norm()),1e-5, (m_position - x).norm() - 1e-5);
        //shadowRay.maxt = ray.maxt - 0.01;

        if (scene->rayIntersect(shadowRay)) {
            return Color3f(0.0f);
        }

        // This gets the the cos(theta) value
        Point3f xtop = m_position - x;
        Normal3f surfNorm = its.shFrame.n;
        float cosTheta = xtop.dot(surfNorm) / (xtop.norm() * surfNorm.norm());

        float coeff = std::fmax(0,cosTheta)/((4 * M_PI * M_PI) * (x-m_position).norm() * (x - m_position).norm());

        /* Return the component-wise absolute
           value of the shading normal as a color */
        return Color3f(coeff*m_energy.x(), coeff * m_energy.y(), coeff * m_energy.z());
    }

    std::string toString() const {
        return "SimpleIntegrator[]";
    }
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END