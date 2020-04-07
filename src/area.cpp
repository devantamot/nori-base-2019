#include <nori/emitter.h>
#include <nori/scene.h>
#include <nori/mesh.h>
#include <nori/dpdf.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
public:
    Mesh *m_mesh;

    AreaLight(const PropertyList& props) {
        m_radiance = props.getColor("radiance");
        //cout << "\tAREA LIGHT CONSTRUCTOR\n";
    }

    void AreaLight::activate(Mesh *m) {
        m_mesh = m;
        m_dpdf = DiscretePDF(m_mesh->getTriangleCount());

        for (int i = 0; i < m->getTriangleCount(); i++) {
            m_dpdf.append(m_mesh->surfaceArea(i));
        }
        m_dpdf.normalize();
        //cout << "\tAREALIGHT activate\n";
    }

    void AreaLight::sample(EmitQueryRecord& bRec, const Point2f& sample) {

        //cout << "\tAsking AreaLight For a sample\n";
        // Sampling proportional to the surface area
        float triIndexSample = ((float)rand()) / RAND_MAX;
        float triIndex = m_dpdf.sampleReuse(triIndexSample);
        //cout << "Using index " << triIndex << " & "<< triIndexSample<<"\n";
        // Barycentric stuff
        float alpha = 1 - sqrt(1 - sample[0]);
        float beta = sample[1] * sqrt(1 - sample[0]);

        Vector3f baryCoord;
        baryCoord[0] = alpha;
        baryCoord[1] = beta;
        baryCoord[2] = 1 - alpha - beta;
        
        /*
        bRec.m_p = m_mesh->getCentroid(triIndex);
        
        // No idea if this works
        for (int i = 0; i < 3; i++) {
            bRec.m_p[i] *= baryCoord[i];
        }*/

        // POSSIBLE SOLUTION
        bRec.m_p = Point3f();
        for (int i = 0; i < 3; i++) {
            bRec.m_p += baryCoord[i] * m_mesh->getVertexPositions().col(m_mesh->getIndices()(i, triIndex));
        }

        // Adding the normal 
        // Per vertex case
        if (m_mesh->getVertexNormals().size() > 0) {
            bRec.m_np = Vector3f(0, 0, 0);
            for (int i = 0; i < 3; i++) {
                bRec.m_np += baryCoord[i] * m_mesh->getVertexNormals().col(m_mesh->getIndices()(i, triIndex));
            }
        }
        else {
            Vector3f a = m_mesh->getVertexPositions().col(m_mesh->getIndices()(1, triIndex)) - m_mesh->getVertexPositions().col(m_mesh->getIndices()(0, triIndex));
            Vector3f b = m_mesh->getVertexPositions().col(m_mesh->getIndices()(2, triIndex)) - m_mesh->getVertexPositions().col(m_mesh->getIndices()(1, triIndex));
            bRec.m_np = a.cross(b);
            for (int i = 0; i < 3; i++) {
                bRec.m_np[i] *= baryCoord[i];
            }
        }

        bRec.m_np /= bRec.m_np.norm();
        //cout << "\t\tGot the Sample!\n";
        
    }

    std::string toString() const {
        return "AreaLight[]";
    }
};
NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END