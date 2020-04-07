/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <nori/object.h>
#include <nori/dpdf.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Convenience data structure used to pass multiple
 * parameters to the evaluation and sampling routines in \ref BSDF
 */
    struct EmitQueryRecord {
    /// Surface Position on Mesh 
    Point3f m_p;

    /// Normal direction 
    Vector3f m_np;

    /// Relative refractive index in the sampled direction
    float m_probDensity;

    /// Measure associated with the sample
    EMeasure m_measure;

    EmitQueryRecord() : m_probDensity(1.0f), m_measure(EUnknownMeasure) {};

    /// Create a new record for sampling the BSDF
    EmitQueryRecord(const Vector3f& wi)
        : m_p(wi), m_probDensity(1.f), m_measure(EUnknownMeasure) { };

    /// Create a new record for querying the BSDF
    EmitQueryRecord(const Vector3f& wi,
        const Vector3f& wo, EMeasure measure)
        : m_p(wi), m_np(wo), m_measure(measure) { };
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    virtual void activate(Mesh *m)  = 0;
    virtual void sample(EmitQueryRecord& bRec, const Point2f& sample)  = 0;
    //virtual Color3f eval(const EmitQueryRecord& bRec)  = 0;
    //virtual float pdf(const EmitQueryRecord& bRec)  = 0;
    Color3f getEmission() const { return m_radiance; }
    DiscretePDF getDPDF() const { return m_dpdf; }
    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
protected: 
    Color3f m_radiance;
    DiscretePDF m_dpdf;
};

NORI_NAMESPACE_END
