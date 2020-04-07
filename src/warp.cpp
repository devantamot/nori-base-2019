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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#include <math.h> 

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = pow(sample[0], 0.5f);
    float theta = 2 * M_PI * sample[1];

    float x = cos(theta) * r;
    float y = sin(theta) * r;

    return Point2f(x, y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    if (pow(p[0], 2) + pow(p[1], 2)) return 1 / M_PI;
    return 0;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f& sample) {
    float radius = sqrt(sample.x());
    float phi = sample.y() * 2.0f * M_PI;
    return Vector3f(radius * cos(phi), radius * sin(phi), sqrt(1 - radius * radius));
}
float Warp::squareToCosineHemispherePdf(const Vector3f& v) {
    if (fabs(pow(v.x(),2) + pow(v.y(),2) + pow(v.z(),2) - 1) >= 1e-5 || v.z() <= 0)
        return 0.0f;
    return v.z() / M_PI;
}


Point2f Warp::ConcentricSampleDisk(const Point2f& u) {
    Point2f uOffset = 2.0f * u - Vector2f(1.0f, 1.0f);

    if (uOffset[0] == 0 && uOffset[1] == 0)
        return Point2f(0.0f, 0.0f);

    float theta, r;
    if (abs(uOffset[0]) > abs(uOffset[1])) {
        r = uOffset[0];
        theta = M_PI/4.0f * (uOffset[1] / uOffset[0]);
    }
    else {
        r = uOffset[1];
        theta = M_PI / 2.0f - M_PI / 4.0f * (uOffset[0] / uOffset[1]);
    }
    return r * Point2f(cos(theta), sin(theta));
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float cVal, sVal;
    if (sample.y() == 1.0f)cVal = 0.0f;
    else {
        float t1 = log(1.0f - sample.y()) * alpha * alpha;
        t1 = 1 - t1;
        cVal = sqrt(1.0f / t1);
    }
    float phi = 2.0f * M_PI * sample.x();
    sVal = sqrt(1 - cVal * cVal);
    return Vector3f(sVal * cos(phi), sVal * sin(phi), cVal);
}

float Warp::squareToBeckmannPdf(const Vector3f& m, float alpha) {
    if ((m.z() > 0) && (m.norm() <= 1)) {
        float theta = acos(m.z());
        float d = 0.5f / M_PI;
        float t1 = tan(theta) / alpha;
        float t2 = cos(theta);
        d *= 2.0f * exp(-t1 * t1);
        d /= alpha * alpha * t2 * t2 * t2;
        return d;
    }
    else return 0.0f;
}

NORI_NAMESPACE_END
