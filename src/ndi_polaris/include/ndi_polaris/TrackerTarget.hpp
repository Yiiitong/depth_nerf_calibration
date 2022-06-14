#pragma once

/*
 * TrackerTarget.hpp
 *
 *  Created on: Aug 11, 2011
 *      Author: duliu
 */

#include "blas.hpp"



//namespace eos
//{


class TrackerTarget
{
public:
    enum TrackingStatus{
        OK = 0,
        Partial = 1,
        Interference = 2,
        NO = 3
    };

public:
    TrackerTarget();
    TrackerTarget( const TrackerTarget& target );
    TrackerTarget( size_t id, const Eigen::Affine3f &trans, int status, float error );
    TrackerTarget( size_t id, const Eigen::Matrix4f &trans, int status, float error );
    TrackerTarget( size_t id, const Eigen::Quaternionf &rot, const Eigen::Vector3f &transl, int status, float error );
    ~TrackerTarget();

    // getters
    size_t id() const;
    const Eigen::Affine3f &trans() const;
    std::vector<float> mat();
    int status() const;
    float error() const;

//    bool isTracked() const;
    bool isTracked( float maxError ) const;

    // setters
    void set( size_t id );
    void set( size_t id, const Eigen::Affine3f &trans, int status, float error );
    void set( size_t id, const Eigen::Matrix4f &trans, int status, float error );
    void set( size_t id, const Eigen::Quaternionf &rot, const Eigen::Vector3f &transl, int status, float error );

protected:
    // target id - somewhere should be stored who this guy is
    size_t m_id;

    // transformations
    Eigen::Affine3f m_trans;

    // status and error
    int m_status;
    float m_error;

    friend class TrackingSystem;

};


//} // end namespace eos

