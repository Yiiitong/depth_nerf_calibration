#pragma once

/*
 * TrackingFrame.hpp
 *
 *  Created on: Aug 23, 2011
 *      Author: duliu
 */

#include <vector>

#include "TrackerTarget.hpp"

//namespace eos
//{


class TrackerFrame
{
public:
    TrackerFrame();
    TrackerFrame( const TrackerFrame &frame );
    TrackerFrame( size_t timestamp, const std::vector< TrackerTarget > &targets );
    ~TrackerFrame();

    size_t timestamp() const;
    //const std::vector< TrackerTarget > &targets() const;
    const std::vector< TrackerTarget > targets();

    void set( size_t timestamp, const std::vector< TrackerTarget > &targets );

protected:
    size_t m_timestamp;
    std::vector< TrackerTarget > m_targets;

    friend class TrackingSystem;
};


//} // end namespace eos

