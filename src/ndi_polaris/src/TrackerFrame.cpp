#include <stdexcept>

#include <ndi_polaris/TrackerFrame.hpp>

TrackerFrame::TrackerFrame()
{
}


TrackerFrame::TrackerFrame( const TrackerFrame &frame ) :
    m_timestamp(frame.m_timestamp),
    m_targets(frame.m_targets)
{}


TrackerFrame::TrackerFrame( size_t timestamp, const std::vector< TrackerTarget > &targets )
{
    set( timestamp, targets );
}


TrackerFrame::~TrackerFrame()
{
}


size_t TrackerFrame::timestamp() const
{
    return m_timestamp;
}


const std::vector< TrackerTarget > TrackerFrame::targets()
{
    return m_targets;
}


void TrackerFrame::set( size_t timestamp, const std::vector<TrackerTarget> &targets )
{
    // check that the id's are in the correct order
    m_targets = targets;
    m_timestamp = timestamp;
}
